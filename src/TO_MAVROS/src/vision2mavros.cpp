#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <deque>
#include <string>

class VisionToMavros {
public:
    VisionToMavros(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : init_vision_flag_(false),vcnt(0)
    {
        // 参数
        double publish_rate;
        std::string vision_pose_topic;
        pnh.param<double>("publish_rate", publish_rate, 30.0);
        pnh.param<std::string>("vision_pose_topic",vision_pose_topic,std::string("/Odometry"));

        //  body -> lidar
        double body_lidar_x, body_lidar_y, body_lidar_z;
        pnh.param<double>("body_lidar_x", body_lidar_x, 0.0); 
        pnh.param<double>("body_lidar_y", body_lidar_y, 0.0);
        pnh.param<double>("body_lidar_z", body_lidar_z, 0.0);
        t_body_lidar_ = Eigen::Vector3d(body_lidar_x, body_lidar_y, body_lidar_z);

        p_vision_body_.setZero();
        q_vision_body_.getIdentity();

        // 订阅与发布
        vision_sub_ = pnh.subscribe(vision_pose_topic, 10, &VisionToMavros::visionCallback, this);
        vision_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

        // 定时器
        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate),&VisionToMavros::publishVisionPose, this);

        // ROS_INFO("FastLIO to MAVROS converter initialized");
        // ROS_INFO("Window size: %d, Publish rate: %.2f Hz", window_size, publish_rate);
    }

private:

    // void visionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        
    //     // 位置
    //     p_vision_body_(0) = msg->pose.pose.position.x;
    //     p_vision_body_(1) = msg->pose.pose.position.y;
    //     p_vision_body_(2) = msg->pose.pose.position.z;

    //     // 姿态
    //     q_vision_body_.setX(msg->pose.pose.orientation.x);
    //     q_vision_body_.setY(msg->pose.pose.orientation.y);
    //     q_vision_body_.setZ(msg->pose.pose.orientation.z);
    //     q_vision_body_.setW(msg->pose.pose.orientation.w);

    //     // 欧拉角
    //     double cur_r, cur_p, cur_y;
    //     tf::Matrix3x3(tf::Quaternion(
    //         msg->pose.pose.orientation.x,
    //         msg->pose.pose.orientation.y,
    //         msg->pose.pose.orientation.z,
    //         msg->pose.pose.orientation.w
    //     )).getRPY(cur_r, cur_p, cur_y);

    //     if(!init_vision_flag_){
    //         p_vision_delta_(0)+=p_vision_body_(0);
    //         p_vision_delta_(1)+=p_vision_body_(1);
    //         p_vision_delta_(2)+=p_vision_body_(2);

    //         // 新增：rpy 累加
    //         rpy_delta_(0) += cur_r;
    //         rpy_delta_(1) += cur_p;
    //         rpy_delta_(2) += cur_y;

    //         vcnt++;
    //         if(vcnt>=100)
    //         {
    //             p_vision_delta_/=100;
    //             rpy_delta_ = rpy_delta_ / 100.0;
    //             init_vision_flag_=true;
    //         }
    //     }
    // }
    void visionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 1. 先把 FastLIO 的输出当成“雷达在 map 下”的位姿读进来
        Eigen::Vector3d p_lidar;
        p_lidar(0) = msg->pose.pose.position.x;
        p_lidar(1) = msg->pose.pose.position.y;
        p_lidar(2) = msg->pose.pose.position.z;

        q_vision_body_.setX(msg->pose.pose.orientation.x);
        q_vision_body_.setY(msg->pose.pose.orientation.y);
        q_vision_body_.setZ(msg->pose.pose.orientation.z);
        q_vision_body_.setW(msg->pose.pose.orientation.w);

        // 2. 由雷达姿态算旋转矩阵 R_map_lidar
        tf::Matrix3x3 R_tf(q_vision_body_);
        Eigen::Matrix3d R_map_lidar;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R_map_lidar(i,j) = R_tf[i][j];



        // 4. 计算机体在 map 下的位置：p_body = p_lidar - R * (0.05, 0, 0)
        Eigen::Vector3d p_body = p_lidar - R_map_lidar * t_body_lidar_;

        // 5. 把“机体位置”存入 p_vision_body_，后面所有代码都当成机体用
        p_vision_body_ = p_body;

        // 6. 机体当前欧拉角（用雷达四元数表示，两者平行安装就一样）
        double cur_r, cur_p, cur_y;
        tf::Matrix3x3(q_vision_body_).getRPY(cur_r, cur_p, cur_y);

        // 7. 初始化阶段：对“机体位姿”做前 100 帧平均，作为零点
        if (!init_vision_flag_) {
            p_vision_delta_ += p_vision_body_;
            rpy_delta_(0)   += cur_r;
            rpy_delta_(1)   += cur_p;
            rpy_delta_(2)   += cur_y;

            vcnt++;
            if (vcnt >= 100) {
                p_vision_delta_ /= 100.0;
                rpy_delta_      /= 100.0;
                init_vision_flag_ = true;
            }
        }
    }

    void publishVisionPose(const ros::TimerEvent&) {
        if (!init_vision_flag_) return;

        //1.发布map下的位姿
        geometry_msgs::PoseStamped vision_msg;
        vision_msg.header.stamp = ros::Time::now();
        vision_msg.header.frame_id = "map";
        //位置消除偏差
        Eigen::Vector3d p_map_ = p_vision_body_-p_vision_delta_;

        vision_msg.pose.position.x = p_map_(0);
        vision_msg.pose.position.y = p_map_(1);
        vision_msg.pose.position.z = p_map_(2);        

        // 姿态消除误差
        double cur_r, cur_p, cur_y;
        tf::Matrix3x3(q_vision_body_).getRPY(cur_r, cur_p, cur_y);
        cur_r -= rpy_delta_(0);
        cur_p -= rpy_delta_(1);
        cur_y -= rpy_delta_(2);

        tf::Quaternion q_map_;
        q_map_.setRPY(cur_r, cur_p, cur_y);

        vision_msg.pose.orientation.x = q_map_.x();
        vision_msg.pose.orientation.y = q_map_.y();
        vision_msg.pose.orientation.z = q_map_.z();
        vision_msg.pose.orientation.w = q_map_.w();

        vision_pub_.publish(vision_msg);


        ROS_INFO("\nposition in map_:\n   x: %.6f\n   y: %.6f\n   z: %.6f\norientation of body:\n   x: %.6f\n   y: %.6f\n   z: %.6f\n   w: %.6f", \
            p_map_[0],p_map_[1],p_map_(2),q_map_.x(),q_map_.y(),q_map_.z(),q_map_.w());


    }

private:
    ros::Subscriber vision_sub_;
    ros::Publisher vision_pub_;
    ros::Timer timer_;

    Eigen::Vector3d p_vision_body_;
    tf::Quaternion q_vision_body_;

    bool init_vision_flag_;
    int vcnt;
    Eigen::Vector3d p_vision_delta_{0,0,0};
    Eigen::Vector3d rpy_delta_{0,0,0};

    Eigen::Vector3d t_body_lidar_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_to_mavros");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    VisionToMavros converter(nh, pnh);
    ros::spin();    
    return 0;
}
