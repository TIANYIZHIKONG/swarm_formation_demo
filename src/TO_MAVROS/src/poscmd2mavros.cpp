#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <cmath>
#include <string>

/**
 * poscmd2mavros
 * -------------
 * 把 ego-planner (traj_server) 输出的 quadrotor_msgs/PositionCommand (ENU, 世界系)
 * 转成 mavros_msgs/PositionTarget 发到 /mavros/setpoint_raw/local，
 * 并负责 OFFBOARD 模式切换 + 解锁 + 起飞/降落 的状态机。
 *
 * 坐标系: **不要在这里做 ENU->NED 转换!**
 * -------------------
 * mavros 的 setpoint_raw 插件本身就会把消息里的 position / velocity / yaw
 * 当成 **ENU** 再转成 NED 发给 PX4。实测(往 /mavros/setpoint_raw/local 发特征值,
 * 再读 PX4 的 uORB trajectory_setpoint):
 *
 *     发进去(pos, yaw)            PX4 收到
 *     (1, 2, 3),  0.70     ->     (2, 1, -3),  0.871 = pi/2 - 0.70
 *     (4, 5, 6),  1.10     ->     (5, 4, -6),  0.471 = pi/2 - 1.10
 *     (-7,-8,-9), -0.50    ->     (-8,-7, 9),  2.071 = pi/2 + 0.50
 *
 * 即  NED_x = ENU_y,  NED_y = ENU_x,  NED_z = -ENU_z,  yaw_ned = pi/2 - yaw_enu
 * (frame_id 填 "map" 还是留空, 结果一样)
 *
 * 所以这里直接发 **ENU** 值即可; 若再自己手动转一次, 就是"双重转换":
 * x/y 互换 + z 反号(向上变向下) -> PX4 以为你要往地下飞 -> 永远停在 landed,
 * 只给 ~0.2% 推力, 无人机根本起不来。这就是之前"解锁了但飞不起来"的根因。
 */
class PosCmdToMavros
{
public:
  PosCmdToMavros(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
  enum class Fsm
  {
    WAIT_FCU, // 等 mavros 连上飞控 / 等 OFFBOARD+armed
    HOLD,     // 已在 OFFBOARD，但还没收到 pos_cmd：原地悬停
    TAKEOFF,  // 起飞到 takeoff_height_
    TRACK,    // 跟踪 ego-planner 的 pos_cmd
    LAND,     // 轨迹完成后自动降落
    DONE
  };

  void stateCb(const mavros_msgs::State::ConstPtr &msg);
  void localPosCb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void posCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void timerCb(const ros::TimerEvent &e);

  void publishSetpoint(const Eigen::Vector3d &p_enu, const Eigen::Vector3d &v_enu,
                       double yaw_enu, double yaw_rate_enu);
  void requestOffboardAndArm();
  void switchTo(Fsm s);

  // ---------------- ros ----------------
  ros::Subscriber state_sub_, local_pos_sub_, poscmd_sub_;
  ros::Publisher setpoint_pub_;
  ros::ServiceClient arming_client_, set_mode_client_;
  ros::Timer timer_;

  // ---------------- 数据 ----------------
  mavros_msgs::State state_;
  bool have_state_{false};
  bool have_local_pos_{false};
  bool have_poscmd_{false};
  Eigen::Vector3d cur_pos_enu_{0.0, 0.0, 0.0};
  double cur_yaw_enu_{0.0};
  quadrotor_msgs::PositionCommand cmd_;
  ros::Time cmd_stamp_;

  // ---------------- 参数 ----------------
  double control_rate_{50.0};
  int control_mode_{0}; // 0: 位置+速度+yaw   1: 速度+yaw_rate
  bool auto_arm_{true};
  bool takeoff_enable_{false};
  double takeoff_height_{1.0};
  double takeoff_tol_{0.25};
  double cmd_timeout_{0.5};
  bool auto_land_on_finish_{true};
  double max_vel_{5.0};
  double max_yaw_rate_{1.0};
  double arm_retry_period_{1.0};
  // pos_cmd 所在的 "公共 world 系" 原点, 相对 mavros/FCU 本地 ENU 原点(本机起飞点)
  // 的位置. 多机编队时必须减掉它, 见 TRACK 里的注释. 单机时保持 (0,0,0).
  Eigen::Vector3d frame_origin_{0.0, 0.0, 0.0};

  // ---------------- 状态机 ----------------
  Fsm fsm_{Fsm::WAIT_FCU};
  Eigen::Vector3d takeoff_p_enu_{0.0, 0.0, 0.0};
  double takeoff_yaw_enu_{0.0};
  ros::Time last_arm_try_;
  bool landed_reported_{false};
};

PosCmdToMavros::PosCmdToMavros(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  pnh.param("control_rate", control_rate_, 50.0);
  pnh.param("control_mode", control_mode_, 0);
  pnh.param("auto_arm", auto_arm_, true);
  pnh.param("takeoff_enable", takeoff_enable_, false);
  pnh.param("takeoff_height", takeoff_height_, 1.0);
  pnh.param("takeoff_tol", takeoff_tol_, 0.25);
  pnh.param("cmd_timeout", cmd_timeout_, 0.5);
  pnh.param("auto_land_on_finish", auto_land_on_finish_, true);
  pnh.param("max_vel", max_vel_, 5.0);
  pnh.param("max_yaw_rate", max_yaw_rate_, 1.0);
  pnh.param("arm_retry_period", arm_retry_period_, 1.0);

  // 多机编队: pos_cmd 是"公共 world 系"的绝对坐标, 而 mavros 的
  // setpoint_raw/local 是"以本机起飞点(EKF origin)为原点"的本地 ENU 系.
  // 不告诉本节点本机起飞点在 world 系里的位置, 它就会把 world 绝对坐标
  // 当成本地坐标发下去 -> 第 i 架整机平移 S_i (单机 S_i=0, 所以以前没暴露).
  pnh.param("origin_x", frame_origin_(0), 0.0);
  pnh.param("origin_y", frame_origin_(1), 0.0);
  pnh.param("origin_z", frame_origin_(2), 0.0);

  std::string poscmd_topic, state_topic, local_pos_topic;
  pnh.param<std::string>("state_topic", state_topic, std::string("/mavros/state"));
  pnh.param<std::string>("local_pos_topic", local_pos_topic,
                         std::string("/mavros/local_position/pose"));
  pnh.param<std::string>("poscmd_topic", poscmd_topic,
                         std::string("/drone_0_planning/pos_cmd"));

  state_sub_ = nh.subscribe(state_topic, 10, &PosCmdToMavros::stateCb, this);
  local_pos_sub_ = nh.subscribe(local_pos_topic, 10, &PosCmdToMavros::localPosCb, this);
  poscmd_sub_ = nh.subscribe(poscmd_topic, 50, &PosCmdToMavros::posCmdCb, this,
                             ros::TransportHints().tcpNoDelay());

  setpoint_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  if (control_rate_ < 10.0)
  {
    ROS_WARN("[poscmd2mavros] control_rate=%.1f 太低，强制改为 50 Hz (PX4 要求 > 2Hz)", control_rate_);
    control_rate_ = 50.0;
  }
  timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &PosCmdToMavros::timerCb, this);

  ROS_INFO("[poscmd2mavros] pos_cmd=%s  control_rate=%.1f  control_mode=%d",
           poscmd_topic.c_str(), control_rate_, control_mode_);
  ROS_INFO("[poscmd2mavros] auto_arm=%d  takeoff_enable=%d  takeoff_height=%.2f",
           (int)auto_arm_, (int)takeoff_enable_, takeoff_height_);
}

void PosCmdToMavros::stateCb(const mavros_msgs::State::ConstPtr &msg)
{
  state_ = *msg;
  have_state_ = true;
}

void PosCmdToMavros::localPosCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  cur_pos_enu_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  // mavros 的 local_position/pose 是 ENU 约定
  const geometry_msgs::Quaternion &q = msg->pose.orientation;
  tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  cur_yaw_enu_ = yaw;

  have_local_pos_ = true;
}

void PosCmdToMavros::posCmdCb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  cmd_ = *msg;
  cmd_stamp_ = ros::Time::now();
  have_poscmd_ = true;
}

void PosCmdToMavros::switchTo(Fsm s)
{
  fsm_ = s;
}

void PosCmdToMavros::requestOffboardAndArm()
{
  const ros::Time now = ros::Time::now();
  if (!last_arm_try_.isZero() && (now - last_arm_try_) < ros::Duration(arm_retry_period_))
    return;
  last_arm_try_ = now;

  if (state_.mode != "OFFBOARD")
  {
    mavros_msgs::SetMode sm;
    sm.request.base_mode = 0;
    sm.request.custom_mode = "OFFBOARD";
    if (set_mode_client_.call(sm) && sm.response.mode_sent)
      ROS_INFO("[poscmd2mavros] OFFBOARD requested");
    else
      ROS_WARN("[poscmd2mavros] OFFBOARD request failed (PX4 必须先收到 >2Hz 的 setpoint)");
    return;
  }

  if (!state_.armed)
  {
    mavros_msgs::CommandBool cb;
    cb.request.value = true;
    if (arming_client_.call(cb) && cb.response.success)
      ROS_INFO("[poscmd2mavros] arming requested");
    else
      ROS_WARN("[poscmd2mavros] arming failed (检查 EKF / 是否已预解锁)");
  }
}

void PosCmdToMavros::publishSetpoint(const Eigen::Vector3d &p_enu, const Eigen::Vector3d &v_enu,
                                     double yaw_enu, double yaw_rate_enu)
{
  // 直接发 ENU, 让 mavros 去做 ENU->NED (原因见文件头注释)
  mavros_msgs::PositionTarget sp;
  sp.header.stamp = ros::Time::now();
  sp.header.frame_id = "map"; // ENU 世界系, 与 FAST-LIO / ego-planner 一致
  sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // FCU 侧目标坐标系

  if (control_mode_ == 1)
  {
    // 速度 + yaw_rate (ENU)
    sp.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                   mavros_msgs::PositionTarget::IGNORE_PY |
                   mavros_msgs::PositionTarget::IGNORE_PZ |
                   mavros_msgs::PositionTarget::IGNORE_AFX |
                   mavros_msgs::PositionTarget::IGNORE_AFY |
                   mavros_msgs::PositionTarget::IGNORE_AFZ |
                   mavros_msgs::PositionTarget::IGNORE_YAW;
    sp.velocity.x = v_enu(0);
    sp.velocity.y = v_enu(1);
    sp.velocity.z = v_enu(2);
    sp.yaw_rate = yaw_rate_enu;
  }
  else
  {
    // 位置 + 速度前馈 + yaw (ENU)  —— PX4 位置环闭环, 推荐
    sp.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                   mavros_msgs::PositionTarget::IGNORE_AFY |
                   mavros_msgs::PositionTarget::IGNORE_AFZ |
                   mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    sp.position.x = p_enu(0);
    sp.position.y = p_enu(1);
    sp.position.z = p_enu(2);
    sp.velocity.x = v_enu(0);
    sp.velocity.y = v_enu(1);
    sp.velocity.z = v_enu(2);
    sp.yaw = yaw_enu;
  }

  setpoint_pub_.publish(sp);
}

void PosCmdToMavros::timerCb(const ros::TimerEvent &)
{
  if (!have_local_pos_)
    return;

  switch (fsm_)
  {
  case Fsm::WAIT_FCU:
  {
    // 先以当前位置为 setpoint 持续发送，满足 PX4 进入 OFFBOARD 的前置条件(>2Hz)
    publishSetpoint(cur_pos_enu_, Eigen::Vector3d::Zero(), cur_yaw_enu_, 0.0);

    if (!have_state_ || !state_.connected)
      return;

    if (auto_arm_)
      requestOffboardAndArm();

    if (state_.mode == "OFFBOARD" && state_.armed)
    {
      if (takeoff_enable_)
      {
        takeoff_p_enu_ = cur_pos_enu_ + Eigen::Vector3d(0.0, 0.0, takeoff_height_);
        takeoff_yaw_enu_ = cur_yaw_enu_;
        ROS_INFO("[poscmd2mavros] OFFBOARD+armed -> TAKEOFF to (%.2f, %.2f, %.2f)",
                 takeoff_p_enu_(0), takeoff_p_enu_(1), takeoff_p_enu_(2));
        switchTo(Fsm::TAKEOFF);
      }
      else
      {
        ROS_INFO("[poscmd2mavros] OFFBOARD+armed -> HOLD (waiting pos_cmd)");
        switchTo(Fsm::HOLD);
      }
    }
    break;
  }

  case Fsm::HOLD:
  {
    // 还没收到 pos_cmd：原地悬停
    publishSetpoint(cur_pos_enu_, Eigen::Vector3d::Zero(), cur_yaw_enu_, 0.0);

    if (!state_.armed || state_.mode != "OFFBOARD")
    {
      switchTo(Fsm::WAIT_FCU);
      break;
    }
    if (have_poscmd_)
    {
      ROS_INFO("[poscmd2mavros] first pos_cmd received -> TRACK");
      switchTo(Fsm::TRACK);
    }
    break;
  }

  case Fsm::TAKEOFF:
  {
    publishSetpoint(takeoff_p_enu_, Eigen::Vector3d::Zero(), takeoff_yaw_enu_, 0.0);

    if (!state_.armed || state_.mode != "OFFBOARD")
    {
      switchTo(Fsm::WAIT_FCU);
      break;
    }
    if ((cur_pos_enu_ - takeoff_p_enu_).norm() < takeoff_tol_)
    {
      ROS_INFO("[poscmd2mavros] takeoff reached (z=%.2f) -> TRACK", cur_pos_enu_(2));
      switchTo(Fsm::TRACK);
    }
    break;
  }

  case Fsm::TRACK:
  {
    if (!state_.armed || state_.mode != "OFFBOARD")
    {
      ROS_WARN("[poscmd2mavros] lost OFFBOARD/armed -> back to WAIT_FCU");
      switchTo(Fsm::WAIT_FCU);
      break;
    }

    const bool fresh =
        have_poscmd_ && (ros::Time::now() - cmd_stamp_) < ros::Duration(cmd_timeout_);
    if (!fresh)
    {
      // 轨迹断了就原地悬停，避免把过期指令喂给飞控
      publishSetpoint(cur_pos_enu_, Eigen::Vector3d::Zero(), cur_yaw_enu_, 0.0);
      break;
    }

    // pos_cmd 是"公共 world 系"的绝对坐标(Swarm-Formation 的编队目标、
    // 机间避让都在这个系里算), 而 mavros 的 setpoint_raw/local 是"以本机
    // 起飞点(EKF origin)为原点"的本地 ENU 系 -> 必须减掉本机起飞点在 world
    // 系里的位置, 否则第 i 架会整体平移 S_i (单机时 S_i=(0,0,0), 无影响).
    const Eigen::Vector3d p(cmd_.position.x - frame_origin_(0),
                            cmd_.position.y - frame_origin_(1),
                            cmd_.position.z - frame_origin_(2));
    Eigen::Vector3d v(cmd_.velocity.x, cmd_.velocity.y, cmd_.velocity.z);
    if (max_vel_ > 0.0 && v.norm() > max_vel_)
      v = v / v.norm() * max_vel_;

    double yr = cmd_.yaw_dot;
    if (max_yaw_rate_ > 0.0 && std::fabs(yr) > max_yaw_rate_)
      yr = std::copysign(max_yaw_rate_, yr);

    publishSetpoint(p, v, cmd_.yaw, yr);

    if (cmd_.trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED)
    {
      if (auto_land_on_finish_)
      {
        ROS_INFO("[poscmd2mavros] trajectory COMPLETED -> LAND");
        switchTo(Fsm::LAND);
      }
      else
      {
        ROS_INFO("[poscmd2mavros] trajectory COMPLETED -> HOLD");
        have_poscmd_ = false;
        switchTo(Fsm::HOLD);
      }
    }
    break;
  }

  case Fsm::LAND:
  {
    publishSetpoint(cur_pos_enu_, Eigen::Vector3d::Zero(), cur_yaw_enu_, 0.0);

    mavros_msgs::SetMode sm;
    sm.request.base_mode = 0;
    sm.request.custom_mode = "AUTO.LAND";
    set_mode_client_.call(sm);

    if (state_.mode == "AUTO.LAND" && !state_.armed && !landed_reported_)
    {
      landed_reported_ = true;
      ROS_INFO("[poscmd2mavros] landed, done.");
      switchTo(Fsm::DONE);
    }
    break;
  }

  case Fsm::DONE:
  default:
    break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poscmd2mavros");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PosCmdToMavros node(nh, pnh);
  ros::spin();
  return 0;
}
