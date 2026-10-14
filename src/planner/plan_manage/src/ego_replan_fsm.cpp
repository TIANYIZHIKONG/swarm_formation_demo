

#include <plan_manage/ego_replan_fsm.h>
#include <cmath>

namespace ego_planner
{
  EGOReplanFSM::~EGOReplanFSM()
  {
    result_file_.close();
  }
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    flag_relan_astar_ = false;
    have_local_traj_ = false;

    /* IMPORTANT: initialize the odometry state. Before the first odometry message
       arrives these values may already be used to build the global trajectory
       (e.g. formationWaypointCallback). Uninitialized values would produce a
       garbage goal/duration and, downstream, a huge (multi-GB) allocation. */
    odom_pos_.setZero();
    odom_vel_.setZero();
    odom_acc_.setZero();
    odom_orient_.setIdentity();

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/result_file", result_fn_, string("/home/zuzu/Documents/Benchmark/21-RSS-ego-swarm/2.24/ego/ego_swarm.txt"));
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    nh.param("fsm/goal_num", goal_num_, -1);
    for (int i = 0; i < goal_num_; i++)
    {
      nh.param("fsm/target" + to_string(i) + "_x", goalpoints_[i][0], -1.0);
      nh.param("fsm/target" + to_string(i) + "_y", goalpoints_[i][1], -1.0);
      nh.param("fsm/target" + to_string(i) + "_z", goalpoints_[i][2], -1.0);
    }

    for (int i = 0; i < 50; i++)
    {
      nh.param("global_goal/relative_pos_" + to_string(i) + "/x", swarm_relative_pts_[i][0], 0.0);
      nh.param("global_goal/relative_pos_" + to_string(i) + "/y", swarm_relative_pts_[i][1], 0.0);
      nh.param("global_goal/relative_pos_" + to_string(i) + "/z", swarm_relative_pts_[i][2], 0.0);
    }

    nh.param("global_goal/swarm_scale", swarm_scale_, 1.0);

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::PolyTraj>("planning/broadcast_traj_recv", 100,
                                                                 &EGOReplanFSM::RecvBroadcastPolyTrajCallback,
                                                                 this,
                                                                 ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);

    result_file_.open(result_fn_, ios::app);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
      planGlobalTrajbyGivenWps();
    }
    else if (target_type_ == TARGET_TYPE::SWARM_MANUAL_TARGET)
    {
      central_goal = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::formationWaypointCallback, this);
    }
    cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        bool success = planFromGlobalTraj(1);

        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else if (timesOfConsecutiveStateCalls().first < max_first_traj_attempts_)
        {
          ROS_ERROR("Failed to generate the first trajectory!!! (attempt %d/%d)",
                    timesOfConsecutiveStateCalls().first, max_first_traj_attempts_);
          changeFSMExecState(SEQUENTIAL_START, "FSM");
        }
        else
        {
          /* Give up on this target instead of re-entering this state forever.
             The old code called changeFSMExecState(SEQUENTIAL_START) here, i.e. it
             re-entered itself on every 10 ms tick; a target that cannot be planned
             (e.g. the drone is parked outside the map / inside an obstacle) then
             kept the FSM in SEQUENTIAL_START indefinitely, and because a new RViz
             goal was only honoured from WAIT_TARGET / EXEC_TRAJ, the drone ignored
             every further goal and hovered in place. Dropping the target and going
             back to WAIT_TARGET keeps the node responsive: the next goal triggers a
             fresh plan (and the drone hovers until then). */
          ROS_ERROR("Failed to generate the first trajectory %d times in a row, dropping the "
                    "target [%f, %f, %f] (odom [%f, %f, %f]). Back to WAIT_TARGET.",
                    timesOfConsecutiveStateCalls().first,
                    end_pt_(0), end_pt_(1), end_pt_(2),
                    odom_pos_(0), odom_pos_(1), odom_pos_(2));
          have_target_ = false;
          have_new_target_ = false;
          have_local_traj_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      bool success = planFromGlobalTraj(1);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      bool success;
      if (flag_relan_astar_)
        success = planFromLocalTraj(true, false);
      else
        success = planFromLocalTraj(false, true);

      if (success)
      {
        flag_relan_astar_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        flag_relan_astar_ = true;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);

      if ((local_target_pt_ - end_pt_).norm() < 0.1) // local target close to the global target
      {
        if (t_cur > info->duration - 0.2)
        {
          have_target_ = false;
          have_local_traj_ = false;

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");

          result_file_ << planner_manager_->pp_.drone_id << "\t" << (ros::Time::now() - planner_manager_->global_start_time_).toSec() << "\t" << planner_manager_->average_plan_time_ << "\n";

          printf("\033[47;30m\n[drone %d reached goal]==============================================\033[0m\n",
                 planner_manager_->pp_.drone_id);
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        /* Recover as soon as the drone has come to a stop. NOTE: enable_fail_safe_
           is forced to false on a depth timeout ("Depth Lost!"), which used to
           latch this state FOREVER - the drone then hovered and silently ignored
           every further RViz goal (one of the reported "stuck after a collision"
           symptoms). An explicit new goal (have_new_target_) is an operator request
           to resume, so re-arm the recovery here; the velocity check below still
           guarantees the drone is at rest before a new trajectory is planned. */
        if (!enable_fail_safe_ && have_new_target_)
        {
          ROS_WARN("[drone %d] New goal received while in EMERGENCY_STOP: re-arming the "
                   "fail-safe recovery.", planner_manager_->pp_.drone_id);
          enable_fail_safe_ = true;
        }

        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    Eigen::Vector3d p_cur = info->traj.getPos(t_cur);
    /* NOTE: 0.8*swarm_clearance is physically meaningless as an emergency
       threshold: with swarm_clearance = 0.5 it is 0.4 m, which is *inside* the
       distance at which two iris propellers already touch
       (2 x (0.22 + 0.128) = 0.696 m -> the warning fired after contact).
       Take the physical contact distance plus a margin as a lower bound. */
    constexpr double SWARM_CONTACT_DIST = 0.70; // [m] 2 x prop tip radius (iris)
    const double CLEARANCE = std::max(SWARM_CONTACT_DIST + 0.15,
                                      0.8 * planner_manager_->getSwarmClearance());
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = planner_manager_->ploy_traj_opt_->getCollisionCheckTimeEnd();
    double t_temp;
    bool occ = false;
    for (double t = t_cur; t < info->duration; t += time_step)
    {
      // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      if (t_cur < t_2_3 && t >= t_2_3)
        break;

      if (map->getInflateOccupancy(info->traj.getPos(t)) == 1)
      {
        ROS_WARN("drone %d is too close to the obstacle at relative time %f!",
                 planner_manager_->pp_.drone_id, t / info->duration);
        t_temp = t;
        occ = true;
        break;
      }

      for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
      {
        if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
            (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->traj_.swarm_traj.at(id).start_time;

        if (t_X > planner_manager_->traj_.swarm_traj.at(id).duration)
          continue;

        Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                   planner_manager_->pp_.drone_id, id, dist);
          t_temp = t;
          occ = true;
          break;
        }
      }
    }

    if (occ)
    {
      /* Handle the collided case immediately */
      ROS_INFO("Try to replan a safe trajectory");
      if (planFromLocalTraj(true, false)) // Make a chance
      // if (planFromLocalTraj(false, true))
      {
        ROS_INFO("Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }
      else
      {
        if (t_temp - t_cur < emergency_time_) // 1.0s of emergency time
        {
          ROS_WARN("Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }
        else
        {
          ROS_WARN("current traj in collision, replan.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    /* Guard: without a valid odometry odom_pos_ is meaningless, and using it to
       build the global trajectory produces garbage (huge duration) downstream. */
    if (!have_odom_ || !odom_pos_.allFinite())
    {
      ROS_WARN("[drone %d] Goal rejected: odometry not ready or not finite.",
               planner_manager_->pp_.drone_id);
      return;
    }

    /* Guard: the incoming goal itself must be finite. */
    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y))
    {
      ROS_ERROR("[drone %d] Goal rejected: non-finite goal position (%f, %f).",
                planner_manager_->pp_.drone_id, msg->pose.position.x, msg->pose.position.y);
      return;
    }

    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->pose.position.x, msg->pose.position.y, 1.0;

    /* Keep the goal inside the map. Outside it the occupancy grid reports -1 =
       "occupied" everywhere, so the goal can never be reached by the planner --
       and once the drone has actually flown there (which is what happened in the
       failing run: the previous goal was at y = -16 while the map only spans
       y in [-15, 15]) no further goal can be planned either: the A* start-point
       escape loop runs out of the A* pool ("Ran out of pool, index=508 -1 22"),
       A* returns an empty path and the planner node used to die on it. */
    if (planner_manager_->grid_map_)
    {
      const Eigen::Vector3d goal_clamped = planner_manager_->grid_map_->clampToMapBounds(end_pt_);
      if ((goal_clamped - end_pt_).norm() > 1e-6)
      {
        ROS_WARN("[drone %d] Goal [%f, %f, %f] is outside the map, clamped to [%f, %f, %f].",
                 planner_manager_->pp_.drone_id,
                 end_pt_(0), end_pt_(1), end_pt_(2),
                 goal_clamped(0), goal_clamped(1), goal_clamped(2));
        end_pt_ = goal_clamped;
      }
    }

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);

    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      std::vector<Eigen::Vector3d> gloabl_traj;
      if (sampleGlobalTrajForVis(gloabl_traj))
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      /* A new goal is always honoured, whatever the FSM is doing at that moment.
         Previously only WAIT_TARGET and EXEC_TRAJ reacted to a goal, so if the FSM
         was sitting in SEQUENTIAL_START (failed first trajectory), GEN_NEW_TRAJ or
         REPLAN_TRAJ (repeated planning failure), every further RViz goal was
         silently ignored and the drone stayed where it was - exactly the reported
         "cannot accept a new waypoint after a collision" behaviour.
         EMERGENCY_STOP keeps its own recovery logic (it already stores the new goal
         and switches to GEN_NEW_TRAJ itself once the drone has stopped). */
      if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      else if (exec_state_ != EMERGENCY_STOP)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg)
  {
    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
               msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Validate the received drone id BEFORE using it as a container size/index:
       swarm_traj is grown up to recv_id below, so an invalid (negative or huge)
       id coming from the network would allocate an unbounded vector. */
    constexpr int max_drone_num = 50;
    if (msg->drone_id < 0 || msg->drone_id >= max_drone_num)
    {
      ROS_ERROR("Received a broadcast trajectory with invalid drone_id = %d, dropped!", msg->drone_id);
      return;
    }

    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id)
      return;

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i)
    {
      int i6 = i * 6;
      cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
          msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
      cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
          msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
      cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
          msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

      dura[i] = msg->duration[i];
    }

    poly_traj::Trajectory trajectory(dura, cMats);
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            break;
          }

          have_recv_pre_agent_ = true;
        }
      }
    }
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &msg)
  {

    auto data = &planner_manager_->traj_.local_traj;

    msg.drone_id = planner_manager_->pp_.drone_id;
    msg.traj_id = data->traj_id;
    msg.start_time = ros::Time(data->start_time);
    msg.order = 5; // todo, only support order = 5 now.

    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();
    msg.duration.resize(piece_num);
    msg.coef_x.resize(6 * piece_num);
    msg.coef_y.resize(6 * piece_num);
    msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        msg.coef_x[i6 + j] = cMat(0, j);
        msg.coef_y[i6 + j] = cMat(1, j);
        msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
  }
  
  /* Sample the global trajectory (every 0.1 s) for visualization.
     The size of the sampled list is derived from global_traj.duration, which is
     NOT trustworthy: if it is uninitialized / non-finite / absurdly large (odometry
     jump, invalid goal, ...), floor(duration/0.1) becomes a huge positive number and
     std::vector<Eigen::Vector3d> tries to allocate tens of GB, which got the planner
     node OOM-killed (observed total-vm ~= 39 GB, anon-rss ~= 23 GB). Therefore the
     duration is validated and the number of points is hard-capped here. */
  bool EGOReplanFSM::sampleGlobalTrajForVis(std::vector<Eigen::Vector3d> &gloabl_traj)
  {
    constexpr double step_size_t = 0.1;

    gloabl_traj.clear();

    const double duration = planner_manager_->traj_.global_traj.duration;
    if (!std::isfinite(duration) || duration < 0.0 || duration > max_global_duration_)
    {
      ROS_ERROR("[drone %d] Abnormal global trajectory duration = %f s. "
                "odom_pos = [%f, %f, %f], end_pt = [%f, %f, %f]. Global path visualization skipped.",
                planner_manager_->pp_.drone_id, duration,
                odom_pos_(0), odom_pos_(1), odom_pos_(2),
                end_pt_(0), end_pt_(1), end_pt_(2));
      return false;
    }

    int i_end = (int)floor(duration / step_size_t);
    if (i_end > max_global_path_vis_pts_)
    {
      ROS_WARN("[drone %d] Global trajectory duration %f s gives %d sample points, "
               "truncated to %d for visualization.",
               planner_manager_->pp_.drone_id, duration, i_end, max_global_path_vis_pts_);
      i_end = max_global_path_vis_pts_;
    }

    gloabl_traj.resize(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
    }

    return true;
  }

  void EGOReplanFSM::formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    /* Guard 1: without a valid odometry odom_pos_ is meaningless, and using it to
       build the global trajectory produces garbage (huge duration) downstream. */
    if (!have_odom_ || !odom_pos_.allFinite())
    {
      ROS_WARN("[drone %d] Goal rejected: odometry not ready or not finite.",
               planner_manager_->pp_.drone_id);
      return;
    }

    /* Guard 2: the incoming goal itself must be finite. */
    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y))
    {
      ROS_ERROR("[drone %d] Goal rejected: non-finite goal position (%f, %f).",
                planner_manager_->pp_.drone_id, msg->pose.position.x, msg->pose.position.y);
      return;
    }

    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;

    bool success = false;
    swarm_central_pos_(0) = msg->pose.position.x;
    swarm_central_pos_(1) = msg->pose.position.y;
    swarm_central_pos_(2) = 0.5;

    int id = planner_manager_->pp_.drone_id;

    Eigen::Vector3d relative_pos;
    relative_pos << swarm_relative_pts_[id][0],
                    swarm_relative_pts_[id][1],
                    swarm_relative_pts_[id][2];
    end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;

    /* Guard 3: the resulting formation target must be finite and reachable. */
    if (!end_pt_.allFinite() || (end_pt_ - odom_pos_).norm() > max_goal_distance_)
    {
      ROS_ERROR("[drone %d] Goal rejected: bad formation target [%f, %f, %f] from relative_pos "
                "[%f, %f, %f], odom [%f, %f, %f], scale %f.",
                id, end_pt_(0), end_pt_(1), end_pt_(2),
                relative_pos(0), relative_pos(1), relative_pos(2),
                odom_pos_(0), odom_pos_(1), odom_pos_(2), swarm_scale_);
      return;
    }

    /* Guard 4: keep the WHOLE formation inside the map. Outside the map the
       occupancy grid reports -1 = "occupied" everywhere, so the local planner can
       not plan there; a drone that ends up outside can then no longer be replanned
       to any goal (A* start-point escape loop -> "Ran out of pool" -> empty path).
       The formation CENTRE is shifted, never the individual targets: clamping each
       drone's target separately squashes the formation onto the boundary, so two
       drones whose offsets differ only along the clamped axis would be commanded to
       the very same point (observed: 0.84 m apart -> permanent swarm "too close"
       warnings + EMERGENCY_STOP). Every drone reads the same relative_pos_* params,
       so all of them compute the same shifted centre and the formation stays rigid. */
    if (planner_manager_->grid_map_)
    {
      const Eigen::Vector3d bound_lo = planner_manager_->grid_map_->getClampMinBound();
      const Eigen::Vector3d bound_hi = planner_manager_->grid_map_->getClampMaxBound();

      /* bounding box of all (scaled) formation offsets: slot i is at
         centre + swarm_scale_ * relative_pos_i, unused slots are (0,0,0) */
      double off_min_x = swarm_scale_ * swarm_relative_pts_[0][0], off_max_x = off_min_x;
      double off_min_y = swarm_scale_ * swarm_relative_pts_[0][1], off_max_y = off_min_y;
      for (int i = 1; i < 50; ++i)
      {
        const double ox = swarm_scale_ * swarm_relative_pts_[i][0];
        const double oy = swarm_scale_ * swarm_relative_pts_[i][1];
        off_min_x = std::min(off_min_x, ox);
        off_max_x = std::max(off_max_x, ox);
        off_min_y = std::min(off_min_y, oy);
        off_max_y = std::max(off_max_y, oy);
      }

      /* allowed centre interval per axis; if the formation is wider than the map
         the interval is empty and its centre is used (per-drone clamp below then
         acts as the last resort) */
      const double cx_lo = bound_lo(0) - off_min_x, cx_hi = bound_hi(0) - off_max_x;
      const double cy_lo = bound_lo(1) - off_min_y, cy_hi = bound_hi(1) - off_max_y;
      const double cx = (cx_lo <= cx_hi) ? std::min(std::max(swarm_central_pos_(0), cx_lo), cx_hi)
                                         : 0.5 * (cx_lo + cx_hi);
      const double cy = (cy_lo <= cy_hi) ? std::min(std::max(swarm_central_pos_(1), cy_lo), cy_hi)
                                         : 0.5 * (cy_lo + cy_hi);

      if (std::abs(cx - swarm_central_pos_(0)) > 1e-6 ||
          std::abs(cy - swarm_central_pos_(1)) > 1e-6)
      {
        ROS_WARN("[drone %d] Formation centre [%f, %f, %f] would push the formation outside the "
                 "map (extent x=[%f, %f] y=[%f, %f]), shifted the whole formation to centre "
                 "[%f, %f, %f]. This drone: [%f, %f, %f].",
                 id, swarm_central_pos_(0), swarm_central_pos_(1), swarm_central_pos_(2),
                 off_min_x, off_max_x, off_min_y, off_max_y,
                 cx, cy, swarm_central_pos_(2),
                 cx + swarm_scale_ * relative_pos(0), cy + swarm_scale_ * relative_pos(1),
                 swarm_central_pos_(2) + swarm_scale_ * relative_pos(2));
      }

      swarm_central_pos_(0) = cx;
      swarm_central_pos_(1) = cy;
      end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;

      /* last resort: a formation wider than the map cannot be kept rigid */
      const Eigen::Vector3d goal_clamped = planner_manager_->grid_map_->clampToMapBounds(end_pt_);
      if ((goal_clamped - end_pt_).norm() > 1e-6)
      {
        ROS_WARN("[drone %d] Formation goal [%f, %f, %f] is still outside the map, clamped to "
                 "[%f, %f, %f].",
                 id, end_pt_(0), end_pt_(1), end_pt_(2),
                 goal_clamped(0), goal_clamped(1), goal_clamped(2));
        end_pt_ = goal_clamped;
      }
    }

    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(end_pt_);


    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      /*** display ***/
      std::vector<Eigen::Vector3d> gloabl_traj;
      if (sampleGlobalTrajForVis(gloabl_traj))
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      /* Honour the new goal from every state (see waypointCallback). In formation
         flight a fresh goal is a fresh global trajectory, so the first trajectory
         has to be planned from scratch (SEQUENTIAL_START). */
      if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      else if (exec_state_ != EMERGENCY_STOP)
        changeFSMExecState(SEQUENTIAL_START, "TRIG");
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps;
    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      wps.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        wps[i](0) = waypoints_[i][0];
        wps[i](1) = waypoints_[i][1];
        wps[i](2) = waypoints_[i][2];
      }
      end_pt_ = wps.back();
      for (size_t i = 0; i < (size_t)waypoint_num_; i++)
      {
        visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
        ros::Duration(0.001).sleep();
      }
    }
    else
      return;

    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero(), wps,
                                                             Eigen::Vector3d::Zero(),
                                                             Eigen::Vector3d::Zero());

    if (success)
    {
      /*** display ***/
      std::vector<Eigen::Vector3d> gloabl_traj;
      if (sampleGlobalTrajForVis(gloabl_traj))
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, false, true))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(bool flag_use_poly_init, bool use_formation)
  {
    double t_debug_start = ros::Time::now().toSec();
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    /* getPos()/getVel()/getAcc() EXTRAPOLATE outside [0, duration]. When the
       local trajectory has already finished (the drone got blocked in front of an
       obstacle and the FSM stayed in EXEC_TRAJ, so the trajectory kept running
       until its end while the drone did not reach the goal), t_cur was tens of
       seconds larger than the duration and the sampled start became ~1e5 m
       garbage. reboundReplan then rejected every attempt, the target was dropped
       after max_first_traj_attempts_ tries and the drone appeared to ignore the
       new RViz goal after a collision. Clamp like the EXEC_TRAJ branch does. */
    t_cur = min(info->duration, t_cur);
    if (t_cur < 0.0)
      t_cur = 0.0;

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    bool success = callReboundReplan(flag_use_poly_init, false, use_formation);

    if (!success)
      return false;

    // cout << "planFromLocalTraj : " << ros::Time::now().toSec() - t_debug_start << endl;

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, end_pt_,
        local_target_pt_, local_target_vel_);

    Eigen::Vector3d desired_start_pt, desired_start_vel, desired_start_acc;

    /* Sampling a B-spline OUTSIDE [0, duration] extrapolates it. A local
       trajectory that has already finished (start_time far in the past, e.g. the
       drone was blocked in front of an obstacle while the FSM stayed in EXEC_TRAJ)
       therefore produced ~1e5 m "start" positions: every reboundReplan call was
       rejected by its input guards and the new RViz goal was dropped after
       max_first_traj_attempts_ tries, which looked like "the drone does not accept
       a new goal after touching an obstacle". Only chain onto the previous
       trajectory while that trajectory is still running. */
    const double now_s = ros::Time::now().toSec();
    const double lc_t_off = now_s + replan_trajectory_time_ - planner_manager_->traj_.local_traj.start_time;
    const bool lc_traj_active =
        have_local_traj_ &&
        std::isfinite(lc_t_off) && std::isfinite(planner_manager_->traj_.local_traj.duration) &&
        lc_t_off >= 0.0 && lc_t_off <= planner_manager_->traj_.local_traj.duration;

    double desired_start_time = lc_traj_active ? (now_s + replan_trajectory_time_) : now_s;
    if (lc_traj_active && use_formation)
    {
      desired_start_pt = planner_manager_->traj_.local_traj.traj.getPos(lc_t_off);
      desired_start_vel = planner_manager_->traj_.local_traj.traj.getVel(lc_t_off);
      desired_start_acc = planner_manager_->traj_.local_traj.traj.getAcc(lc_t_off);
    }
    else
    {
      desired_start_pt = start_pt_;
      desired_start_vel = start_vel_;
      desired_start_acc = start_acc_;
    }
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,
        desired_start_time, local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, use_formation, have_local_traj_);

    have_new_target_ = false;

    if (plan_success)
    {
      traj_utils::PolyTraj msg;
      polyTraj2ROSMsg(msg);
      poly_traj_pub_.publish(msg);
      broadcast_ploytraj_pub_.publish(msg);
      have_local_traj_ = true;
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj msg;
    polyTraj2ROSMsg(msg);
    poly_traj_pub_.publish(msg);

    return true;
  }

} // namespace ego_planner
