// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <cmath>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;
  }

  bool EGOPlannerManager::computeInitReferenceState(const Eigen::Vector3d &start_pt,
                                                    const Eigen::Vector3d &start_vel,
                                                    const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &local_target_pt,
                                                    const Eigen::Vector3d &local_target_vel,
                                                    const double &ts,
                                                    poly_traj::MinJerkOpt &initMJO,
                                                    const bool flag_polyInit)
  {
    static bool flag_first_call = true;

    /*** case 1: use A* initialization ***/
    if (flag_first_call || flag_polyInit)
    {
      flag_first_call = false;
      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      poly_traj::Trajectory traj;
      vector<Eigen::Vector3d> simple_path;
      constexpr double init_of_init_totaldur = 2.0;

      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* step 1: A* search and generate init traj */
      Eigen::MatrixXd ctl_points;

      // traj = ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points);
      ploy_traj_opt_->astarWithMinTraj(headState, tailState, simple_path, ctl_points, initMJO);

      /* A failed A* initialization must be detected BEFORE initMJO is used:
         astarWithMinTraj() returns early (without calling frontendMJ.reset()) on
         an empty path, so initMJO keeps whatever state it had, and
         MinJerkOpt::getTraj() does `traj.reserve(N)` with that N. That is how the
         planner node actually died live:
           [ERROR] astarWithMinTraj: A* returned an empty path ... No initial reference trajectory generated.
           terminate called after throwing an instance of 'std::length_error'
             what():  vector::reserve
           Segmentation fault (core dumped)
         Once the node was gone it never answered any RViz goal again and the drone
         hovered in place, which is the reported "stuck after a collision" symptom. */
      if (simple_path.size() < 2 || ctl_points.cols() <= 0)
      {
        ROS_ERROR("[drone %d] computeInitReferenceState: A* initialization failed "
                  "(simple_path=%zu, ctl_points=%ld, start=[%f, %f, %f], target=[%f, %f, %f]).",
                  pp_.drone_id, simple_path.size(), (long)ctl_points.cols(),
                  start_pt(0), start_pt(1), start_pt(2),
                  local_target_pt(0), local_target_pt(1), local_target_pt(2));
        return false;
      }

      traj = initMJO.getTraj();

      /* A failed / empty A* initialization must not be propagated: everything
         below (and in reboundReplan) is sized from this trajectory's piece
         number. */
      if (traj.getPieceNum() <= 0)
      {
        ROS_ERROR("[drone %d] computeInitReferenceState: A* initialization produced an empty "
                  "trajectory (piece_num=%d, ctl_points=%ld, start=[%f, %f, %f], target=[%f, %f, %f]).",
                  pp_.drone_id, traj.getPieceNum(), (long)ctl_points.cols(),
                  start_pt(0), start_pt(1), start_pt(2),
                  local_target_pt(0), local_target_pt(1), local_target_pt(2));
        return false;
      }

      // show the init simple_path
      vector<vector<Eigen::Vector3d>> path_view;
      path_view.push_back(simple_path);
      visualization_->displayAStarList(path_view, 0);

      // show the init traj for debug
      std::vector<Eigen::Vector3d> point_set;
      for (int i = 0; i < ctl_points.cols(); ++i)
        point_set.push_back(ctl_points.col(i));
      visualization_->displayInitPathListDebug(point_set, 0.2, 0);
    }

    /*** case 2: initialize from previous optimal trajectory ***/
    else
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      /* The number of pieces — and with it the size of innerPs, of the piece
         duration vector and of the Trajectory built below — is derived from this
         distance. A corrupted local target (uninitialized Eigen::Vector3d coming
         out of getLocalTarget) used to turn it into one single ~38 GB allocation
         and the node was OOM-killed (total-vm 39.08 GB, anon-rss ~23 GB).
         Validate the target and clamp the piece count. */
      if (!local_target_pt.allFinite() || !local_target_vel.allFinite())
      {
        ROS_ERROR("[drone %d] computeInitReferenceState: non-finite local target "
                  "pos=[%f, %f, %f], vel=[%f, %f, %f].",
                  pp_.drone_id, local_target_pt(0), local_target_pt(1), local_target_pt(2),
                  local_target_vel(0), local_target_vel(1), local_target_vel(2));
        return false;
      }
      const double lc_target_dist = (start_pt - local_target_pt).norm();
      if (!std::isfinite(lc_target_dist) || lc_target_dist > max_local_target_dist_)
      {
        ROS_ERROR("[drone %d] computeInitReferenceState: local target [%f, %f, %f] is %f m away "
                  "from start [%f, %f, %f] (limit %f m). Rejected.",
                  pp_.drone_id, local_target_pt(0), local_target_pt(1), local_target_pt(2),
                  lc_target_dist, start_pt(0), start_pt(1), start_pt(2), max_local_target_dist_);
        return false;
      }
      int piece_nums = ceil(lc_target_dist / pp_.polyTraj_piece_length);
      if (piece_nums > max_init_piece_nums_)
      {
        ROS_WARN("[drone %d] computeInitReferenceState: initial reference would need %d pieces "
                 "(local target %f m away), clamped to %d.",
                 pp_.drone_id, piece_nums, lc_target_dist, max_init_piece_nums_);
        piece_nums = max_init_piece_nums_;
      }
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88");
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel)
  {
    double t;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    /* ---------- defensive init ----------------------------------------------
       These two outputs MUST always be written. They are passed by the FSM as
       local_target_pt_ / local_target_vel_, which are plain (uninitialized)
       Eigen::Vector3d members. If the branches below are skipped (e.g. because
       global_start_time / duration are not finite, so both comparisons are
       false), the caller would hand uninitialized garbage to
       computeInitReferenceState(), whose
         piece_nums = ceil(||start - local_target|| / polyTraj_piece_length)
       then sizes Eigen::MatrixXd(3, piece_nums-1), a piece_num-sized duration
       vector and a Trajectory with piece_nums Pieces. With garbage this is one
       single ~38 GB allocation, which got the planner node OOM-killed
       (observed total-vm 39.08 GB, anon-rss ~23 GB, on every crash). */
    local_target_pos = global_end_pt;
    local_target_vel.setZero();

    const double glb_start = traj_.global_traj.global_start_time;
    const double glb_dur = traj_.global_traj.duration;
    if (!std::isfinite(glb_start) || !std::isfinite(glb_dur) || !global_end_pt.allFinite())
    {
      ROS_ERROR("[drone %d] getLocalTarget: non-finite global trajectory data "
                "(global_start_time=%f, duration=%f, end=[%f, %f, %f]). "
                "Falling back to the global end point.",
                pp_.drone_id, glb_start, glb_dur,
                global_end_pt(0), global_end_pt(1), global_end_pt(2));
      return;
    }

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (glb_start + glb_dur);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - glb_start);
      if (!pos_t.allFinite())
      {
        ROS_ERROR("[drone %d] getLocalTarget: non-finite sample at global t=%f "
                  "(duration=%f). Keeping the global end point.",
                  pp_.drone_id, t, glb_dur);
        return;
      }
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - glb_start) >= glb_dur) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = glb_start + glb_dur;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      Eigen::Vector3d v = traj_.global_traj.traj.getVel(t - glb_start);
      local_target_vel = v.allFinite() ? v : Eigen::Vector3d::Zero();
    }

    /* ---------- final sanity check ------------------------------------------
       Legit local targets are at most ~planning_horizen (7.5 m) away from the
       start. Anything far beyond that is corrupted data and must never reach
       the min-jerk initialization (see the comment at the top). */
    const double lc_dist = (start_pt - local_target_pos).norm();
    if (!std::isfinite(lc_dist) || lc_dist > max_local_target_dist_)
    {
      ROS_ERROR("[drone %d] getLocalTarget: implausible local target [%f, %f, %f] "
                "(distance to start = %f m > %f m). Falling back to the global end point.",
                pp_.drone_id, local_target_pos(0), local_target_pos(1), local_target_pos(2),
                lc_dist, max_local_target_dist_);
      local_target_pos = global_end_pt;
      local_target_vel.setZero();
    }
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const double trajectory_start_time, const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj,
      const bool use_formation, const bool have_local_traj)
  {
    static int count = 0;

    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
    }

    /* ---------- input validation --------------------------------------------
       The horizon target below sets the number of min-jerk pieces of the initial
       reference state, i.e. the size of a Trajectory / Eigen::MatrixXd. A
       corrupted (uninitialized) target once produced a single ~38 GB allocation
       and the planner node got OOM-killed (total-vm 39.08 GB). Reject garbage
       here, before any of that is sized. */
    if (!start_pt.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
        !local_target_pt.allFinite() || !local_target_vel.allFinite())
    {
      ROS_ERROR("[drone %d] reboundReplan: non-finite input. start=[%f, %f, %f] "
                "vel=[%f, %f, %f] acc=[%f, %f, %f] target=[%f, %f, %f] target_vel=[%f, %f, %f]. Rejected.",
                pp_.drone_id, start_pt(0), start_pt(1), start_pt(2),
                start_vel(0), start_vel(1), start_vel(2),
                start_acc(0), start_acc(1), start_acc(2),
                local_target_pt(0), local_target_pt(1), local_target_pt(2),
                local_target_vel(0), local_target_vel(1), local_target_vel(2));
      return false;
    }
    const double target_dist = (start_pt - local_target_pt).norm();
    if (!std::isfinite(target_dist) || target_dist > max_local_target_dist_)
    {
      ROS_ERROR("[drone %d] reboundReplan: local target [%f, %f, %f] is %f m away from "
                "start [%f, %f, %f] (limit %f m). Rejected.",
                pp_.drone_id, local_target_pt(0), local_target_pt(1), local_target_pt(2),
                target_dist, start_pt(0), start_pt(1), start_pt(2), max_local_target_dist_);
      return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

   
    poly_traj::MinJerkOpt initMJO;
    if (!computeInitReferenceState(start_pt, start_vel, start_acc,
                                   local_target_pt, local_target_vel,
                                   ts, initMJO, flag_polyInit))
    {
      return false;
    }


    Eigen::MatrixXd cstr_pts = initMJO.getInitConstrainPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    ploy_traj_opt_->setControlPoints(cstr_pts);

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    poly_traj::Trajectory initTraj = initMJO.getTraj();
    int PN = initTraj.getPieceNum();
    Eigen::MatrixXd all_pos = initTraj.getPositions();
    Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
    tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(headState, tailState,
                                                            innerPts, initTraj.getDurations(),
                                                            cstr_pts, use_formation);
 
    t_opt = ros::Time::now() - t_start;

    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success
         << ",count_success= " << count_success << endl;
    average_plan_time_ = sum_time / count_success;

    if (have_local_traj && use_formation)
    {
      double delta_replan_time = trajectory_start_time - ros::Time::now().toSec();
      if (delta_replan_time > 0)
        ros::Duration(delta_replan_time).sleep();
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), trajectory_start_time);
    }
    else
    {
      traj_.setLocalTraj(ploy_traj_opt_->getMinJerkOptPtr()->getTraj(), ros::Time::now().toSec()); // todo time
    }

    visualization_->displayOptimalList(cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    traj_.setLocalTraj(stopMJO.getTraj(), ros::Time::now().toSec());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < ploy_traj_opt_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    /* ---------------------------------------------------------------
       Input validation. This function is the single producer of
       traj_.global_traj.duration = dist / max_vel_. If the inputs are
       corrupted (e.g. odometry jump / uninitialized state), the resulting
       duration becomes absurd, and the callers sample the trajectory with
       vector<Eigen::Vector3d>(duration/0.1), which allocates tens of GB and
       gets the node OOM-killed. Reject such inputs here at the source.
       --------------------------------------------------------------- */
    if (waypoints.empty())
    {
      ROS_ERROR("planGlobalTrajWaypoints: empty waypoint list!");
      return false;
    }
    if (!start_pos.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
        !end_vel.allFinite() || !end_acc.allFinite())
    {
      ROS_ERROR("planGlobalTrajWaypoints: non-finite start state (pos=%f,%f,%f vel=%f,%f,%f acc=%f,%f,%f) "
                "or end state.",
                start_pos(0), start_pos(1), start_pos(2),
                start_vel(0), start_vel(1), start_vel(2),
                start_acc(0), start_acc(1), start_acc(2));
      return false;
    }
    double total_dist = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      if (!waypoints[i].allFinite())
      {
        ROS_ERROR("planGlobalTrajWaypoints: non-finite waypoint %lu = [%f, %f, %f].",
                  (unsigned long)i, waypoints[i](0), waypoints[i](1), waypoints[i](2));
        return false;
      }
      total_dist += (i == 0) ? (waypoints[0] - start_pos).norm()
                             : (waypoints[i] - waypoints[i - 1]).norm();
    }
    if (!(pp_.max_vel_ > 1e-3))
    {
      ROS_ERROR("planGlobalTrajWaypoints: invalid max_vel_ = %f.", pp_.max_vel_);
      return false;
    }
    if (!(total_dist < max_global_traj_dist_))
    {
      ROS_ERROR("planGlobalTrajWaypoints: requested global trajectory is too long (%f m, start=[%f, %f, %f], "
                "goal=[%f, %f, %f]). Rejected.",
                total_dist, start_pos(0), start_pos(1), start_pos(2),
                waypoints.back()(0), waypoints.back()(1), waypoints.back()(2));
      return false;
    }

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < waypoints.size() - 1; i++)
        innerPts.col(i) = waypoints[i];
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }
    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_;
    Eigen::VectorXd time_vec(waypoints.size());
    int try_num = 0;
    do
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }
      globalMJO.generate(innerPts, time_vec);
      // cout << "try_num : " << try_num << endl;
      // cout << "max vel : " << globalMJO.getTraj().getMaxVelRate() << endl;
      // cout << "time_vec : " << time_vec.transpose() << endl;

      des_vel /= 1.2;
      try_num++;
    } while (globalMJO.getTraj().getMaxVelRate() > pp_.max_vel_ && try_num <= 5);

    /* Final sanity check on the produced trajectory: its duration is later used
       to size std::vector<Eigen::Vector3d>(duration/0.1) for visualization, so an
       absurd value would be turned into a multi-GB allocation. */
    const double global_duration = globalMJO.getTraj().getTotalDuration();
    if (!std::isfinite(global_duration) || global_duration < 0.0 || global_duration > max_global_traj_duration_)
    {
      ROS_ERROR("planGlobalTrajWaypoints: abnormal global trajectory duration = %f s "
                "(total dist = %f m, max_vel = %f). Rejected.",
                global_duration, total_dist, pp_.max_vel_);
      return false;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
