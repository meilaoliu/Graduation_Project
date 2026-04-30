
#include <plan_manage/ego_replan_fsm.h>

#define PI 3.1415926
#define yaw_error_max 20.0/180*PI

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/w_adjust_", w_adjust, 1.0);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
    
    // forward_only 参数：true 表示车辆只能前进，不允许倒车
    // 同时设置全局参数，让 traj_server 也能读取
    nh.param("fsm/forward_only", forward_only_, true);
    ros::param::set("/forward_only", forward_only_);  // 设置全局参数供 traj_server 使用
    ROS_INFO("[FSM] forward_only = %s", forward_only_ ? "true" : "false");

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    dir = POSITIVE;
      goal_last << 0,0,0;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("/odom_map", 1, &EGOReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
    minco_pub_ = nh.advertise<ego_planner::MINCOTraj>("/planning/minco_traj", 10);
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);
    // cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/twd_velocity_controller/cmd_vel",100);
    cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100);
    adjust_cmd_pub_ = nh.advertise<std_msgs::UInt8>("/is_adjust_yaw",100);
    odom_adjust_pub_ = nh.advertise<nav_msgs::Odometry>("/odom_adjust",100);
    dir_pub = nh.advertise<std_msgs::UInt8>("/direction",100);
    stop_pub = nh.advertise<std_msgs::UInt8>("/emergency_stop",100);
    // 分段完成事件：当一段全局轨迹自然执行完毕时发布段编号，供上层调度器（nlp_commander）触发拍照/dwell/下一段
    segment_done_pub_ = nh.advertise<std_msgs::UInt32>("/segment_done", 10);

    is_target_receive = false;

    // 始终订阅 /global_waypoints，便于在任何 flight_type 下都能由外部脚本注入分段巡检任务（纯增量、对原模式无影响）
    segment_waypoints_sub_ = nh.subscribe("/global_waypoints", 1, &EGOReplanFSM::segmentWaypointsCallback, this);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/way_point", 1, &EGOReplanFSM::goal_callback, this);
      //waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep();
      while (ros::ok() && !have_odom_)
        ros::spinOnce();
      planGlobalTrajbyGivenWps();
    }
    else if (target_type_ == TARGET_TYPE::DYNAMIC_WAYPOINTS)
    {
      // 等待外部 /global_waypoints 推送，启动时无需做任何事；状态会停在 INIT/WAIT_TARGET
      ROS_INFO("[FSM] DYNAMIC_WAYPOINTS mode: waiting for /global_waypoints (nav_msgs/Path).");
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;


    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
      {
        // 修复 B: 同 goal_callback, 大航向偏差时改走 GEN_NEW_TRAJ 触发 ADJUST_POSE
        double target_dir = std::atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
        double angle_diff = target_dir - yaw;
        while (angle_diff > PI) angle_diff -= 2 * PI;
        while (angle_diff < -PI) angle_diff += 2 * PI;
        const double yaw_thresh = forward_only_ ? (PI / 2.0) : (2.0 * PI / 3.0);
        if (std::abs(angle_diff) > yaw_thresh)
        {
          ROS_WARN("[waypointCallback] Large heading offset (%.1f deg), stop and replan from rest",
                   std::abs(angle_diff) * 180.0 / PI);
          callEmergencyStop(odom_pos_);
          changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "TRIG");
        }
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::segmentWaypointsCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (!have_odom_)
    {
      ROS_WARN("[FSM][segment] odom not ready, ignore /global_waypoints message.");
      return;
    }

    if (msg->poses.empty())
    {
      ROS_WARN("[FSM][segment] /global_waypoints with empty path, ignored.");
      return;
    }

    // 段编号优先取 header.seq；上游手动设置时可保证段号严格递增
    current_segment_id_ = static_cast<uint32_t>(msg->header.seq);

    // 把 Path 中所有 pose 视为本段的 waypoint 序列。
    // 约定：上层 (nlp_commander 的 SegmentScheduler) 已经按"停留点"切好段，
    //       因此本段首尾即停留点；这里强制首尾速度/加速度为 0，符合论文中"段间静止拍照"的语义。
    std::vector<Eigen::Vector3d> wps;
    wps.reserve(msg->poses.size());
    for (const auto &p : msg->poses)
    {
      wps.emplace_back(p.pose.position.x, p.pose.position.y, p.pose.position.z);
    }

    // 起点位姿用当前 odom，符合"机器人就停在上段终点"的物理事实
    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 段末作为本段的 end_pt_，用于后续 EXEC_TRAJ 终止判定
    end_pt_ = wps.back();

    // 可视化所有段内 waypoint
    for (size_t i = 0; i < wps.size(); ++i)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(i));
      ros::Duration(0.001).sleep();
    }

    if (!success)
    {
      ROS_ERROR("[FSM][segment %u] planGlobalTrajWaypoints failed (n=%zu).", current_segment_id_, wps.size());
      has_active_segment_ = false;
      return;
    }

    // 渲染整段全局参考轨迹（B样条/MINCO 共用同一份 global_data_）
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> global_traj;
    global_traj.reserve(std::max(0, i_end));
    for (int i = 0; i < i_end; ++i)
    {
      global_traj.push_back(planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t));
    }
    visualization_->displayGlobalPathList(global_traj, 0.1, 0);

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    trigger_ = true;
    has_active_segment_ = true;

    ROS_INFO("[FSM][segment %u] accepted, %zu waypoints, duration=%.2fs.",
             current_segment_id_, wps.size(), planner_manager_->global_data_.global_duration_);

    // 触发新一段的局部轨迹生成；若当前正在执行轨迹，按 waypointCallback 中已有的逻辑处理大航向偏差
    if (exec_state_ == WAIT_TARGET || exec_state_ == INIT)
      changeFSMExecState(GEN_NEW_TRAJ, "SEG");
    else if (exec_state_ == EXEC_TRAJ)
    {
      double target_dir = std::atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
      double angle_diff = target_dir - yaw;
      while (angle_diff > PI) angle_diff -= 2 * PI;
      while (angle_diff < -PI) angle_diff += 2 * PI;
      const double yaw_thresh = forward_only_ ? (PI / 2.0) : (2.0 * PI / 3.0);
      if (std::abs(angle_diff) > yaw_thresh)
      {
        callEmergencyStop(odom_pos_);
        changeFSMExecState(GEN_NEW_TRAJ, "SEG");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "SEG");
      }
    }
    else
    {
      changeFSMExecState(GEN_NEW_TRAJ, "SEG");
    }
  }

void EGOReplanFSM::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Eigen::Vector3d req_end_pt;
    req_end_pt << msg->pose.position.x, msg->pose.position.y, odom_pos_(2);

    // === 修复: 目标点位于障碍物内的处理 ===
    // 若用户/上层指定了一个落在膨胀障碍内的终点, 直接交给规划器只会陷入
    // "全局规划失败 → REPLAN 死循环 → 无止境失败" 的状态。
    // 这里先做一次螺旋搜索, 把目标投射到最近的可达自由点; 若邻域内无自由点
    // (如终点深陷大型障碍中央), 则直接拒绝该目标。
    auto goal_map = planner_manager_->grid_map_;
    if (goal_map && goal_map->getInflateOccupancy(req_end_pt) == 1)
    {
        bool found_free = false;
        Eigen::Vector3d free_pt = req_end_pt;
        constexpr double max_search_radius = 2.0;
        constexpr double step = 0.15;
        for (double r = step; r <= max_search_radius && !found_free; r += step)
        {
            for (double a = 0; a < 2.0 * M_PI - 1e-3; a += M_PI / 8.0)
            {
                Eigen::Vector3d cand(req_end_pt(0) + r * std::cos(a),
                                     req_end_pt(1) + r * std::sin(a),
                                     req_end_pt(2));
                if (goal_map->getInflateOccupancy(cand) == 0)
                {
                    free_pt = cand;
                    found_free = true;
                    break;
                }
            }
        }
        if (!found_free)
        {
            ROS_ERROR("[goal_callback] Goal (%.2f, %.2f) lies inside obstacle and no free point within %.1fm. Goal rejected.",
                      req_end_pt(0), req_end_pt(1), max_search_radius);
            return;
        }
        ROS_WARN("[goal_callback] Goal (%.2f, %.2f) inside obstacle, projected to nearest free (%.2f, %.2f).",
                 req_end_pt(0), req_end_pt(1), free_pt(0), free_pt(1));
        req_end_pt = free_pt;
    }

    end_pt_ = req_end_pt;
    trigger_ = true;
    init_pt_ = odom_pos_;
    goal_last = end_pt_;

    bool success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), 
                                                     end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
        /*** display ***/
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }
        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        /*** FSM 状态转换 ***/
        if (exec_state_ == WAIT_TARGET)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            is_target_receive = true;
        }
        else if (exec_state_ == EXEC_TRAJ)
        {
            // forward_only 模式下：检查新目标是否在车辆后方
            if (forward_only_)
            {
                double current_heading = yaw;
            double target_dir = atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
            double angle_diff = target_dir - current_heading;
            // 归一化到 [-PI, PI]
            while (angle_diff > PI) angle_diff -= 2 * PI;
            while (angle_diff < -PI) angle_diff += 2 * PI;
            
            // 如果目标在后方（角度差 > 90度），强制停止并从静止重新规划
            if (abs(angle_diff) > PI / 2.0)
            {
                    ROS_WARN("[goal_callback] Target behind (%.1f deg), stop and replan from rest", 
                         abs(angle_diff) * 180.0 / PI);
                callEmergencyStop(odom_pos_);
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            }
            else
            {
                    changeFSMExecState(REPLAN_TRAJ, "TRIG");
                }
            }
            else
            {
                // 允许倒车模式: 默认走 REPLAN_TRAJ; 但若新目标方向相对当前航向偏差过大,
                // planFromCurrentTraj 出来的拼接弧会过于扭曲, 出现"画大圆"而不是停下转向。
                // === 修复 B: 与 forward_only 一致, 大偏差时强制 EmergencyStop + GEN_NEW_TRAJ,
                // 让 GEN_NEW_TRAJ 内部的 yaw_error 检查触发 ADJUST_POSE 原地转向。
                double current_heading = yaw;
                double target_dir = atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
                double angle_diff = target_dir - current_heading;
                while (angle_diff > PI) angle_diff -= 2 * PI;
                while (angle_diff < -PI) angle_diff += 2 * PI;
                // 倒车模式下偏差超过 ~120deg (大于 PI/2 + 缓冲) 时, 即便允许倒车,
                // 直接拼弧也很难走通; 此时停车并走 GEN_NEW_TRAJ 让 ADJUST_POSE 介入
                if (std::abs(angle_diff) > 2.0 * PI / 3.0)
                {
                    ROS_WARN("[goal_callback] Large heading offset (%.1f deg), stop and replan from rest",
                             std::abs(angle_diff) * 180.0 / PI);
                    callEmergencyStop(odom_pos_);
                    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
                }
                else
                {
                    changeFSMExecState(REPLAN_TRAJ, "TRIG");
                }
            }
            is_target_receive = true;
        }

        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
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

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // forward_only 模式下，不根据 dir 修改 yaw，保持真实车头朝向
    if(!forward_only_ && dir==NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }
      nav_msgs::Odometry odom_adjust;
      geometry_msgs::Quaternion quat_adj=tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
      odom_adjust = *msg;
      odom_adjust.pose.pose.orientation = quat_adj;
      odom_adjust_pub_.publish(odom_adjust);

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::changeDirection() {
      if(dir == POSITIVE)
      {
          dir = NEGATIVE;
      }else
      {
          dir = POSITIVE;
      }
      std_msgs::UInt8 dir_new;
      dir_new.data = dir;
      dir_pub.publish(dir_new);
  }



  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case ADJUST_POSE:
    {
        // Fix I: 不论 forward_only 与否, 进入 ADJUST_POSE 必须先等线速度趋零再开始转向
        // 否则倒车模式下会边滑边转, 表现为"画圈"
        double linear_speed = odom_vel_.head<2>().norm();  // 计算2D平面线速度
        if (linear_speed > 0.05)  // 阈值 0.05 m/s, 比 0.1 严
        {
            // 发送停止命令
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmd_pub_.publish(cmd_vel);
            
            static int wait_count = 0;
            if (wait_count++ % 50 == 0)  // 每0.5秒打印一次
            {
                ROS_INFO("[ADJUST_POSE] Waiting for vehicle to stop: speed=%.3f m/s", linear_speed);
            }
            break;  // 不执行后续的转向逻辑，等待下次回调
        }
        
        yaw_error = yaw_start-yaw;
        //first step : calculate the yaw error
        if(abs(yaw_error)>PI)
        {
            yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
        }
        static int count=0;
        if(count%100==0)
        {
            string directions[2] = {"POSITIVE","NAGETIVE"};
            cout << "direction : "<<directions[int(dir)]<<endl;
            cout<<"current yaw: "<<yaw<<endl;
            cout<<"yaw error : "<<yaw_error<<endl;
            count=0;
        }
        count+=1;
        if(abs(yaw_error)>yaw_error_max)
        {
            is_adjust_pose.data = 1;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;
            cmd_pub_.publish(cmd_vel);
            adjust_cmd_pub_.publish(is_adjust_pose);
        }else
        {
            is_adjust_pose.data = 0;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z =0;
            cmd_pub_.publish(cmd_vel);
            adjust_cmd_pub_.publish(is_adjust_pose);
            
            // 原地转向完成后，必须重新规划轨迹
            // 因为之前的轨迹是基于旧的状态规划的，直接恢复会导致状态不匹配
            // GEN_NEW_TRAJ 会调用 callReboundReplan(true) 从当前静止状态重新规划
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
    }

    case GEN_NEW_TRAJ:
    {
      // === 修复 B: 同 tick 重试冷却 ===
      // 33ms 内堆 11 次同 start/同 target 的 max_restarts 是死循环主源:
      // FSM 失败 → 立即 changeFSMExecState(GEN_NEW_TRAJ, ...) → 输入完全没变 → 同样失败。
      // 此处强制最少间隔 100ms, 期间留 odom/map 一次更新机会, 并避免日志被刷爆。
      if (consecutive_gen_failure_count_ > 0)
      {
        ros::Time now_t = ros::Time::now();
        if ((now_t - last_gen_failure_stamp_).toSec() < 0.10)
        {
          // 不切状态, 让 exec_timer_ 下个 tick (~10ms) 再来; 真正放行需 >=100ms 后
          break;
        }
      }

      start_pt_ = odom_pos_;
      // P0-B: 不再硬清零 start_vel_。MINCO finelyCheck 在 |v|≈0 时
      // 会让承载点的曲率/碰撞段判断失真，进而把 NaN 端点喂给 A*，
      // 触发大量 reason=max_restarts 与 EMERGENCY_STOP。
      // 仅在车辆基本静止时才清零；否则保留沿当前航向的速度分量。
      double v_norm = odom_vel_.head<2>().norm();
      if (v_norm < 0.05)
      {
        start_vel_.setZero();
      }
      else
      {
        Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
        double yaw = atan2(rot_x(1), rot_x(0));
        start_vel_ << v_norm * std::cos(yaw), v_norm * std::sin(yaw), 0.0;
      }
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      // P2-B: 对齐原版 planFromGlobalTraj(10) — 单次失败不立即跳走,
      // 在同一帧内最多重试 10 次, 第 1 次用确定多项式初始化, 后续启用随机扰动
      // 以跨越 max_restarts/反弹卡死的局部坑。避免 FSM 重入延迟把短暂失败
      // 误升级为 EMERGENCY_STOP。
      bool success = false;
      for (int i = 0; i < 10; ++i)
      {
        bool flag_random = flag_random_poly_init || (i > 0);
        if (callReboundReplan(true, flag_random))
        {
          success = true;
          if (i > 0)
            ROS_INFO("[FSM] GEN_NEW_TRAJ succeeded after %d retries", i);
          break;
        }
      }
      if (success)
      {
          // Fix G: 成功一次 → 重置 stuck 计数
          consecutive_init_collision_count_ = 0;
          consecutive_gen_failure_count_ = 0;
          // 根据轨迹类型获取速度
          Eigen::Vector3d vel_start;
          auto info = &planner_manager_->local_data_;
          if (info->use_minco_traj_)
          {
            vel_start = info->minco_traj_.getVel(0.1);
          }
          else
          {
            vel_start = info->velocity_traj_.evaluateDeBoor(0.1);
          }
          yaw_start = atan2(vel_start(1),vel_start(0));
          cout<<"yaw start : "<<yaw_start<<endl;
          yaw_error = yaw_start-yaw;

          //first step : calculate the yaw error
          if(abs(yaw_error)>PI)
          {
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
          }
          cout<<"yaw error : "<<yaw_error<<endl;
          
          if (forward_only_)
          {
              // forward_only 模式：需要原地掉头，不修改 yaw 和 dir
              // 如果轨迹方向与车头方向差异过大，进入 ADJUST_POSE 状态原地转向
              if(abs(yaw_error) > yaw_error_max)
              {
                  ROS_WARN("[FSM forward_only] Need to turn: yaw=%.1f, traj=%.1f, error=%.1f deg",
                           yaw * 180.0 / PI, yaw_start * 180.0 / PI, yaw_error * 180.0 / PI);
                  cmd_vel.linear.x = 0;
                  cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;
                  changeFSMExecState(ADJUST_POSE, "TRIG");
                  last_state_ = GEN_NEW_TRAJ;
                  is_target_receive=false;
                  return;
              }
          }
          else
          {
              // 原有逻辑：允许倒车，通过修改 yaw 和 dir 来"虚拟转向"
          if(abs(yaw_error)>PI/2.0)
          {
              if(yaw>0)
              {
                  yaw -= PI;
              }else if(yaw<0)
              {
                  yaw += PI;
              }
              changeDirection();
              yaw_error = yaw_start - yaw;
          }
          if(abs(yaw_error)>yaw_error_max)
          {
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;
              changeFSMExecState(ADJUST_POSE, "TRIG");
              last_state_ = GEN_NEW_TRAJ;
              is_target_receive=false;
              return;
              }
          }
          cout<<"yaw error : "<<yaw_error<<endl;
        info->start_time_ = ros::Time::now();
        publishBspline();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        // Fix G: 检测连续 init_collision_dense 卡死
        const std::string &g_reason = planner_manager_->getMincoLastFailureReason();
        if (g_reason == "init_collision_dense")
          consecutive_init_collision_count_++;
        else
          consecutive_init_collision_count_ = 0;

        if (consecutive_init_collision_count_ >= 20)
        {
          ROS_ERROR("[FSM Fix G] %d consecutive init_collision_dense in GEN_NEW_TRAJ, goal unreachable",
                    consecutive_init_collision_count_);
          callEmergencyStop(odom_pos_);
          have_target_ = false;
          consecutive_init_collision_count_ = 0;
          changeFSMExecState(WAIT_TARGET, "FSM_FixG");
        }
        else if (consecutive_init_collision_count_ >= 5)
        {
          ROS_WARN("[FSM Fix G] %d consecutive init_collision_dense, emergency stop + replan global",
                   consecutive_init_collision_count_);
          callEmergencyStop(odom_pos_);
          // 用当前位置重做全局 poly: 让 last_progress_time_=0 重新挑局部目标
          planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                            end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
          consecutive_init_collision_count_ = 0;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM_FixG");
        }
        else
        {
          // 通用退避: 不论 reason, 连续失败堆积时强制 emergency + 全局重规
          // 切断 "30+ Replan / 33ms 同 start 同 target 重复失败" 死循环
          consecutive_gen_failure_count_++;
          // 修复 B: 记录失败时间戳, 进入 GEN_NEW_TRAJ 入口的 100ms 冷却窗口
          last_gen_failure_stamp_ = ros::Time::now();
          if (consecutive_gen_failure_count_ >= 3)
          {
            // === 修复: 若失败原因是终点已落入障碍 (例如地图更新后),
            // 反复 planGlobalTraj 也无济于事 → 直接放弃当前 target
            auto map_now = planner_manager_->grid_map_;
            if (map_now && map_now->getInflateOccupancy(end_pt_) == 1)
            {
              ROS_ERROR("[FSM] %d consecutive failures and end_pt (%.2f, %.2f) is now in obstacle. Drop target, back to WAIT_TARGET.",
                        consecutive_gen_failure_count_, end_pt_(0), end_pt_(1));
              callEmergencyStop(odom_pos_);
              have_target_ = false;
              have_new_target_ = false;
              trigger_ = false;
              consecutive_gen_failure_count_ = 0;
              changeFSMExecState(WAIT_TARGET, "FSM");
              break;
            }
            ROS_WARN("[FSM] %d consecutive GEN_NEW_TRAJ failures, emergency stop + global replan",
                     consecutive_gen_failure_count_);
            callEmergencyStop(odom_pos_);
            planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                              end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            consecutive_gen_failure_count_ = 0;
          }
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
          // Fix G: 成功一次 → 重置 stuck 计数
          consecutive_init_collision_count_ = 0;
          // 根据轨迹类型获取速度
          Eigen::Vector3d vel_start;
          auto info = &planner_manager_->local_data_;
          if (info->use_minco_traj_)
          {
            vel_start = info->minco_traj_.getVel(0.1);
          }
          else
          {
            vel_start = info->velocity_traj_.evaluateDeBoor(0.1);
          }
          yaw_start = atan2(vel_start(1),vel_start(0));
          yaw_error = yaw_start-yaw;
          
          info->start_time_ = ros::Time::now();
          std_msgs::UInt8 stop_cmd;
          stop_cmd.data = 0;
          stop_pub.publish(stop_cmd);
          publishBspline();
          
          // 可视化重规划的轨迹
          if (planner_manager_->pp_.use_minco_ && info->use_minco_traj_)
          {
            visualization_->displayMincoTraj(info->minco_traj_, 0.05, 0);
          }
          
          changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
          // Fix G: REPLAN_TRAJ 失败也走 stuck 检测
          const std::string &g_reason2 = planner_manager_->getMincoLastFailureReason();
          if (g_reason2 == "init_collision_dense")
            consecutive_init_collision_count_++;
          else
            consecutive_init_collision_count_ = 0;

          if (consecutive_init_collision_count_ >= 20)
          {
            ROS_ERROR("[FSM Fix G] %d consecutive init_collision_dense in REPLAN_TRAJ, goal unreachable",
                      consecutive_init_collision_count_);
            callEmergencyStop(odom_pos_);
            have_target_ = false;
            consecutive_init_collision_count_ = 0;
            changeFSMExecState(WAIT_TARGET, "FSM_FixG");
          }
          else if (consecutive_init_collision_count_ >= 5)
          {
            ROS_WARN("[FSM Fix G] %d consecutive init_collision_dense (REPLAN), emergency stop + replan global",
                     consecutive_init_collision_count_);
            callEmergencyStop(odom_pos_);
            planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                              end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            consecutive_init_collision_count_ = 0;
            changeFSMExecState(GEN_NEW_TRAJ, "FSM_FixG");
          }
          else if (exec_state_ == REPLAN_TRAJ)
          {
              changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
          // 否则保持planFromCurrentTraj()设置的状态（如ADJUST_POSE）
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos;
      if (planner_manager_->pp_.use_minco_)
      {
        pos = info->minco_traj_.getPos(t_cur);
      }
      else
      {
        pos = info->position_traj_.evaluateDeBoorT(t_cur);
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      double dist_to_goal = (end_pt_ - pos).norm();
      double dist_to_goal_actual = (end_pt_ - odom_pos_).norm();
      
      if (t_cur > info->duration_ - 1e-2)
      {
        // 到达轨迹终点 - 添加调试信息
        ROS_INFO("[FSM] Traj ended: t=%.2fs, dur=%.2fs, dist_traj=%.2fm, dist_actual=%.2fm", 
                 t_cur, info->duration_, dist_to_goal, dist_to_goal_actual);
        
        // 只有当实际距离也很近时才认为到达目标
        if (dist_to_goal_actual < 0.5)
        {
          have_target_ = false;
          // 分段巡检：到达本段终点时通知上层调度器（适用于 B样条 / MINCO 两种局部规划器，因为本判定基于 odom + global_data_，与具体样条无关）
          if (has_active_segment_)
          {
            std_msgs::UInt32 done_msg;
            done_msg.data = current_segment_id_;
            segment_done_pub_.publish(done_msg);
            ROS_INFO("[FSM][segment %u] done, notified /segment_done.", current_segment_id_);
            has_active_segment_ = false;
          }
          changeFSMExecState(WAIT_TARGET, "FSM");
          return;
        }
        else
        {
          // 轨迹结束但还没到目标
          // 使用 GEN_NEW_TRAJ 而不是 REPLAN_TRAJ，因为当前轨迹已过期
          // REPLAN_TRAJ 依赖有效的当前轨迹，而此时 t_to_lc_end < 0 会导致失败
          ROS_WARN("[FSM] Traj ended but not at goal (dist=%.2fm), generating new trajectory!", dist_to_goal_actual);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      // 持续发送停止命令，确保车辆可靠停止
      std_msgs::UInt8 stop_cmd;
      stop_cmd.data = 1;
      stop_pub.publish(stop_cmd);
      
      // 同时直接发送零速度命令作为备份
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      cmd_pub_.publish(cmd_vel);

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
        {
          // 恢复正常状态前，清除停止命令
          stop_cmd.data = 0;
          stop_pub.publish(stop_cmd);
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    // 根据轨迹类型获取当前状态
    if (planner_manager_->pp_.use_minco_)
    {
      start_pt_ = info->minco_traj_.getPos(t_cur);
      start_vel_ = info->minco_traj_.getVel(t_cur);
      start_acc_ = info->minco_traj_.getAcc(t_cur);
    }
    else
    {
      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    }

    // 计算目标方向与当前航向的误差
    yaw_start = atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
    yaw_error = yaw_start - yaw;
    if (abs(yaw_error) > PI)
    {
        yaw_error = yaw_error - yaw_error / abs(yaw_error) * 2 * PI;
      }

    // forward_only 模式：检查是否需要掉头
    if (forward_only_)
    {
        // 只在收到新目标时检查航向误差，行走过程中的重规划不触发掉头
        // 这样可以避免弯道行驶时因轨迹方向变化而频繁停车
        if (is_target_receive)
        {
            // 如果航向误差过大，需要原地掉头
            if (abs(yaw_error) > yaw_error_max)
            {
                ROS_WARN("[FSM forward_only] Need to turn: yaw=%.1f, traj=%.1f, error=%.1f deg", 
                         yaw * 180.0 / PI, yaw_start * 180.0 / PI, yaw_error * 180.0 / PI);
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = yaw_error / abs(yaw_error) * w_adjust;
                changeFSMExecState(ADJUST_POSE, "TRIG");
                is_target_receive = false;
                return false;
            }
        }
        
        // 行走过程中：不再强制清零速度，避免初始状态突变导致轨迹振荡
        // 原先这里在速度方向与目标方向夹角>120°时清零速度和加速度，
        // 会造成连续重规划时初始条件不一致，MINCO生成的轨迹形状跳跃
    }
        else
        {
        // 允许倒车模式：收到新目标时检查是否需要切换方向
        if (is_target_receive)
        {
            if (abs(yaw_error) > PI / 2.0)
            {
                // 目标在后方，切换到倒车模式
                if (yaw > 0)
            {
                yaw -= PI;
                }
                else if (yaw < 0)
            {
                yaw += PI;
            }
            changeDirection();
            yaw_error = yaw_start - yaw;
                start_vel_ << -start_vel_(0), -start_vel_(1), 0;
                start_acc_ << -start_acc_(0), -start_acc_(1), 0;
            std_msgs::UInt8 stop_cmd;
            stop_cmd.data = 1;
            stop_pub.publish(stop_cmd);
            }
        }
    }
    is_target_receive = false;


    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 在以下状态下不进行碰撞检查：
    // - WAIT_TARGET: 等待目标
    // - ADJUST_POSE: 正在原地掉头，轨迹还未开始执行
    // - INIT: 初始化
    // - traj_id_ <= 0: 轨迹未初始化
    // - start_time_ 未设置
    if (exec_state_ == WAIT_TARGET || exec_state_ == ADJUST_POSE || 
        exec_state_ == INIT || info->traj_id_ <= 0 || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- 使用预缓存点进行碰撞检测 ---------- */
    const double t_cur = (ros::Time::now() - info->start_time_).toSec();
    const PtsChk_t &pts_chk = info->pts_chk_;

    // MINCO 模式：使用预缓存点进行高效检测
    if (info->use_minco_traj_ && pts_chk.size() > 0)
    {
      // 定位当前时间所在的 piece/segment
      double t_temp = t_cur;
      int i_start = info->minco_traj_.locatePieceIdx(t_temp);
      
      if (i_start >= (int)pts_chk.size())
        return;  // 当前时间已超出预检测范围
      
      // 找到第一个 t > t_cur 的检测点
      size_t j_start = 0;
      for (; i_start < (int)pts_chk.size(); ++i_start)
      {
        for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
        {
          if (pts_chk[i_start][j_start].first > t_cur)
            goto find_ij_start;
        }
      }
    find_ij_start:;
      
      // 确定检测范围：只检测前 2/3 或全部（如果接近目标）
      const bool touch_the_end = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
      
      // 遍历预缓存的检测点
      for (size_t i = i_start; i < i_end; ++i)
      {
        for (size_t j = (i == (size_t)i_start ? j_start : 0); j < pts_chk[i].size(); ++j)
        {
          double t = pts_chk[i][j].first;
          Eigen::Vector3d p = pts_chk[i][j].second;
          
          // 2D 碰撞检测（地面机器人）
          Eigen::Vector2d p2d;
          p2d << p(0), p(1);
          
          if (map->getInflateOccupancy2d(p2d))
          {
            /* 检测到碰撞，尝试重规划 */
            if (planFromCurrentTraj())
            {
              // 重规划成功：立即发布新轨迹，重置时间
              ROS_INFO("[SAFETY] Replan success when collision detected at t=%.2f/%.2f", t, info->duration_);
              info->start_time_ = ros::Time::now();
              publishBspline();  // 立即发布新轨迹
              
              // 可视化更新
              if (planner_manager_->pp_.use_minco_ && info->use_minco_traj_)
              {
                visualization_->displayMincoTraj(info->minco_traj_, 0.05, 0);
              }
              
              changeFSMExecState(EXEC_TRAJ, "SAFETY");
              return;
            }
            else
            {
              // 重规划失败
              if (t - t_cur < emergency_time_)
              {
                ROS_WARN("[SAFETY] Emergency stop! Collision in %.2fs", t - t_cur);
                changeFSMExecState(EMERGENCY_STOP, "SAFETY");
              }
              else
              {
                ROS_WARN("[SAFETY] Collision at t=%.2f, replan later", t);
                changeFSMExecState(REPLAN_TRAJ, "SAFETY");
              }
              return;
            }
          }
        }
      }
    }
    else
    {
      // B 样条模式或 pts_chk 为空：使用固定步长在线采样（兼容旧逻辑）
      constexpr double time_step = 0.01;
      double t_2_3 = info->duration_ * 2 / 3;
      for (double t = t_cur; t < info->duration_; t += time_step)
      {
        if (t_cur < t_2_3 && t >= t_2_3)
          break;

        Eigen::Vector3d pos_cur;
        if (info->use_minco_traj_)
        {
          pos_cur = info->minco_traj_.getPos(t);
        }
        else
        {
          pos_cur = info->position_traj_.evaluateDeBoorT(t);
        }
        
        Eigen::Vector2d pos_cur2d;
        pos_cur2d << pos_cur(0), pos_cur(1);
        
        if (map->getInflateOccupancy2d(pos_cur2d))
        {
          if (planFromCurrentTraj())
          {
            // 重规划成功：立即发布新轨迹，重置时间
            info->start_time_ = ros::Time::now();
            publishBspline();
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_)
            {
              ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
        }
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    start_pt_(2) = odom_pos_(2);
    start_vel_(2) = 0;
    start_acc_(2) = 0;
    local_target_pt_(2) = odom_pos_(2);
    local_target_vel_(2) = 0;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_,
                                        (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj,
                                        touch_goal_); // P2-C: 显式传 touch_goal
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;
      
      // 根据轨迹类型进行可视化
      if (planner_manager_->pp_.use_minco_ && info->use_minco_traj_)
      {
        // MINCO 轨迹可视化：直接显示 MINCO 轨迹
        visualization_->displayMincoTraj(info->minco_traj_, 0.05, 0);
      }
      else
      {
        // B 样条轨迹可视化
        Eigen::MatrixXd control_points = info->position_traj_.get_control_points();
        for(int i=0;i<control_points.cols();i++) control_points.col(i)(2) = odom_pos_(2);
        visualization_->displayOptimalList(control_points, 0);
      }
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;
    info->start_time_ = ros::Time::now();
    
    // 根据轨迹类型选择发布方式
    if (planner_manager_->pp_.use_minco_)
    {
        // MINCO 模式：发布 MINCO 轨迹消息
    publishMincoTraj();
    }
    else
    {
        // B样条模式：发布 B样条消息
        ego_planner::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        bspline_pub_.publish(bspline);
    }

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;

    // P2-C: 每次进入先复位 touch_goal_, 由后续逻辑显式置位
    touch_goal_ = false;
    
    // 保存上一次的局部目标时间戳（用于 MINCO 初始化）
    planner_manager_->global_data_.last_glb_t_of_lc_tgt_ = planner_manager_->global_data_.glb_t_of_lc_tgt_;
    
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        // 更新当前局部目标时间戳（用于 MINCO 初始化）
        planner_manager_->global_data_.glb_t_of_lc_tgt_ = t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      // 更新为全局轨迹终点时间
      planner_manager_->global_data_.glb_t_of_lc_tgt_ = planner_manager_->global_data_.global_start_time_.toSec() + planner_manager_->global_data_.global_duration_;
      // P2-C: 已经把局部目标推到全局终点, 显式置 touch_goal_
      touch_goal_ = true;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
      // P2-C: 距离全局终点小于刹车距离 -> 视为 touch_goal, 末态零速
      touch_goal_ = true;
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }

    // (Fix A 已回退: 经验证沿全局轨迹回溯/侧向扫描在 substation 高密度场景中
    //  收益甚微 [0/30], 且会让 last_failure_reason_ 标签更乱。
    //  保持原 ego-planner 行为, "Local target in collision" 由 roughlyCheck 处理。)
  }

  void EGOReplanFSM::publishBspline() {

    auto info = &planner_manager_->local_data_;
    
    // 根据轨迹类型选择发布方式
    if (planner_manager_->pp_.use_minco_)
    {
      // MINCO 模式：直接发布 MINCO 轨迹消息
      publishMincoTraj();
    }
    else
    {
      // B 样条模式：发布 B 样条消息
      ego_planner::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = odom_pos_(2);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_.publish(bspline);
    }
  }

  void EGOReplanFSM::publishMincoTraj()
  {
    auto info = &planner_manager_->local_data_;
    
    // 检查轨迹是否有效
    int piece_num = info->minco_traj_.getPieceNum();
    
    if (piece_num <= 0)
    {
      ROS_WARN("[FSM] publishMincoTraj: empty trajectory (piece_num=%d), skip publishing", piece_num);
      return;
    }
    
    ego_planner::MINCOTraj minco_msg;
    minco_msg.header.stamp = ros::Time::now();
    minco_msg.header.frame_id = "world";
    minco_msg.traj_id = info->traj_id_;
    minco_msg.order = 5;
    minco_msg.start_time = info->start_time_;
    
    // 获取轨迹的 piece 数量和持续时间
    Eigen::VectorXd durations = info->minco_traj_.getDurations();
    
    minco_msg.duration.reserve(piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      minco_msg.duration.push_back(durations(i));
    }
    
    // 提取多项式系数
    minco_msg.coef_x.reserve(piece_num * 6);
    minco_msg.coef_y.reserve(piece_num * 6);
    minco_msg.coef_z.reserve(piece_num * 6);
    
    for (int i = 0; i < piece_num; ++i)
    {
      poly_traj::CoefficientMat coeff = info->minco_traj_[i].getCoeffMat();
      for (int j = 0; j < 6; ++j)
      {
        minco_msg.coef_x.push_back(coeff(0, j));
        minco_msg.coef_y.push_back(coeff(1, j));
        minco_msg.coef_z.push_back(coeff(2, j));
      }
    }
    
    minco_pub_.publish(minco_msg);
    ROS_INFO_THROTTLE(2.0, "[FSM] Published MINCO traj: %d pieces, duration=%.2fs", piece_num, durations.sum());
  }

} // namespace ego_planner
