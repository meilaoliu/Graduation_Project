
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

    is_target_receive = false;

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
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

void EGOReplanFSM::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    end_pt_ << msg->pose.position.x, msg->pose.position.y, odom_pos_(2);

    //cout << "Triggered!" << endl;
    trigger_ = true;

    double error_x = msg->pose.position.x - odom_pos_(0);
    double error_y = msg->pose.position.y - odom_pos_(1);

    init_pt_ = odom_pos_;

    bool success = false;


    goal_last = end_pt_;

//    last_state_ = exec_state_;
//    yaw_start = atan2(error_y,error_x);
//    yaw_error = yaw_start-yaw;
//
//    //first step : calculate the yaw error
//    if(abs(yaw_error)>PI)
//    {
//        yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
//    }
//    if(abs(yaw_error)>PI/2.0)
//    {
//        if(yaw>0)
//        {
//            yaw -= PI;
//        }else if(yaw<0)
//        {
//            yaw += PI;
//        }
//        changeDirection();
//        //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
//        yaw_error = yaw_start - yaw;
//
//    }
//    if(abs(yaw_error)>yaw_error_max)
//    {
//        cmd_vel.twist.linear.x = 0;
//        cmd_vel.twist.angular.z = yaw_error/abs(yaw_error)*w_adjust;
//        bool success = false;
//        end_pt_ << msg->point.x, msg->point.y, msg->point.z;
//        success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
//        changeFSMExecState(ADJUST_POSE, "TRIG");
//        return;
//    }

    success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

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

        //goal is too close to current pose

        /*** FSM ***/
        if (exec_state_ == WAIT_TARGET)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            is_target_receive = true;
        }
        else if (exec_state_ == EXEC_TRAJ)
        {
            // 检查新目标是否在车辆前方
            // 计算当前速度方向（车头朝向）
            double current_heading = yaw; // 使用当前航向角
            double target_dir = atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
            double angle_diff = target_dir - current_heading;
            // 归一化到 [-PI, PI]
            while (angle_diff > PI) angle_diff -= 2 * PI;
            while (angle_diff < -PI) angle_diff += 2 * PI;
            
            // 如果目标在后方（角度差 > 90度），强制停止并从静止重新规划
            if (abs(angle_diff) > PI / 2.0)
            {
                ROS_WARN("New target is behind! Stopping and replanning from rest. angle_diff=%.1f deg", 
                         abs(angle_diff) * 180.0 / PI);
                // 发送停止命令
                callEmergencyStop(odom_pos_);
                // 进入 GEN_NEW_TRAJ 而不是 REPLAN_TRAJ，这样会从静止开始规划
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            }
            else
            {
                // 目标在前方，正常重规划
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
            }
            is_target_receive = true;
        }

        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

        // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
        //visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
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

  void EGOReplanFSM::checkYawError() {
      yaw_error = yaw_start-yaw;
      //first step : find the real yaw error
      if(abs(yaw_error)>yaw_error_max)
      {
          //if yaw error is larger than PI,it means the symbol between current yaw and target yaw is different
          if(abs(yaw_error)>PI)
          {
              //calculate the real yaw error with symbol
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
              //if real yaw error larger than PI/2, change the direction of robot
              if(abs(yaw_error)>PI/2)
              {
                  // forward_only 模式下不改变方向，而是需要原地掉头
                  if (!forward_only_) {
                  changeDirection();
                  }
              }
          }
          else
          {
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;

          }
      }
  }

  double EGOReplanFSM::calculateYawError(double yaw_cur,double yaw_target) {
      double error = yaw_target - yaw_cur;
      if(abs(error)>PI)
      {
          error = error - error/abs(error)*2*PI;
          return error;
      }
      else
      {
          return error;
      }
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
//        if(last_state_!=WAIT_TARGET&&last_state_!=EMERGENCY_STOP)
//        {
//            callEmergencyStop(odom_pos_);
//        }
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
            changeFSMExecState(EXEC_TRAJ, "FSM");
            adjust_cmd_pub_.publish(is_adjust_pose);
            auto info = &planner_manager_->local_data_;
            info->start_time_ = ros::Time::now();
            publishBspline();
        }
        break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_;
      
      // 给一个指向目标的微小初速度，引导轨迹方向（特别是当目标在身后时）
      // 这样生成的轨迹起始切线会指向目标，让 traj_server 正确检测到需要掉头
      if (have_target_) {
          double heading = atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
          start_vel_ << 0.01 * cos(heading), 0.01 * sin(heading), 0;
      } else {
          start_vel_ << 0, 0, 0;
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

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
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
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
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
          changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
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
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
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

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
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

    // 强制检查速度方向与目标方向
    double current_vel_dir = atan2(start_vel_(1), start_vel_(0));
    double target_dir = atan2((end_pt_ - odom_pos_)(1), (end_pt_ - odom_pos_)(0));
    double dir_diff = target_dir - current_vel_dir;
    while (dir_diff > PI) dir_diff -= 2 * PI;
    while (dir_diff < -PI) dir_diff += 2 * PI;

    // 如果速度方向相反（>90度）且速度不为0
    // 解决绕圈问题：强制停车，不使用当前速度作为初始条件
    if (abs(dir_diff) > PI / 2.0 && start_vel_.norm() > 0.1)
    {
        ROS_WARN("Target is behind! Force stop to avoid circling.");
        
        // 1. 强制将起始速度设为 0（模拟急刹车）
        start_vel_.setZero();
        start_acc_.setZero();
        
        // 2. 触发掉头逻辑
        yaw_start = target_dir;
        yaw_error = yaw_start - yaw;
        if (abs(yaw_error) > PI) yaw_error -= yaw_error / abs(yaw_error) * 2 * PI;
        
        if (abs(yaw_error) > yaw_error_max)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = yaw_error / abs(yaw_error) * w_adjust;
            changeFSMExecState(ADJUST_POSE, "TRIG");
            return false;
        }
    }

    yaw_start = atan2((end_pt_-odom_pos_)(1),(end_pt_-odom_pos_)(0));
    yaw_error = yaw_start-yaw;

    if(is_target_receive)
    {
        if(abs(yaw_error)>PI)
        {
            yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
        }
        
        if (forward_only_)
        {
            // forward_only 模式：如果目标在身后，需要原地掉头
            if(abs(yaw_error) > yaw_error_max)
            {
                ROS_WARN("[FSM forward_only planFromCurrentTraj] Need to turn: error=%.1f deg", 
                         yaw_error * 180.0 / PI);
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;
                changeFSMExecState(ADJUST_POSE, "TRIG");
                is_target_receive = false;
                return false;
            }
        }
        else
        {
            // 原有逻辑：允许倒车
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
            start_vel_ <<-start_vel_(0),-start_vel_(1),0;
            start_acc_ <<-start_acc_(0),-start_acc_(1),0;
            std_msgs::UInt8 stop_cmd;
            stop_cmd.data = 1;
            stop_pub.publish(stop_cmd);
            }
        }
        is_target_receive = false;
    }
    //first step : calculate the yaw error


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
    // - start_time_ 未设置
    if (exec_state_ == WAIT_TARGET || exec_state_ == ADJUST_POSE || 
        exec_state_ == INIT || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos_cur;
      if (info->use_minco_traj_)
      {
        // 使用 MINCO 轨迹精确评估
        pos_cur = info->minco_traj_.getPos(t);
      }
      else
      {
        pos_cur = info->position_traj_.evaluateDeBoorT(t);
      }
      Eigen::Vector2d pos_cur2d;
      pos_cur2d << pos_cur(0),pos_cur(1);
      if (map->getInflateOccupancy2d(pos_cur2d))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
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
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
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
    if (info->use_minco_traj_)
    {
      // 使用 MINCO 轨迹时，仍然发布 B 样条近似（用于可视化和兼容性）
      // 但实际控制应该基于 MINCO 轨迹
      ROS_INFO_ONCE("Publishing B-spline approximation of MINCO trajectory");
    }

    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    // Emergency stop 总是使用 B 样条（因为是静止点）
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

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
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
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

  void EGOReplanFSM::publishBspline() {

      auto info = &planner_manager_->local_data_;
      // 注意：start_time_ 应该在调用 publishBspline() 之前设置，这里不再重复设置
      // 这样可以确保掉头完成后轨迹从正确的时间开始
    /* publish traj */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    if (planner_manager_->pp_.use_minco_)
    {
      // MINCO 轨迹：采样后发布为 B 样条格式（供 MPC 使用）
      // 注意：这里发布的是采样点，MPC 需要适配
      double dt = 0.1;
      int sample_num = std::max(5, (int)(info->duration_ / dt));
      dt = info->duration_ / sample_num;
      
      bspline.pos_pts.reserve(sample_num + 1);
      bspline.knots.reserve(sample_num + 4); // B 样条节点数
      
      // 采样位置点
      for (int i = 0; i <= sample_num; ++i)
      {
        double t = std::min(i * dt, info->duration_);
        Eigen::Vector3d pt = info->minco_traj_.getPos(t);
        geometry_msgs::Point gpt;
        gpt.x = pt(0);
        gpt.y = pt(1);
        gpt.z = odom_pos_(2); // 使用当前 Z 高度
        bspline.pos_pts.push_back(gpt);
      }
      
      // 生成节点向量（均匀 B 样条）
      for (int i = 0; i < sample_num + 4; ++i)
      {
        bspline.knots.push_back(i * dt);
      }
    }
    else
    {
      // B 样条轨迹：直接发布控制点
      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      Eigen::Vector3d point_temp;
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
    }

    // 根据轨迹类型选择发布方式
    if (planner_manager_->pp_.use_minco_ && info->use_minco_traj_)
    {
      // MINCO 模式：只发布 MINCO 轨迹消息，不发布 B 样条近似
      // 这样可以避免 traj_server 中的模式切换混乱
      publishMincoTraj();
    }
    else
    {
      // B 样条模式：发布 B 样条消息
      bspline_pub_.publish(bspline);
    }
  }

  void EGOReplanFSM::publishMincoTraj()
  {
    auto info = &planner_manager_->local_data_;
    
    ego_planner::MINCOTraj minco_msg;
    minco_msg.header.stamp = ros::Time::now();
    minco_msg.header.frame_id = "world";
    minco_msg.traj_id = info->traj_id_;
    minco_msg.order = 5;
    minco_msg.start_time = info->start_time_;
    
    // 获取轨迹的 piece 数量和持续时间
    int piece_num = info->minco_traj_.getPieceNum();
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
  }

} // namespace ego_planner
