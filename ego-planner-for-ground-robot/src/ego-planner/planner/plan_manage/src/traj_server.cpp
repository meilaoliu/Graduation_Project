#include "bspline_opt/uniform_bspline.h"
#include "minco_opt/poly_traj_utils.hpp"
#include "MPC.hpp"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ego_planner/Bspline.h"
#include "ego_planner/MINCOTraj.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
//#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "time.h"
#include <fstream>
#include <iomanip>
#include <signal.h>
#include <cmath>
#include <sys/stat.h>

#define PI 3.1415926
#define N 15

ros::Publisher vel_cmd_pub;

//quadrotor_msgs::PositionCommand cmd;
geometry_msgs::Twist cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool use_minco_traj_ = false; // 标志：是否使用 MINCO 轨迹
bool is_orientation_init = false;
vector<UniformBspline> traj_;
boost::shared_ptr<poly_traj::Trajectory> minco_traj_; // MINCO 轨迹
double traj_duration_;
ros::Time start_time_,time_s,time_e;
int traj_id_;

Eigen::Vector3d odom_pos_,odom_vel_;
Eigen::Quaterniond odom_orient_;

MPC_controller mpc_controller;
double roll,pitch,yaw;
geometry_msgs::PoseStamped pose_cur;
tf::Quaternion quat;
std_msgs::UInt8 is_adjust_pose;
std_msgs::UInt8 dir;

// ==========  data logging ==========
struct CmdLogEntry {
    double time;       // relative to system start
    double ref_vx, ref_vy;   // reference velocity from trajectory
    double ref_ax, ref_ay;   // reference acceleration from trajectory
    double ref_speed;        // |v|
    double ref_curvature;    // kappa = (vx*ay - vy*ax) / |v|^3
    double cmd_v, cmd_w;     // MPC output: linear vel, angular vel
    double odom_x, odom_y, odom_yaw;  // robot actual state
};
std::vector<CmdLogEntry> cmd_log_entries_;
ros::Time system_start_time_;
bool system_start_time_set_ = false;

std::string getLogDir() {
    std::string pkg_path = ros::package::getPath("ego_planner");
    // pkg_path = .../src/ego-planner/planner/plan_manage
    // target  = .../src/benchmark
    std::string log_dir = pkg_path + "/../../../../src/benchmark";
    // Resolve to canonical form
    char resolved[PATH_MAX];
    if (realpath(log_dir.c_str(), resolved)) {
        log_dir = std::string(resolved);
    }
    mkdir(log_dir.c_str(), 0755);
    return log_dir;
}

void saveCmdLogToFile() {
    if (cmd_log_entries_.empty()) {
        ROS_WARN("[TrajServer] No log entries to save.");
        return;
    }
    std::string csv_path = getLogDir() + "/cmd_profile.csv";
    std::ofstream csv_writer(csv_path, std::ios::out | std::ios::trunc);
    csv_writer << "time,ref_vx,ref_vy,ref_ax,ref_ay,ref_speed,ref_curvature,cmd_v,cmd_w,odom_x,odom_y,odom_yaw" << std::endl;
    csv_writer << std::fixed << std::setprecision(6);
    for (const auto &e : cmd_log_entries_) {
        csv_writer << e.time << "," << e.ref_vx << "," << e.ref_vy << ","
                   << e.ref_ax << "," << e.ref_ay << "," << e.ref_speed << ","
                   << e.ref_curvature << "," << e.cmd_v << "," << e.cmd_w << ","
                   << e.odom_x << "," << e.odom_y << "," << e.odom_yaw << std::endl;
    }
    csv_writer.close();
    ROS_INFO("[TrajServer] Saved %zu log entries to %s", cmd_log_entries_.size(), csv_path.c_str());
}

void sigintHandler(int sig) {
    saveCmdLogToFile();
    ros::shutdown();
}
// ========== End logging ==========

enum DIRECTION {POSITIVE=0,NEGATIVE=1};

// yaw control
double t_step;

// 前进模式控制
bool forward_only_ = true;  // 默认只允许前进，不允许倒车

std_msgs::UInt8 stop_command;

////time record
clock_t start_clock,end_clock;
double duration;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);


  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  // B 样条模式：只有当 FSM 使用 B 样条优化器时才会收到此消息
  // MINCO 模式下会收到 MINCOTraj 消息，不会进入此回调
  use_minco_traj_ = false;
  receive_traj_ = true;

}

void mincoTrajCallback(ego_planner::MINCOTrajConstPtr msg)
{
  
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters! duration_size=%zu, coef_x_size=%zu", 
              msg->duration.size(), msg->coef_x.size());
    return;
  }
  if (msg->duration.size() == 0)
  {
    ROS_WARN("[traj_server] Empty trajectory, skip!");
    return;
  }

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

  minco_traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = minco_traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  use_minco_traj_ = true;
  receive_traj_ = true;
  
  ROS_INFO_THROTTLE(2.0, "[traj_server] MINCO traj: %d pieces, duration=%.2fs", piece_nums, traj_duration_);
}

void poseCallback(geometry_msgs::PoseStampedConstPtr msg)
{
    pose_cur = *msg;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
}

void adjust_yaw_Callback(std_msgs::UInt8ConstPtr msg)
{
    is_adjust_pose = *msg;
}

void dirCallback(const std_msgs::UInt8ConstPtr& msg)
{
    dir = *msg;
}

void MPC_calculate(double &t_cur)
{
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::MatrixXd u_k;
    Eigen::Vector3d pos_r,pos_r_1,pos_final,v_r_1,v_r_2,X_k;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r,x_r_1,x_r_2;
    double v_linear_1,w;
    double t_k,t_k_1;

    //ROS_INFO("Run to here!");

    // 根据轨迹类型获取速度和位置
    Eigen::Vector3d vel_start, pos_final_3d;
    if (use_minco_traj_)
    {
      vel_start = minco_traj_->getVel(t_cur);
      pos_final_3d = minco_traj_->getPos(traj_duration_);
    }
    else
    {
      vel_start = traj_[1].evaluateDeBoor(t_cur);
      pos_final_3d = traj_[0].evaluateDeBoor(traj_duration_);
    }
    
    double yaw_start;
    // 如果起始速度太小，使用稍后的点来计算方向，避免 atan2(0,0)
    if (vel_start.norm() < 0.1)
    {
        Eigen::Vector3d pos_now, pos_next;
        double t_next = std::min(t_cur + 0.5, traj_duration_);
        if (use_minco_traj_)
        {
            pos_now = minco_traj_->getPos(t_cur);
            pos_next = minco_traj_->getPos(t_next);
        }
        else
        {
            pos_now = traj_[0].evaluateDeBoor(t_cur);
            pos_next = traj_[0].evaluateDeBoor(t_next);
        }
        yaw_start = atan2((pos_next - pos_now)(1), (pos_next - pos_now)(0));
        // ROS_INFO_THROTTLE(1.0, "[Traj Server] Low speed, using lookahead for yaw_start: %.2f", yaw_start);
    }
    else
    {
        yaw_start = atan2(vel_start(1), vel_start(0));
    }

    bool is_orientation_adjust=false;
    double orientation_adjust=0;
    pos_final = pos_final_3d;

    // forward_only 模式：检查轨迹方向与车头方向
    if (forward_only_)
    {
        double yaw_diff = yaw_start - yaw;
        // 归一化到 [-PI, PI]
        while (yaw_diff > PI) yaw_diff -= 2 * PI;
        while (yaw_diff < -PI) yaw_diff += 2 * PI;
        
        // 调试日志：帮助分析为什么不掉头
        ROS_INFO_THROTTLE(0.5, "[TrajServer DEBUG] V=%.2f, yaw=%.1f, traj=%.1f, diff=%.1f", 
            vel_start.norm(), yaw * 180.0 / PI, yaw_start * 180.0 / PI, yaw_diff * 180.0 / PI);

        // 如果轨迹方向与车头方向相差超过 90 度，停止并原地转向
        if (abs(yaw_diff) > PI / 2.0)
        {
            cmd.linear.x = 0;  // 停止前进
            // 原地转向，朝向轨迹方向
            double turn_speed = 0.5;  // 转向速度 rad/s
            cmd.angular.z = (yaw_diff > 0) ? turn_speed : -turn_speed;
            
            static int print_count = 0;
            if (print_count++ % 10 == 0)
            {
                ROS_WARN("[Traj server] Turning: yaw=%.1f, traj=%.1f, diff=%.1f deg", 
                         yaw * 180.0 / PI, yaw_start * 180.0 / PI, yaw_diff * 180.0 / PI);
            }
            
            vel_cmd_pub.publish(cmd);
            return;  // 不执行 MPC，等待转向完成
        }
    }

        is_orientation_init=true;
        for(int i=0;i<N;i++)
        {

            t_k = t_cur+i*t_step;
            t_k_1 = t_cur+(i+1)*t_step;

            // 根据轨迹类型获取位置和速度
            if (use_minco_traj_)
            {
              pos_r = minco_traj_->getPos(t_k);
              pos_r_1 = minco_traj_->getPos(t_k_1);
              v_r_1 = minco_traj_->getVel(t_k);
              v_r_2 = minco_traj_->getVel(t_k_1);
            }
            else
            {
            pos_r = traj_[0].evaluateDeBoor(t_k);
            pos_r_1 = traj_[0].evaluateDeBoor(t_k_1);
              v_r_1 = traj_[1].evaluateDeBoor(t_k);
              v_r_2 = traj_[1].evaluateDeBoor(t_k_1);
            }

            x_r(0) = pos_r(0);
            x_r(1) = pos_r(1);

            v_r_1(2)=0;
            v_r_2(2)=0;
            v_linear_1 = v_r_1.norm();
            if((t_k-traj_duration_)>=0)
            {
                x_r(2) = atan2((pos_r-pos_final)(1),(pos_r-pos_final)(0));
            }
            else
            {
                //x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));
                x_r(2) = atan2(v_r_1(1),v_r_1(0));
            }



            double yaw1 = atan2(v_r_1(1),v_r_1(0));
            double yaw2 = atan2(v_r_2(1),v_r_2(0));

            if(abs(yaw2-yaw1)>PI)
            {
                //ROS_WARN("orientation suddenly change !");
                //cout<<"current index : "<<i+1<<endl;
                //cout<<"yaw 1 : "<<yaw1<<endl;
                //cout<<"yaw 2 : "<<yaw2<<endl;
                is_orientation_adjust = true;
                if((yaw2-yaw1)<0)
                {
                    orientation_adjust = 2*PI;
                    w = (2*PI+(yaw2-yaw1))/t_step;
                }
                else
                {
                    w = ((yaw2-yaw1)-2*PI)/t_step;
                    orientation_adjust = -2*PI;
                }
            }
            else
            {
                w = (yaw2-yaw1)/t_step;
            }

            if(is_orientation_adjust==true)
            {
                //cout<<"orientation before adjust : "<< x_r(2)<<endl;
                x_r(2) +=orientation_adjust;
                //cout<<"orientation after adjust : "<< x_r(2)<<endl;
            }

            u_r(0) = v_linear_1;
            u_r(1) = w;
//                if(t_c>(tp-5*t_step))
//                {
//                    cout<<"Ur "<<i+1<<" : "<<endl<<u_r<<endl;
//                    cout<<"Xr "<<i+1<<" : "<<endl<<x_r<<endl;
//                }
            X_r.push_back(x_r);
            U_r.push_back(u_r);
        }

        //X_k(0) = odom_map.pose.pose.position.x - my_map.info.origin.position.x;
        //X_k(1) = odom_map.pose.pose.position.y - my_map.info.origin.position.y;
        X_k(0) = odom_pos_(0);
        X_k(1) = odom_pos_(1);
        if(yaw/X_r[0](2)<0&&abs(yaw)>(PI*5/6))
        {
            if(yaw<0)
            {
                X_k(2) = yaw + 2*PI;
            }
            else
            {
                X_k(2) = yaw - 2*PI;
            }
        }
        else
        {
            X_k(2) = yaw;
        }
        // cout<<"xr  : "<<X_r[0]<<endl;
        // cout<<"xk  : "<<X_k<<endl;

        //ROS_INFO("Run to here!");
        u_k = mpc_controller.MPC_Solve_qp(X_k,X_r,U_r,N);


//            cout<<"Xk "<<" : "<<endl<<X_k<<endl;
        double vel_cmd = u_k.col(0)(0);
        
        // 前进模式控制
        if (forward_only_)
        {
            // 只允许前进模式：
            // 1. 忽略方向切换（不倒车）
            // 2. 如果 MPC 输出负速度，强制为 0
            if (vel_cmd < 0)
        {
                vel_cmd = 0;
            }
            cmd.linear.x = vel_cmd;  // 始终正向
        }
        else
        {
            // 双向模式：根据方向切换
            if(dir.data == NEGATIVE)
            {
                cmd.linear.x = -vel_cmd;
            }
            else
            {
                cmd.linear.x = vel_cmd;
            }
        }

        cmd.angular.z = u_k.col(0)(1);
       static int conut1 = 0;
       conut1+=1;
       // 降低调试输出频率，每200次输出一次关键信息
       if(conut1%200==0)
       {
           ROS_INFO("[MPC] cmd: v=%.2fm/s, w=%.2frad/s | ref: v=%.2f, w=%.2f", 
                    u_k.col(0)(0), u_k.col(0)(1), U_r[0](0), U_r[0](1));
           conut1=0;
       }


        vel_cmd_pub.publish(cmd);
//        control_times+=1;
//        //cout<<"control_times : "<<control_times<<endl;
//        //cout<<"current t : "<<t_c<<endl;
//        if(t_c>tp)
//        {
//            is_trajectory_trace=false;
//            is_orientation_init=false;
//            web_cmd_vel.angular.z = 0;
//            web_cmd_vel.linear.x = 0;
//            web_cmd_pub.publish(web_cmd_vel);
//            control_times=0;
//        }
        //ros::Time t_end = ros::Time::now();
        //ROS_INFO("control total time : %5.3f ms",(t_end-t_start).toSec()*1000);
   // }

}

void stopCallback(std_msgs::UInt8ConstPtr msg)
{
    stop_command = *msg;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
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

    // forward_only 模式下，不根据 dir 修改 yaw
    // 这样 yaw 始终是真实的车头朝向
    if (!forward_only_ && dir.data == NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }

}

void cmdCallback(const ros::TimerEvent &e)
{
    /* no publishing before receive traj_ */
    if (stop_command.data==1)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub.publish(cmd);
        return;
    }

    // 当 FSM 处于 ADJUST_POSE 状态时，停止轨迹跟踪
    // FSM 会直接控制车辆转向，traj_server 不应该发送任何控制命令
    // 否则两者会冲突导致车辆抖动
    if (is_adjust_pose.data == 1)
    {
        // 不发送任何命令，让 FSM 完全控制
        return;
    }

    if (!receive_traj_)
        return;
    //ROS_WARN("Run here !");
    ros::Time time_s = ros::Time::now();
    double t_cur = (time_s - start_time_).toSec();

//    Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
//    Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
//    double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));

    static ros::Time time_last = ros::Time::now();

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        start_clock = clock();
        MPC_calculate(t_cur);
        end_clock = clock();
        duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;

        // ---- SUPER-style logging: record at control callback ----
        if (!system_start_time_set_) {
            system_start_time_ = time_s;
            system_start_time_set_ = true;
        }
        CmdLogEntry entry;
        entry.time = (time_s - system_start_time_).toSec();

        Eigen::Vector3d ref_vel, ref_acc;
        if (use_minco_traj_) {
            ref_vel = minco_traj_->getVel(t_cur);
            ref_acc = minco_traj_->getAcc(t_cur);
        } else {
            ref_vel = traj_[1].evaluateDeBoor(t_cur);
            ref_acc = traj_[2].evaluateDeBoor(t_cur);
        }
        entry.ref_vx = ref_vel(0);
        entry.ref_vy = ref_vel(1);
        entry.ref_ax = ref_acc(0);
        entry.ref_ay = ref_acc(1);
        double v2 = ref_vel(0)*ref_vel(0) + ref_vel(1)*ref_vel(1);
        entry.ref_speed = std::sqrt(v2);
        double v3 = v2 * entry.ref_speed;
        entry.ref_curvature = (v3 > 1e-6) ? (ref_vel(0)*ref_acc(1) - ref_vel(1)*ref_acc(0)) / v3 : 0.0;
        entry.cmd_v = cmd.linear.x;
        entry.cmd_w = cmd.angular.z;
        entry.odom_x = odom_pos_(0);
        entry.odom_y = odom_pos_(1);
        entry.odom_yaw = yaw;
        cmd_log_entries_.push_back(entry);
        // ---- End logging ----
    }
    else if (t_cur >= traj_duration_)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub.publish(cmd);
        is_orientation_init=false;
    }
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }
    time_last = time_s;

    vel_cmd_pub.publish(cmd);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigintHandler);
  ros::NodeHandle node("~");

  std::string cmd_topic,pose_topic;
  node.getParam("/ego_planner_node/fsm/pose_topic",pose_topic);
  node.getParam("/ego_planner_node/fsm/vel_topic",cmd_topic);


  ros::Subscriber bspline_sub = node.subscribe("/planning/bspline", 10, bsplineCallback);
  ros::Subscriber minco_sub = node.subscribe("/planning/minco_traj", 10, mincoTrajCallback);
  ros::Subscriber pose_sub = node.subscribe(pose_topic, 10, poseCallback);
  ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
  ros::Subscriber stop_sub = node.subscribe("/emergency_stop",10,stopCallback);
  ros::Subscriber adjust_yaw_sub = node.subscribe("/is_adjust_yaw",10,adjust_yaw_Callback);
  ros::Subscriber command_sub = node.subscribe("/direction",10,dirCallback);

  mpc_controller.MPC_init(node);
  vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  stop_command.data = 0;
  dir.data = POSITIVE;
  t_step = 0.03;
  
  // 读取前进模式参数
  // 参数在 ego_planner_node 节点内定义，所以路径是 /ego_planner_node/fsm/forward_only
  // 按优先级尝试多个路径：
  // 1. /ego_planner_node/fsm/forward_only (与 advanced_param.xml 定义一致)
  // 2. /forward_only (FSM 设置的全局参数，用于兼容)
  if (!ros::param::get("/ego_planner_node/fsm/forward_only", forward_only_)) {
      if (!ros::param::get("/forward_only", forward_only_)) {
          // 都没找到，使用默认值
          forward_only_ = true;
          ROS_WARN("[Traj server]: forward_only param not found, using default: true");
      }
  }
  ROS_INFO("[Traj server]: forward_only = %s", forward_only_ ? "true (no reverse)" : "false (bidirectional)");


  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.03), cmdCallback);

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}