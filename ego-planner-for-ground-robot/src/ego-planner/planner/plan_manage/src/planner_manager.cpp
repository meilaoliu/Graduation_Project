#include <plan_manage/planner_manager.h>
#include <thread>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <ros/package.h>
#include <cmath>

namespace ego_planner
{
    namespace
    {
        Eigen::Vector3d projectToPlane(Eigen::Vector3d p, const double z)
        {
            p.z() = z;
            return p;
        }

        Eigen::Vector3d planarDerivative(Eigen::Vector3d d)
        {
            d.z() = 0.0;
            return d;
        }
    }

    // SECTION interfaces for setup and query

    EGOPlannerManager::EGOPlannerManager() {}

    EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

    void EGOPlannerManager::computeCurrentTrajMetrics(double &traj_length, double &max_speed, double &max_curvature)
    {
        traj_length = 0.0;
        max_speed = 0.0;
        max_curvature = 0.0;

        const double duration = local_data_.duration_;
        const double dt_sample = 0.02;
        if (duration <= 1e-6)
        {
            return;
        }

        Eigen::Vector3d last_pt;
        if (local_data_.use_minco_traj_)
            last_pt = local_data_.minco_traj_.getPos(0.0);
        else
            last_pt = local_data_.position_traj_.evaluateDeBoorT(0.0);

        for (double t = dt_sample; t <= duration + 1e-6; t += dt_sample)
        {
            const double t_eval = std::min(t, duration);
            Eigen::Vector3d pt;
            if (local_data_.use_minco_traj_)
                pt = local_data_.minco_traj_.getPos(t_eval);
            else
                pt = local_data_.position_traj_.evaluateDeBoorT(t_eval);
            traj_length += (pt - last_pt).norm();
            last_pt = pt;

            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            if (local_data_.use_minco_traj_)
            {
                vel = local_data_.minco_traj_.getVel(t_eval);
                acc = local_data_.minco_traj_.getAcc(t_eval);
            }
            else
            {
                vel = local_data_.velocity_traj_.evaluateDeBoorT(t_eval);
                acc = local_data_.acceleration_traj_.evaluateDeBoorT(t_eval);
            }
            double vx = vel(0), vy = vel(1), ax = acc(0), ay = acc(1);
            double spd = std::sqrt(vx * vx + vy * vy);
            if (spd > max_speed)
                max_speed = spd;
            if (spd > 0.1)
            {
                double kappa = std::abs(vx * ay - vy * ax) / (spd * spd * spd);
                if (kappa > max_curvature)
                    max_curvature = kappa;
            }
        }
    }

    void EGOPlannerManager::appendPlanningStats(double total_plan_time_sec, int iterations, const std::string &planner_type)
    {
        double traj_length, max_speed, max_curvature;
        computeCurrentTrajMetrics(traj_length, max_speed, max_curvature);

        std::string log_dir = ros::package::getPath("ego_planner") + "/../../../../src/benchmark";
        char resolved[PATH_MAX];
        if (realpath(log_dir.c_str(), resolved))
            log_dir = std::string(resolved);
        mkdir(log_dir.c_str(), 0755);

        std::string csv_path = log_dir + "/planning_stats.csv";
        bool file_exists = std::ifstream(csv_path).good();
        std::ofstream fout(csv_path, std::ios::app);
        if (!fout.is_open())
        {
            return;
        }

        if (!file_exists)
        {
            fout << "planner_type,traj_id,plan_time_ms,traj_length_m,duration_s,max_speed,max_curvature,iterations,success" << std::endl;
        }

        fout << std::fixed << std::setprecision(4)
             << planner_type << ","
             << local_data_.traj_id_ << ","
             << total_plan_time_sec * 1000.0 << ","
             << traj_length << ","
             << local_data_.duration_ << ","
             << max_speed << ","
             << max_curvature << ","
             << iterations << ","
             << 1 << std::endl;
        fout.close();
    }

    void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
    {
        /* read algorithm parameters */

        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/max_w", pp_.max_w_, -1.0);
        nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, 1.5);
        nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
        nh.param("manager/use_minco", use_minco_, false);
        nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs_, false);
        
        // 将 use_minco_ 同步到 pp_.use_minco_，确保外部代码能正确访问
        pp_.use_minco_ = use_minco_;

        local_data_.traj_id_ = 0;
        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);

        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam(nh);
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(300, 300));

        if (use_minco_)
        {
            minco_optimizer_.reset(new PolyTrajOptimizer);
            minco_optimizer_->setParam(nh);
            minco_optimizer_->setEnvironment(grid_map_);
            minco_optimizer_->setDroneId(-1); // Single robot
        }

        visualization_ = vis;
    }

    // Fix G: passthrough, FSM 用来侦测连续 init_collision_dense
    std::string EGOPlannerManager::getMincoLastFailureReason() const
    {
        if (minco_optimizer_)
            return minco_optimizer_->getLastFailureReason();
        return "";
    }

    // !SECTION

    // SECTION rebond replanning

    bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                          Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                          Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                          bool touch_goal)
    {
        // 如果使用 MINCO，调用 MINCO 重规划
        if (use_minco_)
        {
            return reboundReplanMinco(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, flag_polyInit, flag_randomPolyTraj, touch_goal);
        }

        static int count = 0;
        std::cout << endl
                  << "[rebo replan]: -------------------------------------" << count++ << std::endl;
        cout.precision(3);
        cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
             << endl;

        if ((start_pt - local_target_pt).norm() < 0.2)
        {
            cout << "Close to goal" << endl;
            continous_failures_count_++;
            return false;
        }

        ros::Time t_start = ros::Time::now();
        ros::Duration t_init, t_opt, t_refine;

        /*** STEP 1: INIT ***/
        double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        vector<Eigen::Vector3d> point_set, start_end_derivatives;
        static bool flag_first_call = true, flag_force_polynomial = false;
        bool flag_regenerate = false;
        do
        {
            point_set.clear();
            start_end_derivatives.clear();
            flag_regenerate = false;

            if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
            {
                flag_first_call = false;
                flag_force_polynomial = false;

                PolynomialTraj gl_traj;

                double dist = (start_pt - local_target_pt).norm();
                double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

                if (!flag_randomPolyTraj)
                {
                    gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
                }
                else
                {
                    Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
                    Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
                    Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                         (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                                         (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
                    random_inserted_pt.z() = start_pt.z();
                    Eigen::MatrixXd pos(3, 3);
                    pos.col(0) = projectToPlane(start_pt, start_pt.z());
                    pos.col(1) = random_inserted_pt;
                    pos.col(2) = projectToPlane(local_target_pt, start_pt.z());
                    Eigen::VectorXd t(2);
                    t(0) = t(1) = time / 2;
                    gl_traj = PolynomialTraj::minSnapTraj(
                        pos, planarDerivative(start_vel), planarDerivative(local_target_vel),
                        planarDerivative(start_acc), Eigen::Vector3d::Zero(), t);
                }

                double t;
                bool flag_too_far;
                ts *= 1.5; // ts will be divided by 1.5 in the next
                do
                {
                    ts /= 1.5;
                    point_set.clear();
                    flag_too_far = false;
                    Eigen::Vector3d last_pt = gl_traj.evaluate(0);
                    for (t = 0; t < time; t += ts)
                    {
                        Eigen::Vector3d pt = gl_traj.evaluate(t);
                        if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
                        {
                            flag_too_far = true;
                            break;
                        }
                        last_pt = pt;
                        point_set.push_back(pt);
                    }
                } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
                t -= ts;
                start_end_derivatives.push_back(gl_traj.evaluateVel(0));
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
                start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
            }
            else // Initial path generated from previous trajectory.
            {

                double t;
                double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

                vector<double> pseudo_arc_length;
                vector<Eigen::Vector3d> segment_point;
                pseudo_arc_length.push_back(0.0);
                for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
                {
                    segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                    if (t > t_cur)
                    {
                        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                    }
                }
                t -= ts;

                double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
                if (poly_time > ts)
                {
                    PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                                  local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                                  local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                                  local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

                    for (t = ts; t < poly_time; t += ts)
                    {
                        if (!pseudo_arc_length.empty())
                        {
                            segment_point.push_back(gl_traj.evaluate(t));
                            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                        }
                        else
                        {
                            ROS_ERROR("pseudo_arc_length is empty, return!");
                            continous_failures_count_++;
                            return false;
                        }
                    }
                }

                double sample_length = 0;
                double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
                size_t id = 0;
                do
                {
                    cps_dist /= 1.5;
                    point_set.clear();
                    sample_length = 0;
                    id = 0;
                    while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
                    {
                        if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                        {
                            point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                                (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                            sample_length += cps_dist;
                        }
                        else
                            id++;
                    }
                    point_set.push_back(local_target_pt);
                } while (point_set.size() < 7); // If the start point is very close to end point, this will help

                start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
                start_end_derivatives.push_back(Eigen::Vector3d::Zero());

                if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
                {
                    flag_force_polynomial = true;
                    flag_regenerate = true;
                }
            }
        } while (flag_regenerate);

        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

        vector<vector<Eigen::Vector3d>> a_star_pathes;
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        t_init = ros::Time::now() - t_start;

        static int vis_id = 0;
        visualization_->displayInitPathList(point_set, 0.2, 0);
        visualization_->displayAStarList(a_star_pathes, vis_id);

        t_start = ros::Time::now();

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        cout << "first_optimize_step_success=" << flag_step_1_success << endl;
        if (!flag_step_1_success)
        {
            // visualization_->displayOptimalList( ctrl_pts, vis_id );
            continous_failures_count_++;
            return false;
        }
        //visualization_->displayOptimalList( ctrl_pts, vis_id );

        t_opt = ros::Time::now() - t_start;
        t_start = ros::Time::now();

        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;
        if (!pos.checkFeasibility(ratio, false))
        {
         cout << "Need to reallocate time." << endl;

         Eigen::MatrixXd optimal_control_points;
         flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
         if (flag_step_2_success)
             pos = UniformBspline(optimal_control_points, 3, ts);
         }

         if (!flag_step_2_success)
         {
          printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
          continous_failures_count_++;
          return false;
          }

        t_refine = ros::Time::now() - t_start;

        // save planned results
        updateTrajInfo(pos, ros::Time::now());

        double total_plan_time = (t_init + t_opt + t_refine).toSec();
        cout << "total time:\033[42m" << total_plan_time << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

                appendPlanningStats(total_plan_time, bspline_optimizer_rebound_->getIterNum(), "bspline");

        // success. YoY
        continous_failures_count_ = 0;
        return true;
    }

    bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
    {
        if (use_minco_)
        {
            // 生成 MINCO 停止轨迹：一个静止在 stop_pos 的单段轨迹
            Eigen::Matrix3d headState, tailState;
            headState << stop_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            tailState << stop_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            
            poly_traj::MinJerkOpt stopMJO;
            stopMJO.reset(headState, tailState, 1);
            Eigen::MatrixXd innerPs;  // 空的内部点
            Eigen::VectorXd durations(1);
            durations(0) = 1.0;  // 1秒停止轨迹
            stopMJO.generate(innerPs, durations);
            
            setLocalTrajFromOpt(stopMJO, true);
            local_data_.start_time_ = ros::Time::now();
        }
        else
        {
            // 原有 B样条逻辑
        Eigen::MatrixXd control_points(3, 6);
        for (int i = 0; i < 6; i++)
        {
            control_points.col(i) = stop_pos;
        }
        updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());
        }

        return true;
    }

    bool EGOPlannerManager::ControlledStop(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &start_acc, const Eigen::Vector3d &stop_pos,
                                           double duration)
    {
        if (!use_minco_)
            return false;

        const double fixed_z = start_pos.z();
        Eigen::Matrix3d headState, tailState;
        Eigen::Vector3d start_pos_2d = start_pos;
        Eigen::Vector3d start_vel_2d = start_vel;
        Eigen::Vector3d start_acc_2d = start_acc;
        Eigen::Vector3d stop_pos_2d = stop_pos;
        start_pos_2d.z() = fixed_z;
        start_vel_2d.z() = 0.0;
        start_acc_2d.z() = 0.0;
        stop_pos_2d.z() = fixed_z;
        duration = std::max(0.5, duration);

        headState << start_pos_2d, start_vel_2d, start_acc_2d;
        tailState << stop_pos_2d, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        poly_traj::MinJerkOpt stopMJO;
        stopMJO.reset(headState, tailState, 1);
        Eigen::MatrixXd innerPs;
        Eigen::VectorXd durations(1);
        durations(0) = duration;
        stopMJO.generate(innerPs, durations);

        setLocalTrajFromOpt(stopMJO, true);
        local_data_.start_time_ = ros::Time::now();
        return true;
    }

    bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory
        if (waypoints.empty())
        {
            ROS_WARN("[global_traj] empty waypoint sequence.");
            return false;
        }

        const double fixed_z = start_pos.z();
        const Eigen::Vector3d start_pos_2d = projectToPlane(start_pos, fixed_z);
        const Eigen::Vector3d start_vel_2d = planarDerivative(start_vel);
        const Eigen::Vector3d start_acc_2d = planarDerivative(start_acc);
        const Eigen::Vector3d end_vel_2d = planarDerivative(end_vel);
        const Eigen::Vector3d end_acc_2d = planarDerivative(end_acc);

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos_2d);

        // 去重阈值：避免上一段终点 == 下一段拓扑起节点造成 inner waypoint 与起点重合，
        // 进而让 minSnapTraj 第一段 time=norm/v_max≈0、5阶多项式系数 1/T^5 爆炸。
        const double dedup_dist = 0.3; // m
        for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
        {
            if (std::fabs(waypoints[wp_i].z() - fixed_z) > 1e-3)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "[global_traj] project waypoint z %.3f to fixed plane z %.3f.",
                                  waypoints[wp_i].z(), fixed_z);
            }
            Eigen::Vector3d wp_2d = projectToPlane(waypoints[wp_i], fixed_z);
            const double gap = (wp_2d - points.back()).norm();
            if (gap < dedup_dist)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "[global_traj] drop near-duplicate waypoint #%zu at (%.2f, %.2f), "
                                  "gap to prev = %.3f m (< %.2f m).",
                                  wp_i, wp_2d.x(), wp_2d.y(), gap, dedup_dist);
                continue;
            }
            points.push_back(wp_2d);
        }

        if (points.size() < 2)
        {
            ROS_WARN("[global_traj] after dedup only %zu point(s) left; abort.", points.size());
            return false;
        }

        double total_len = 0;
        for (size_t i = 0; i + 1 < points.size(); i++)
        {
            total_len += (points[i + 1] - points[i]).norm();
        }

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        double dist_thresh = max(total_len / 8, 4.0);

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                    Eigen::Vector3d inter_pt =
                            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        // for ( int i=0; i<inter_points.size(); i++ )
        // {
        //   cout << inter_points[i].transpose() << endl;
        // }

        // write position matrix
        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        // 段时长下限：兜底防御任意奇异短段（如 inter_points 后仍出现 < 0.6s 的段），
        // 避免 5 阶多项式系数 1/T^5 让全局轨迹瞬时速度爆炸。
        const double min_seg_time = 0.6; // s
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
            if (time(i) < min_seg_time) time(i) = min_seg_time;
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel_2d, end_vel_2d, start_acc_2d, end_acc_2d, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos_2d, start_vel_2d, start_acc_2d, pos.col(1), end_vel_2d, end_acc_2d, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);
        if (visualization_)
        {
            visualization_->displayGlobalWaypointList(inter_points, 0.25, 0);
        }

        return true;
    }

    bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory
        const double fixed_z = start_pos.z();
        const Eigen::Vector3d start_pos_2d = projectToPlane(start_pos, fixed_z);
        const Eigen::Vector3d end_pos_2d = projectToPlane(end_pos, fixed_z);
        const Eigen::Vector3d start_vel_2d = planarDerivative(start_vel);
        const Eigen::Vector3d start_acc_2d = planarDerivative(start_acc);
        const Eigen::Vector3d end_vel_2d = planarDerivative(end_vel);
        const Eigen::Vector3d end_acc_2d = planarDerivative(end_acc);
        if (std::fabs(end_pos.z() - fixed_z) > 1e-3)
        {
            ROS_WARN_THROTTLE(2.0,
                              "[global_traj] project goal z %.3f to fixed plane z %.3f.",
                              end_pos.z(), fixed_z);
        }

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos_2d);
        const double dedup_dist = 0.3; // m，与 planGlobalTrajWaypoints 同步
        if ((end_pos_2d - start_pos_2d).norm() < dedup_dist)
        {
            ROS_WARN("[global_traj] goal is within %.2f m of current pos; skip global plan.", dedup_dist);
            return false;
        }
        points.push_back(end_pos_2d);

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        const double dist_thresh = 4.0;

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                    Eigen::Vector3d inter_pt =
                            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        // write position matrix
        int pt_num = inter_points.size();
        //ROS_INFO("point num : %d",inter_points.size());
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        const double min_seg_time = 0.6; // s
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
            if (time(i) < min_seg_time) time(i) = min_seg_time;
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel_2d, end_vel_2d, start_acc_2d, end_acc_2d, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos_2d, start_vel_2d, start_acc_2d, end_pos_2d, end_vel_2d, end_acc_2d, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {
        double t_inc;

        Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

        // std::cout << "ratio: " << ratio << std::endl;
        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        traj = UniformBspline(ctrl_pts, 3, ts);

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
        bspline_optimizer_rebound_->ref_pts_.clear();
        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

        return success;
    }

    void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
    {
        local_data_.use_minco_traj_ = false; // 标记使用 B 样条轨迹
        local_data_.start_time_ = time_now;
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                           Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;
        // double length = bspline.getLength(0.1);
        // int seg_num = ceil(length / pp_.ctrl_pt_dist);

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

    bool EGOPlannerManager::reboundReplanMinco(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                                               Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                               bool touch_goal)
    {
        ros::Time t_start = ros::Time::now();
        ros::Duration t_init, t_opt;

        static int count = 0;
        cout << "\033[47;30m\n[" << t_start << "] Replan Minco " << count++ << "\033[0m" << endl;

        // === [起点 v/a 安全 clamp] 切断 "上一帧失败 → odom v/a 异常 → BVP cost=1e+213 → -1007" 的反馈链 ===
        // 任意 NaN/Inf 直接置零; 模长超过 max_vel/max_acc 的等比缩放
        auto sanitize = [](Eigen::Vector3d &v, double lim, const char *tag) {
          if (!v.allFinite()) {
            ROS_WARN("[CLAMP] %s has NaN/Inf, reset to zero", tag);
            v.setZero();
            return;
          }
          double n = v.norm();
          if (n > lim && lim > 1e-6) {
            ROS_WARN("[CLAMP] %s norm=%.3f > %.3f, scale down", tag, n, lim);
            v *= (lim / n);
          }
        };
        sanitize(start_vel, pp_.max_vel_, "start_vel");
        sanitize(start_acc, pp_.max_acc_, "start_acc");
        const double fixed_z = start_pt(2);
        local_target_pt(2) = fixed_z;
        local_target_vel(2) = 0.0;
        start_vel(2) = 0.0;
        start_acc(2) = 0.0;

        /*** STEP 1: INIT ***/
        // P2-C: 优先使用上层推算的 touch_goal; 其仅在初始未设置时才起作用。
        const bool effective_touch_goal = touch_goal || global_data_.localTrajReachTarget();
        minco_optimizer_->setIfTouchGoal(effective_touch_goal);
        double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

        poly_traj::MinJerkOpt initMJO;
        if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                              flag_polyInit, flag_randomPolyTraj, ts, initMJO))
        {
            return false;
        }

        Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
        vector<std::pair<int, int>> segments;
        if (minco_optimizer_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
        {
            ROS_WARN_STREAM("[MINCO] Initial fine check failed: reason="
                            << minco_optimizer_->getLastFailureReason()
                            << ", start=" << start_pt.transpose()
                            << ", local_target=" << local_target_pt.transpose()
                            << ", ctrl_pts=" << cstr_pts.cols());
            return false;
        }

        t_init = ros::Time::now() - t_start;

        std::vector<Eigen::Vector3d> point_set;
        for (int i = 0; i < cstr_pts.cols(); ++i)
            point_set.push_back(cstr_pts.col(i));
        visualization_->displayInitPathList(point_set, 0.2, 0);

        t_start = ros::Time::now();

        /*** STEP 2: OPTIMIZE ***/
        bool flag_success = false;
        poly_traj::MinJerkOpt best_MJO;

        if (pp_.use_multitopology_trajs_)
        {
            // 多拓扑模式：生成多条候选轨迹
            vector<vector<Eigen::Vector3d>> vis_trajs;
            
            poly_traj::Trajectory initTraj = initMJO.getTraj();
            int PN = initTraj.getPieceNum();
            Eigen::MatrixXd all_pos = initTraj.getPositions();
            Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
            Eigen::Matrix<double, 3, 3> headState, tailState;
            headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
            tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
            
            std::vector<ConstraintPoints> trajs = minco_optimizer_->distinctiveTrajs(segments);
            Eigen::VectorXi success = Eigen::VectorXi::Zero(trajs.size());
            double final_cost, min_cost = 999999.0;
            
            for (int i = trajs.size() - 1; i >= 0; i--)
            {
                minco_optimizer_->setConstraintPoints(trajs[i]);
                minco_optimizer_->setUseMultitopologyTrajs(true);
                
                if (minco_optimizer_->optimizeTrajectory(headState, tailState,
                                                         innerPts, initTraj.getDurations(), final_cost))
                {
                    success[i] = true;
                    
                    if (final_cost < min_cost)
                    {
                        min_cost = final_cost;
                        best_MJO = minco_optimizer_->getMinJerkOpt();
                        flag_success = true;
                    }
                    
                    // 可视化
                    Eigen::MatrixXd ctrl_pts_temp = minco_optimizer_->getMinJerkOpt().getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
                    std::vector<Eigen::Vector3d> point_set_temp;
                    for (int j = 0; j < ctrl_pts_temp.cols(); j++)
                    {
                        point_set_temp.push_back(ctrl_pts_temp.col(j));
                    }
                    vis_trajs.push_back(point_set_temp);
                }
            }
            
            if (trajs.size() > 1)
            {
                cout << "\033[1;33m"
                     << "multi-trajs=" << trajs.size() << ",\033[1;0m"
                     << " Success:fail=" << success.sum() << ":" << success.size() - success.sum() << endl;
            }
            
            // 可视化多条轨迹（只在有成功轨迹时）
            if (!vis_trajs.empty())
            {
                visualization_->displayMultiOptimalPathList(vis_trajs, 0.1);
            }
        }
        else
        {
            // 单拓扑模式：只优化一条轨迹
            poly_traj::Trajectory initTraj = initMJO.getTraj();
            int PN = initTraj.getPieceNum();
            Eigen::MatrixXd all_pos = initTraj.getPositions();
            Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
            Eigen::Matrix<double, 3, 3> headState, tailState;
            headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
            tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
            double final_cost;
            flag_success = minco_optimizer_->optimizeTrajectory(headState, tailState,
                                                                innerPts, initTraj.getDurations(), final_cost);
            best_MJO = minco_optimizer_->getMinJerkOpt();
        }

        t_opt = ros::Time::now() - t_start;

        /*** STEP 3: Store and display results ***/
        cout << "Success=" << (flag_success ? "yes" : "no") << endl;
        if (flag_success)
        {
            static double sum_time = 0;
            static int count_success = 0;
            sum_time += (t_init + t_opt).toSec();
            count_success++;
            printf("Time:\033[42m%.3fms,\033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
                   (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000, t_opt.toSec() * 1000, sum_time / count_success * 1000);

            bool local_traj_ok = setLocalTrajFromOpt(best_MJO, effective_touch_goal);
            cstr_pts = best_MJO.getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
            if (local_traj_ok)
            {
                appendPlanningStats((t_init + t_opt).toSec(), minco_optimizer_->getIterNum(), "minco");
            }
            else
            {
                flag_success = false;
                ROS_WARN("[MINCO] Optimizer returned a trajectory, but PlannerManager rejected it before publishing");
            }
            
            // 单拓扑模式下显示最优轨迹（多拓扑模式已经在上面显示过了）
            if (!pp_.use_multitopology_trajs_)
            {
                if (cstr_pts.cols() == 0)
                {
                    ROS_WARN("cstr_pts is empty! cps_num_prePiece=%d", minco_optimizer_->get_cps_num_prePiece_());
                }
                visualization_->displayOptimalList(cstr_pts, 0);
            }

            if (flag_success)
                continous_failures_count_ = 0;
            else
                continous_failures_count_++;
        }
        else
        {
            cstr_pts = minco_optimizer_->getMinJerkOpt().getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
            visualization_->displayInitPathList(point_set, 0.2, 0);
            ROS_WARN_STREAM("[MINCO] Optimize failed: reason="
                            << minco_optimizer_->getLastFailureReason()
                            << ", lbfgs=" << minco_optimizer_->getLastLbfgsResult()
                            << ", restarts=" << minco_optimizer_->getLastRestartCount()
                            << ", rebounds=" << minco_optimizer_->getLastReboundCount()
                            << ", final_cost=" << minco_optimizer_->getLastFinalCost()
                            << ", start=" << start_pt.transpose()
                            << ", local_target=" << local_target_pt.transpose()
                            << ", init_ms=" << t_init.toSec() * 1000.0
                            << ", opt_ms=" << t_opt.toSec() * 1000.0);

            continous_failures_count_++;
        }

        return flag_success;
    }

    bool EGOPlannerManager::computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
        const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
        const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
        poly_traj::MinJerkOpt &initMJO)
    {

        static bool flag_first_call = true;

        if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
        {
            flag_first_call = false;

            /* basic params */
            Eigen::Matrix3d headState, tailState;
            Eigen::MatrixXd innerPs;
            Eigen::VectorXd piece_dur_vec;
            int piece_nums;
            // === 修复 A: BVP 初值 totaldur 距离自适应 ===
            // 原本固定 2.0s, 距离 5m 时初值会让中段 v ~2.5m/s, a ~9m/s² (远超限制 1.3/2.0),
            // LBFGS 在 max_restarts=3 内根本拉不回可行域 → 永远 max_restarts → FSM 死循环。
            // 用 dist / (0.7 * max_vel) 估计走完所需时间, 下限 2.0s 防止极近距离时分母太小。
            const double _init_dist = (start_pt - local_target_pt).head<2>().norm();
            const double _vref = std::max(0.5, pp_.max_vel_);
            const double init_of_init_totaldur = std::max(2.0, _init_dist / (0.7 * _vref));
            
            // 优化起点速度方向：仅在速度很小时调整
            // 速度不为零时保持原始速度，让FSM的ADJUST_POSE处理掉头
            Eigen::Vector3d adjusted_start_vel = start_vel;
            
            if (start_vel.norm() < 0.1)
            {
                // 速度很小时（接近静止），给一个指向目标的微小初速度
                // 这样优化器生成的轨迹方向会更合理
                double vel_norm = 0.05; // 给一个小值作为方向提示
                double target_direction = atan2((local_target_pt - start_pt)(1), (local_target_pt - start_pt)(0));
                adjusted_start_vel(0) = vel_norm * cos(target_direction);
                adjusted_start_vel(1) = vel_norm * sin(target_direction);
                adjusted_start_vel(2) = 0;
                ROS_DEBUG("Near-zero velocity: setting initial direction toward target");
            }
            // 移除了高速时强制调整速度方向的逻辑
            // 让FSM的ADJUST_POSE状态处理需要掉头的情况

            
            headState << start_pt, adjusted_start_vel, start_acc;
            tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

            /* determined or random inner point */
            if (!flag_randomPolyTraj)
            {
                if (innerPs.cols() != 0)
                {
                    ROS_ERROR("innerPs.cols() != 0");
                }

                piece_nums = 1;
                piece_dur_vec.resize(1);
                piece_dur_vec(0) = init_of_init_totaldur;
            }
            else
            {
                Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
                Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
                innerPs.resize(3, 1);
                innerPs = (start_pt + local_target_pt) / 2 +
                          (((double)rand()) / RAND_MAX - 0.5) *
                              (start_pt - local_target_pt).norm() *
                              horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                          (((double)rand()) / RAND_MAX - 0.5) *
                              (start_pt - local_target_pt).norm() *
                              vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

                piece_nums = 2;
                piece_dur_vec.resize(2);
                piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
            }

            /* generate the init of init trajectory */
            initMJO.reset(headState, tailState, piece_nums);
            initMJO.generate(innerPs, piece_dur_vec);
            poly_traj::Trajectory initTraj = initMJO.getTraj();

            /* generate the real init trajectory */
            piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
            if (piece_nums < 2)
                piece_nums = 2;
            double piece_dur = init_of_init_totaldur / (double)piece_nums;
            piece_dur_vec.resize(piece_nums);
            piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
            innerPs.resize(3, piece_nums - 1);
            int id = 0;
            double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
            double fixed_z = start_pt(2); // 固定 Z 坐标为起点的 Z
            for (double t = t_s; t < t_e; t += piece_dur)
            {
                innerPs.col(id) = initTraj.getPos(t);
                innerPs.col(id)(2) = fixed_z; // 强制 Z 坐标保持一致
                id++;
            }
            if (id != piece_nums - 1)
            {
                ROS_ERROR("Should not happen! x_x");
                return false;
            }
            initMJO.reset(headState, tailState, piece_nums);
            initMJO.generate(innerPs, piece_dur_vec);
        }
        else /*** case 2: initialize from previous optimal trajectory ***/
        {
            if (global_data_.last_glb_t_of_lc_tgt_ < 0.0)
            {
                ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
                return false;
            }

            /* the trajectory time system is a little bit complicated... */
            double passed_t_on_lctraj = ros::Time::now().toSec() - local_data_.start_time_.toSec();
            double t_to_lc_end = local_data_.duration_ - passed_t_on_lctraj;
            if (t_to_lc_end < 0)
            {
                ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
                return false;
            }
            double t_to_lc_tgt = t_to_lc_end +
                                 (global_data_.glb_t_of_lc_tgt_ - global_data_.last_glb_t_of_lc_tgt_);
            int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
            if (piece_nums < 2)
                piece_nums = 2;

            Eigen::Matrix3d headState, tailState;
            Eigen::MatrixXd innerPs(3, piece_nums - 1);
            Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
            headState << start_pt, start_vel, start_acc;
            tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

            double fixed_z = start_pt(2); // 固定 Z 坐标
            double t = piece_dur_vec(0);
            for (int i = 0; i < piece_nums - 1; ++i)
            {
                if (t < t_to_lc_end)
                {
                    innerPs.col(i) = local_data_.minco_traj_.getPos(t + passed_t_on_lctraj);
                }
                else if (t <= t_to_lc_tgt)
                {
                    double glb_t = t - t_to_lc_end + global_data_.last_glb_t_of_lc_tgt_ - global_data_.global_start_time_.toSec();
                    innerPs.col(i) = global_data_.global_traj_.evaluate(glb_t);
                }
                else
                {
                    ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
                }
                
                innerPs.col(i)(2) = fixed_z; // 强制 Z 坐标保持一致

                t += piece_dur_vec(i + 1);
            }

            initMJO.reset(headState, tailState, piece_nums);
            initMJO.generate(innerPs, piece_dur_vec);
        }

        return true;
    }

    bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
    {
        poly_traj::Trajectory traj = opt.getTraj();

        auto timeScaleTraj = [](const poly_traj::Trajectory &input, double scale) {
            std::vector<double> durations;
            std::vector<poly_traj::CoefficientMat> coeffs;
            durations.reserve(input.getPieceNum());
            coeffs.reserve(input.getPieceNum());
            for (int i = 0; i < input.getPieceNum(); ++i)
            {
                poly_traj::CoefficientMat coeff = input[i].getCoeffMat();
                for (int col = 0; col < 5; ++col)
                {
                    const int degree = 5 - col;
                    coeff.col(col) /= std::pow(scale, degree);
                }
                durations.push_back(input[i].getDuration() * scale);
                coeffs.push_back(coeff);
            }
            return poly_traj::Trajectory(durations, coeffs);
        };

        double max_v = 0.0, max_a = 0.0, max_w = 0.0, max_z_err = 0.0;
        const double fixed_z = traj.getPos(0.0)(2);
        const double dt = 0.05;
        for (double t = 0.0; t <= traj.getTotalDuration(); t += dt)
        {
            Eigen::Vector3d pos = traj.getPos(t);
            Eigen::Vector3d vel = traj.getVel(t);
            Eigen::Vector3d acc = traj.getAcc(t);
            max_z_err = std::max(max_z_err, std::abs(pos(2) - fixed_z));
            const double speed = vel.head<2>().norm();
            max_v = std::max(max_v, speed);
            max_a = std::max(max_a, acc.head<2>().norm());
            if (speed > 0.05)
            {
                const double cross = vel(0) * acc(1) - vel(1) * acc(0);
                max_w = std::max(max_w, std::abs(cross) / (speed * speed));
            }
        }
        if (max_z_err > 0.03)
        {
            ROS_WARN("[MINCO_REJECT] z drift %.3fm exceeds 2D plane tolerance", max_z_err);
            return false;
        }

        const double vel_scale = max_v / std::max(0.1, pp_.max_vel_ * 0.98);
        const double acc_scale = std::sqrt(max_a / std::max(0.1, pp_.max_acc_ * 0.98));
        const double w_scale = max_w / std::max(0.1, pp_.max_w_ * 0.98);
        const double scale = std::max(1.0, std::max(vel_scale, std::max(acc_scale, w_scale)));
        if (!std::isfinite(scale) || scale > 4.0)
        {
            ROS_WARN("[MINCO_REJECT] infeasible after scaling: scale=%.2f, max_v=%.2f, max_a=%.2f, max_w=%.2f",
                     scale, max_v, max_a, max_w);
            return false;
        }
        if (scale > 1.02)
        {
            const double old_duration = traj.getTotalDuration();
            traj = timeScaleTraj(traj, scale);
            ROS_WARN("[MINCO_SCALE] time-scale %.2f: max_v %.2f->%.2f, max_a %.2f->%.2f, max_w %.2f->%.2f, dur %.2f->%.2f",
                     scale, max_v, max_v / scale, max_a, max_a / (scale * scale), max_w, max_w / scale,
                     old_duration, traj.getTotalDuration());
        }
        
        // 计算预检测点缓存，用于高效安全碰撞检测
        Eigen::MatrixXd cps = opt.getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
        PtsChk_t pts_to_check;
        
        int i_end = ConstraintPoints::ground_robot_safety_check_id(cps, touch_goal);
        bool ret = minco_optimizer_->computePointsToCheck(traj, i_end, pts_to_check);
        
        if (ret && pts_to_check.size() >= 1 && pts_to_check.back().size() >= 1)
        {
            // 存储 MINCO 轨迹和预计算的检测点
            local_data_.minco_traj_ = traj;
            local_data_.pts_chk_ = pts_to_check;  // 存储预计算的碰撞检测点
            local_data_.use_minco_traj_ = true;
            local_data_.duration_ = traj.getTotalDuration();
            local_data_.traj_id_++;
            local_data_.start_pos_ = traj.getJuncPos(0);
            local_data_.start_time_ = ros::Time::now();  // 设置轨迹开始时间
            
            return true;
        }
        
        ROS_WARN("[PlannerManager] Failed to compute points to check for trajectory");
        return false;
    }

} // namespace ego_planner
