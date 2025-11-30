// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{

    // SECTION interfaces for setup and query

    EGOPlannerManager::EGOPlannerManager() {}

    EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

    void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
    {
        /* read algorithm parameters */

        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
        nh.param("manager/use_minco", use_minco_, false);
        nh.param("manager/use_multitopology_trajs", pp_.use_multitopology_trajs_, false);

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

    // !SECTION

    // SECTION rebond replanning

    bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                          Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                          Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
    {
        // 如果使用 MINCO，调用 MINCO 重规划
        if (use_minco_)
        {
            return reboundReplanMinco(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, flag_polyInit, flag_randomPolyTraj);
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
                    Eigen::MatrixXd pos(3, 3);
                    pos.col(0) = start_pt;
                    pos.col(1) = random_inserted_pt;
                    pos.col(2) = local_target_pt;
                    Eigen::VectorXd t(2);
                    t(0) = t(1) = time / 2;
                    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
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

        cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

        // success. YoY
        continous_failures_count_ = 0;
        return true;
    }

    bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
    {
        Eigen::MatrixXd control_points(3, 6);
        for (int i = 0; i < 6; i++)
        {
            control_points.col(i) = stop_pos;
        }

        updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

        return true;
    }

    bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);

        for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
        {
            points.push_back(waypoints[wp_i]);
        }

        double total_len = 0;
        total_len += (start_pos - waypoints[0]).norm();
        for (size_t i = 0; i < waypoints.size() - 1; i++)
        {
            total_len += (waypoints[i + 1] - waypoints[i]).norm();
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
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);
        points.push_back(end_pos);

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
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
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
                                               Eigen::Vector3d local_target_pt, Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
    {
        ros::Time t_start = ros::Time::now();
        ros::Duration t_init, t_opt;

        static int count = 0;
        cout << "\033[47;30m\n[" << t_start << "] Replan Minco " << count++ << "\033[0m" << endl;

        /*** STEP 1: INIT ***/
        minco_optimizer_->setIfTouchGoal(global_data_.localTrajReachTarget()); // Approximate check
        double ts = pp_.ctrl_pt_dist / pp_.max_vel_;

        poly_traj::MinJerkOpt initMJO;
        if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                              flag_polyInit, flag_randomPolyTraj, ts, initMJO))
        {
            continous_failures_count_++;
            return false;
        }

        Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
        vector<std::pair<int, int>> segments;
        if (minco_optimizer_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
        {
            continous_failures_count_++;
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

            setLocalTrajFromOpt(best_MJO, global_data_.localTrajReachTarget());
            cstr_pts = best_MJO.getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
            
            // 单拓扑模式下显示最优轨迹（多拓扑模式已经在上面显示过了）
            if (!pp_.use_multitopology_trajs_)
            {
                if (cstr_pts.cols() == 0)
                {
                    ROS_WARN("cstr_pts is empty! cps_num_prePiece=%d", minco_optimizer_->get_cps_num_prePiece_());
                }
                visualization_->displayOptimalList(cstr_pts, 0);
            }

            continous_failures_count_ = 0;
        }
        else
        {
            cstr_pts = minco_optimizer_->getMinJerkOpt().getInitConstraintPoints(minco_optimizer_->get_cps_num_prePiece_());
            visualization_->displayInitPathList(point_set, 0.2, 0);

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
            constexpr double init_of_init_totaldur = 2.0;
            
            // 方案C: 优化起点速度方向，避免大角度差异导致掉头
            // 如果速度很小，不要过度依赖当前速度方向
            Eigen::Vector3d adjusted_start_vel = start_vel;
            
            if (start_vel.norm() < 0.1)
            {
                // 速度很小时，直接指向目标
                double vel_norm = 0.1; 
                double target_direction = atan2((local_target_pt - start_pt)(1), (local_target_pt - start_pt)(0));
                adjusted_start_vel(0) = vel_norm * cos(target_direction);
                adjusted_start_vel(1) = vel_norm * sin(target_direction);
                adjusted_start_vel(2) = 0;
            }
            else
            {
                double current_yaw = atan2(start_vel(1), start_vel(0));
                double target_direction = atan2((local_target_pt - start_pt)(1), (local_target_pt - start_pt)(0));
                double yaw_diff = target_direction - current_yaw;
                
                // 归一化角度差到 [-PI, PI]
                while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
                
                // 如果方向差异太大（>60度），调整起点速度方向
                if (abs(yaw_diff) > M_PI / 3.0)
                {
                    double vel_norm = std::max(start_vel.norm(), 0.1); 
                    adjusted_start_vel(0) = vel_norm * cos(target_direction);
                    adjusted_start_vel(1) = vel_norm * sin(target_direction);
                    adjusted_start_vel(2) = 0;
                    ROS_INFO("Adjusted start velocity direction: yaw_diff=%.1f deg", yaw_diff * 180 / M_PI);
                }
            }
            
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
            piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.ctrl_pt_dist);
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
            int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.ctrl_pt_dist);
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
        
        // 存储 MINCO 轨迹
        local_data_.minco_traj_ = traj;
        local_data_.use_minco_traj_ = true; // 标记使用 MINCO 轨迹
        // 注意：start_time_ 由 FSM 在外部设置，这里不设置
        local_data_.duration_ = traj.getTotalDuration();
        local_data_.traj_id_++;
        local_data_.start_pos_ = traj.getJuncPos(0);
        
        // 为了兼容性和发布，生成高精度 B 样条近似
        // 使用更密集的采样以提高精度
        double dt = pp_.ctrl_pt_dist / pp_.max_vel_ * 0.5; // 使用更小的时间步长
        int sample_num = std::max(10, (int)(local_data_.duration_ / dt));
        dt = local_data_.duration_ / sample_num;
        
        vector<Eigen::Vector3d> point_set;
        for (int i = 0; i <= sample_num; ++i)
        {
            double t = std::min(i * dt, local_data_.duration_);
            point_set.push_back(traj.getPos(t));
        }
        
        vector<Eigen::Vector3d> start_end_derivatives;
        start_end_derivatives.push_back(traj.getVel(0.0));
        start_end_derivatives.push_back(traj.getVel(local_data_.duration_));
        start_end_derivatives.push_back(traj.getAcc(0.0));
        start_end_derivatives.push_back(traj.getAcc(local_data_.duration_));
        
        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivatives, ctrl_pts);
        
        // B 样条仅用于发布，实际评估使用 MINCO
        local_data_.position_traj_ = UniformBspline(ctrl_pts, 3, dt);
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        
        return true;
    }

} // namespace ego_planner