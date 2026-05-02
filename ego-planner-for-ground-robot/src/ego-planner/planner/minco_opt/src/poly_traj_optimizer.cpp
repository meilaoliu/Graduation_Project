#include "minco_opt/poly_traj_optimizer.h"
#include <random>

using namespace std;

#define VERBOSE_OUTPUT false
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      double &final_cost)
  {
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    // Preparision 1: Some mise params
    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_unsafe, flag_success;
    multitopology_data_.initial_obstacles_avoided = false;
    // P1-A 修订: 进入新一轮优化前清空, 避免上一次调用残留覆盖本次诊断
    last_failure_reason_ = "";

    // Preparision 2: Trajectory related params
    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();
    jerkOpt_.reset(iniState, finState, piece_num_);
    variable_num_ = 4 * (piece_num_ - 1) + 1;
    double x_init[variable_num_];
    memcpy(x_init, initInnerPts.data(), initInnerPts.size() * sizeof(x_init[0]));
    Eigen::Map<Eigen::VectorXd> Vt(x_init + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);

    // Preparision 3: LBFGS related params
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.abs_curv_cond = 0;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-2;
    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      flag_force_return = false;
      force_stop_type_ = DONT_STOP;
      flag_still_unsafe = false;
      flag_success = false;

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params);

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      // [RESTART_DIAG] 每轮 LBFGS 退出后的状态: 用于查 max_restarts 真因
      ROS_INFO_THROTTLE(0.5,
                        "[RESTART_DIAG] iter=%d restart=%d rebound=%d lbfgs_ret=%d(%s) force_stop=%d cost=%.3e time_ms=%.1f",
                        iter_num_, restart_nums, rebound_times, result,
                        lbfgs::lbfgs_strerror(result), (int)force_stop_type_,
                        final_cost, time_ms);

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        flag_force_return = false;

        /* double check: fine collision check */
        std::vector<std::pair<int, int>> segments_nouse;
          if (finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false) == CHK_RET::OBS_FREE)
          {

            flag_success = true;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);

            // Log trajectory quality metrics
            {
              poly_traj::Trajectory traj = jerkOpt_.getTraj();
              double max_v = 0, max_a = 0, max_k = 0;
              double dt = 0.05;
              for (double t = 0; t < traj.getTotalDuration(); t += dt)
              {
                Eigen::Vector3d v = traj.getVel(t);
                Eigen::Vector3d a = traj.getAcc(t);
                double vn = v.head<2>().norm();
                double an = a.head<2>().norm();
                max_v = std::max(max_v, vn);
                max_a = std::max(max_a, an);
                if (vn > 0.3)
                {
                  double cross = v(0) * a(1) - v(1) * a(0);
                  double ki = std::abs(cross) / (vn * vn * vn);
                  max_k = std::max(max_k, ki);
                }
              }
              printf("[TRAJ_CAND] max_v=%.2f, max_a=%.2f, max_k=%.2f, duration=%.2f\n",
                max_v, max_a, max_k, traj.getTotalDuration());

              printf("[TRAJ_OK] max_v=%.2f, max_a=%.2f, max_k=%.2f, duration=%.2f\n",
                     max_v, max_a, max_k, traj.getTotalDuration());
            }
          }
          else
          {
            // A not-blank return value means collision to obstales
            flag_still_unsafe = true;
            restart_nums++;
            PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
            // [Fix J: 确定性 restart 扰动] 必须保留: amp=0 会让 3 次 restart 包同一盆地 → 毫无意义
            {
              double amp = (restart_nums == 1) ? 0.005 : (restart_nums == 2 ? 0.02 : 0.05);
              for (int k = 0; k < (int)initInnerPts.size(); ++k)
                x_init[k] += ((k % 2 == 0) ? amp : -amp);
            }
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iter_num_, time_ms);
      }
      else
      {
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iter_num_, time_ms);
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // [Fix J: ROUNDING_ERROR 同样确定性微扰动]
        if (result == lbfgs::LBFGSERR_ROUNDING_ERROR && restart_nums < 3)
        {
          flag_still_unsafe = true;
          restart_nums++;
          double amp = (restart_nums == 1) ? 0.005 : (restart_nums == 2 ? 0.02 : 0.05);
          for (int k = 0; k < (int)initInnerPts.size(); ++k)
            x_init[k] += ((k % 2 == 0) ? amp : -amp);
        }
      }

    } while ((flag_still_unsafe && restart_nums < 3) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    // Update diagnostic tracking variables
    last_lbfgs_result_ = flag_success ? 1 : 0;
    last_restart_count_ = restart_nums;
    last_rebound_count_ = rebound_times;
    last_final_cost_ = final_cost;
    if (!flag_success)
    {
      // P1-A 修订: 优先保留下游已经填好的细分标签
      // (init_collision_dense / astar_init_err / astar_search_err / astar_input_diverged 等),
      // 仅当还未被赋值时, 再退化到通用桶 max_restarts/max_rebounds/force_stop_error/optimization_failed.
      const bool specific_already_set =
          (last_failure_reason_ == "init_collision_dense") ||
          (last_failure_reason_ == "astar_init_err") ||
          (last_failure_reason_ == "astar_search_err") ||
          (last_failure_reason_ == "astar_input_diverged");
      if (!specific_already_set)
      {
        if (restart_nums >= 3)
          last_failure_reason_ = "max_restarts";
        else if (rebound_times > 20)
          last_failure_reason_ = "max_rebounds";
        else if (force_stop_type_ == STOP_FOR_ERROR)
          last_failure_reason_ = "force_stop_error";
        else
          last_failure_reason_ = "optimization_failed";
      }
    }
    else
    {
      last_failure_reason_ = "";
    }

    return flag_success;
  }

  bool PolyTrajOptimizer::computePointsToCheck(
      poly_traj::Trajectory &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    pts_check.clear();
    pts_check.resize(id_cps_end);
    const double RES = grid_map_->getResolution(), RES_2 = RES / 2;
    Eigen::VectorXd durations = traj.getDurations();
    Eigen::VectorXd t_seg_start(durations.size() + 1);
    t_seg_start(0) = 0;
    for (int i = 0; i < durations.size(); ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);
    const double DURATION = durations.sum();
    double t = 0.0, t_step = min(RES / max_vel_, durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5);
    Eigen::Vector3d pt_last = traj.getPos(0.0);
    // pts_check[0].push_back(pt_last);
    int id_cps_curr = 0, id_piece_curr = 0;

    while (true)
    {
      if (t > DURATION)
      {
        if (touch_goal_ && pts_check.size() > 0)
        {
          while (pts_check.back().size() == 0)
          {
            pts_check.pop_back();
          }

          if (pts_check.size() <= 0)
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)pts_check.size());
            return false;
          }
          else
          {
            return true;
          }
        }
        else
        {
          // 当 touch_goal_=false 时，如果已经收集到足够的点，也认为成功
          if (pts_check.size() > 0 && id_cps_curr > 0)
          {
            ROS_WARN_ONCE("Trajectory duration exceeded but collected %d constraint points, continuing...", (int)pts_check.size());
            return true;
          }
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d, pts_check.size()=%d", touch_goal_, (int)pts_check.size());
          pts_check.clear();
          return false;
        }
      }

      const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / cps_num_prePiece_ * ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
      if (t >= next_t_stp)
      {
        if (id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1))
        {
          ++id_piece_curr;
        }
        if (++id_cps_curr >= id_cps_end)
        {
          break;
        }
      }

      Eigen::Vector3d pt = traj.getPos(t);
      if (t < 1e-5 || pts_check[id_cps_curr].size() == 0 || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        pt_last = pt;
      }

      t += t_step;
    }

    return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const poly_traj::MinJerkOpt &pt_data,
      const bool flag_first_init /*= true*/)
  {

    Eigen::MatrixXd init_points = pt_data.getInitConstraintPoints(cps_num_prePiece_);
    poly_traj::Trajectory traj = pt_data.getTraj();

    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    int in_id = -1, out_id = -1;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_); // only check closed 2/3 points.

    // === 修复 A: 地面机器人 5s 轨迹的最后 1/3 (~2m) 不能省略校验,
    // 否则 finelyCheck 直接漏过尾段的障碍 → FSM 当成 OBS_FREE 发布 →
    // checkCollisionCallback 才在 t=2.x 处补救, 但车已开过去撞了。
    // touch_goal 时 two_thirds_id 已 = cols-1; 这里只在非 touch_goal 路径下扩到 95%。
    if (!touch_goal_)
    {
      int extended = init_points.cols() - 2; // 留 1 个尾点缓冲, 其余全检
      if (extended > i_end) i_end = extended;
    }

    PtsChk_t pts_check;
    if (!computePointsToCheck(traj, i_end, pts_check))
    {
      return CHK_RET::ERR;
    }

    for (int i = 0; i < i_end; ++i)
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j)
      {
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second);

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)
          {
            in_id = i;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /* Collision free and return in advance */
    if (segment_ids.size() == 0)
    {
      return CHK_RET::OBS_FREE;
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      // Skip A* if points have diverged (NaN/uninitialized -> spurious huge distance)
      double dist = (in - out).norm();
      if (!in.allFinite() || !out.allFinite() || dist > 50.0)
      {
        ROS_WARN_THROTTLE(1.0,
                          "[finelyCheck] A* skipped: diverged points (dist=%.1f), drop segment %zu/%zu",
                          dist, i + 1, segment_ids.size());
        last_failure_reason_ = "astar_input_diverged";
        // Drop this bad segment instead of aborting the whole optimization step.
        // If all segments are bad, the loop ends with empty a_star_pathes and we
        // fall through to OBS_FREE below, letting the caller proceed with the
        // current init guess (the next iteration's roughlyCheck will re-evaluate).
        segment_ids.erase(segment_ids.begin() + i);
        --i;
        continue;
      }
      // Convert 3D to 2D for ground robot A*
      Eigen::Vector2d in_2d(in.x(), in.y()), out_2d(out.x(), out.y());
      // P2-A + P1-A: 区分 INIT_ERR (端点无效) 与 SEARCH_ERR (扩展失败)
      ASTAR_RET ret = a_star_->AstarSearchTyped(grid_map_->getResolution(), in_2d, out_2d);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else if (ret == ASTAR_RET::INIT_ERR)
      {
        // 端点本身无效 (在障碍/出界): 丢段 + 记录可识别原因, 让上层 reReplan
        ROS_WARN_THROTTLE(1.0,
                          "[finelyCheck] A* INIT_ERR seg %zu/%zu, drop", i + 1, segment_ids.size());
        last_failure_reason_ = "astar_init_err";
        segment_ids.erase(segment_ids.begin() + i);
        --i;
        continue;
      }
      else if (i + 1 < segment_ids.size()) // SEARCH_ERR, connect the next segment
      {
        last_failure_reason_ = "astar_search_err";
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A corner case 2, I have never exeam it.");
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");
        last_failure_reason_ = "astar_search_err";
        return CHK_RET::ERR;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {

    // int end_idx = cps_.cp_size - 1;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_); // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.cp_size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("Local target in collision, skip this planning.");

          // P1-A: 显式标记 "局部目标贴障碍" 失败 (与 max_restarts 区分)
          last_failure_reason_ = "init_collision_dense";
          // P1-B: 记录 force_stop 触发点, 便于事后回溯
          ROS_DEBUG_NAMED("minco_force_stop", "STOP_FOR_ERROR @ roughlyCheck: terminal-in-obstacle");
          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        // Skip A* if points have diverged (gradient explosion)
        double dist = (in - out).norm();
        if (dist > 50.0 || std::isnan(in(0)) || std::isnan(out(0)))
        {
          ROS_WARN("[roughlyCheck] A* skipped: diverged points (dist=%.1f)", dist);
          segment_ids.erase(segment_ids.begin() + i);
          --i;
          continue;
        }
        // Convert 3D to 2D for ground robot A*
        Eigen::Vector2d in_2d(in.x(), in.y()), out_2d(out.x(), out.y());
        // P2-A + P1-A: 区分 INIT_ERR / SEARCH_ERR
        ASTAR_RET ret = a_star_->AstarSearchTyped(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in_2d, out_2d);
        if (ret == ASTAR_RET::SUCCESS)
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else if (ret == ASTAR_RET::INIT_ERR)
        {
          ROS_WARN_THROTTLE(1.0,
                            "[roughlyCheck] A* INIT_ERR seg %zu/%zu, drop", i + 1, segment_ids.size());
          last_failure_reason_ = "astar_init_err";
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
        else if (i + 1 < segment_ids.size()) // SEARCH_ERR, connect the next segment
        {
          last_failure_reason_ = "astar_search_err";
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A corner case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          last_failure_reason_ = "astar_search_err";
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[i].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      // P1-B: 记录 rebound 触发点 (含段数), 便于诊断 rebound 频率
      ROS_DEBUG_NAMED("minco_force_stop",
                      "STOP_FOR_REBOUND @ roughlyCheck: %zu collision segment(s)",
                      segment_ids.size());
      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool PolyTrajOptimizer::allowRebound(void) //zxzxzx
  {
    // criterion 1
    if (iter_num_ < 3)
      return false;

    // criterion 2
    double min_product = 1;
    for (int i = 3; i <= cps_.points.cols() - 4; ++i) // ignore head and tail
    {
      double product = ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized()).dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
      if (product < min_product)
      {
        min_product = product;
      }
    }
    if (min_product < 0.87) // 30 degree
      return false;

    // criterion 3
    if (multitopology_data_.use_multitopology_trajs)
    {
      if (!multitopology_data_.initial_obstacles_avoided)
      {
        bool avoided = true;
        for (int i = 1; i < cps_.points.cols() - 1; ++i)
        {
          if (cps_.base_point[i].size() > 0)
          {
            // Only adopts "0" since finelyCheckAndSetConstraintPoints() after one optimization can add more base_points.
            if ((cps_.points.col(i) - cps_.base_point[i][0]).dot(cps_.direction[i][0]) < 0)
            {
              avoided = false;
              break;
            }
          }
        }

        multitopology_data_.initial_obstacles_avoided = avoided;
      }

      if (!multitopology_data_.initial_obstacles_avoided)
      {
        return false;
      }
    }

    // all the criterion passed
    return true;
  }

  /* multi-topo support */
  std::vector<ConstraintPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ConstraintPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
      ConstraintPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)
        {
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
            for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstraintPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!");
            if (VERBOSE_OUTPUT)
              cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN_COND(VERBOSE_OUTPUT, "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN_COND(VERBOSE_OUTPUT, "base_point and control point are too close!, 2");
          if (VERBOSE_OUTPUT)
            cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    // Step 2. Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ConstraintPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.cp_size)
      {

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    // Eigen::VectorXd T(Eigen::VectorXd::Constant(piece_nums, opt->t2T(x[n - 1]))); // same t
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(4);

    opt->VirtualT2RealT(t, T); // Unbounded virtual time to real time

    opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}
    
    // Force 2D: Clear Z component in coefficient matrix immediately after generation
    opt->jerkOpt_.get_gdC().col(2).setZero();

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAJGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost

    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP); // Gradient prepagation

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost); // Real time back to virtual time

    opt->iter_num_ += 1;
    
    // Force 2D optimization: clear all Z gradients
    gradP.row(2).setZero();

    if (opt->iter_num_ % 10 == 0 || VERBOSE_OUTPUT) {
    printf("[DEBUG] Iter: %d | Total: %.2f | Smooth: %.2f | Obs: %.2f | Feas(V+A+K): %.2f | Dist: %.2f | Time: %.2f\n",
            opt->iter_num_,
            smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost,
            smoo_cost,
            obs_swarm_feas_qvar_costs(0), // Obstacle
            obs_swarm_feas_qvar_costs(2), // Feasibility (V+A+K)
            obs_swarm_feas_qvar_costs(3), // SqrVariance
            time_cost);
  }

    double total_cost = smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost;

    // Detect NaN early and log which cost component caused it
    if (std::isnan(total_cost) || std::isinf(total_cost))
    {
      ROS_ERROR("[COST_NAN] iter=%d | Smooth=%.2e | Obs=%.2e | Feas=%.2e | Dist=%.2e | Time=%.2e | T_min=%.4f | T_max=%.4f",
                opt->iter_num_, smoo_cost,
                obs_swarm_feas_qvar_costs(0), obs_swarm_feas_qvar_costs(2),
                obs_swarm_feas_qvar_costs(3), time_cost,
                T.minCoeff(), T.maxCoeff());
    }

    return total_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    // Fix H2: Vt > V_MAX 区段改用 atan 软饶和, RT 硬上限 ≤ RT_MAX + SLOPE_MAX * V_WIDTH
    // Fix H 原版线性外推 RT = 21*Vt 在 Vt=1e50 时 RT=2e51, 下游 T^4/T^6 仍会 inf
    // atan 形式: Vt=V_MAX 值与一阶导连续, Vt→∞ 时 RT → 221+210 = 431
    constexpr double V_MAX = 20.0;
    constexpr double RT_MAX = (0.5 * V_MAX + 1.0) * V_MAX + 1.0; // = 221.0
    constexpr double SLOPE_MAX = V_MAX + 1.0;                     // = 21.0
    constexpr double V_WIDTH = 10.0;
    constexpr double K_ATAN = M_PI / (2.0 * V_WIDTH);
    for (int i = 0; i < VT.size(); ++i)
    {
      if (VT(i) > V_MAX)
        RT(i) = RT_MAX + (SLOPE_MAX / K_ATAN) * std::atan(K_ATAN * (VT(i) - V_MAX));
      else if (VT(i) > 0.0)
        RT(i) = ((0.5 * VT(i) + 1.0) * VT(i) + 1.0);
      else
        RT(i) = 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    // Fix H2: 与 atan 饶和配套, 导数 d(RT)/d(Vt) = SLOPE_MAX / (1 + (K_ATAN*(Vt-V_MAX))^2)
    constexpr double V_MAX = 20.0;
    constexpr double SLOPE_MAX = V_MAX + 1.0;
    constexpr double V_WIDTH = 10.0;
    constexpr double K_ATAN = M_PI / (2.0 * V_WIDTH);
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > V_MAX)
      {
        double u = K_ATAN * (VT(i) - V_MAX);
        gdVT2Rt = SLOPE_MAX / (1.0 + u * u);
      }
      else if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d gradp, gradv, grada;
    Eigen::Vector3d gradK_v, gradK_a;
    double costp, costv, costa, costK;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaKc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaKt;
    double omg;
    int i_dp = 0;
    costs.setZero();

    double t = 0;
    for (int i = 0; i < N; ++i)
    {
      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // feasibility: velocity constraint
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(2) += omg * step * costv;
        }

        // feasibility: acceleration constraint
        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(2) += omg * step * costa;
        }

        // feasibility: curvature constraint (left-right separation with eps regularization)
        if (feasibilityGradCostK(vel, acc, gradK_v, gradK_a, costK))
        {
          gradViolaKc = beta1 * gradK_v.transpose() + beta2 * gradK_a.transpose();
          gradViolaKt = alpha * (gradK_v.dot(acc) + gradK_a.dot(jer));
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaKc;
          gdT(i) += omg * (costK / K + step * gradViolaKt);
          costs(2) += omg * step * costK;
        }

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    // lengthVarianceWithGradCost2p(cps_.points, K, gdp, var);
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(3) += var;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
      double dist = ray.dot(cps_.direction[i_dp][j]);
      double dist_err = obs_clearance_ - dist;
      double dist_err_soft = obs_clearance_soft_ - dist;
      Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

      if (dist_err > 0)
      {
        ret = true;
        costp += wei_obs_ * pow(dist_err, 3);
        gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
      }

      if (dist_err_soft > 0)
      {
        ret = true;
        double r = 0.05;
        double rsqr = r * r;
        double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
        costp += wei_obs_soft_ * rsqr * (term - 1.0);
        gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
      }
    }

    return ret;
  }

/*
  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
      return false;
  }
*/
  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJ(const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    double jpen = j.squaredNorm() - max_jer_ * max_jer_;
    if (jpen > 0)
    {
      gradj = wei_feas_ * 6 * jpen * jpen * j;
      costj = wei_feas_ * jpen * jpen * jpen;
      return true;
    }
    return false;
  }

  // Curvature constraint using Inequality Transformation (de-denominator method)
  // Constraint: |κ| = |(v×a)/|v|³| ≤ κ_max
  // Transformed: (v×a)² - κ_max² · |v|⁶ ≤ 0  (no division, no singularity at v=0)
  bool PolyTrajOptimizer::feasibilityGradCostK(const Eigen::Vector3d &vel,
                                               const Eigen::Vector3d &acc,
                                               Eigen::Vector3d &gradK_v,
                                               Eigen::Vector3d &gradK_a,
                                               double &costK)
  {
    double v2 = vel.squaredNorm();
    double cross = vel(0) * acc(1) - vel(1) * acc(0); // 2D cross product
    double cross2 = cross * cross;

    double k_lim_sqr = max_curv_ * max_curv_;
    double v4 = v2 * v2;
    double v6 = v4 * v2;

    // P = (v×a)² - κ²·|v|⁶.  P > 0 means constraint violated.
    double penalty = cross2 - k_lim_sqr * v6;

    if (penalty > 0)
    {
      // === Huber 型分段惩罚: 防止 wei_curv 偏大或重 replan 边界处 |a|↑ 时
      //     |grad| 直接爆到 1e8 量级、LBFGS 线搜索撞 ROUNDING_ERROR/COST_NAN ===
      // penalty < THR: 平方惩罚 (灵敏)
      // penalty ≥ THR: 线性惩罚, |dJ/dP| = 2·wei·THR 上界
      const double THR = 0.3;
      double dJ_dP;
      if (penalty < THR)
      {
        costK = wei_curv_ * penalty * penalty;
        dJ_dP = 2.0 * wei_curv_ * penalty;
      }
      else
      {
        costK = wei_curv_ * (THR * THR + 2.0 * THR * (penalty - THR));
        dJ_dP = 2.0 * wei_curv_ * THR;
      }

      Eigen::Vector3d B_vel(-vel(1), vel(0), 0.0);
      Eigen::Vector3d B_acc(-acc(1), acc(0), 0.0);

      // dP/da = 2 · cross · B_vel
      Eigen::Vector3d dP_da = 2.0 * cross * B_vel;
      // dP/dv = -2 · cross · B_acc - 6 · κ² · |v|⁴ · v
      Eigen::Vector3d dP_dv = -2.0 * cross * B_acc - 6.0 * k_lim_sqr * v4 * vel;

      gradK_a = dJ_dP * dP_da;
      gradK_v = dJ_dP * dP_dv;

      static int curv_log_count = 0;
      if (curv_log_count++ % 50 == 0)
      {
        double ki_approx = (v2 > 1e-6) ? sqrt(cross2) / (v2 * sqrt(v2)) : 0.0;
        ROS_WARN("[CURV] ki~=%.3f, max=%.3f, |v|=%.3f, |a|=%.3f, penalty=%.3f, costK=%.3f",
                 ki_approx, max_curv_, sqrt(v2), acc.norm(), penalty, costK);
      }

      return true;
    }
    return false;
  }


  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                       const int n,
                                                       Eigen::MatrixXd &gdp,
                                                       double &var)
  {
    int N = ps.cols() - 1;
    int M = N / n;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd ds = dps.colwise().norm().transpose();
    Eigen::VectorXd ls(M), lsqrs(M);
    for (int i = 0; i < M; i++)
    {
      ls(i) = ds.segment(i * n, n).sum();
      lsqrs(i) = ls(i) * ls(i);
    }
    double lm = ls.mean();
    double lsqrm = lsqrs.mean();
    var = wei_sqrvar_ * (lsqrm - lm * lm) + 250.0 * M * lm;
    Eigen::VectorXd gdls = wei_sqrvar_ * 2.0 / M * (ls.array() - lm) + 250.0;
    Eigen::MatrixXd gdds = dps.colwise().normalized();
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i < M; i++)
    {
      gdp.block(0, i * n, 3, n) -= gdls(i) * gdds.block(0, i * n, 3, n);
      gdp.block(0, i * n + 1, 3, n) += gdls(i) * gdds.block(0, i * n, 3, n);
    }
    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_curv", wei_curv_, wei_feas_);      // 默认fallback到wei_feas_
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);
    if (max_jer_ <= 0.0)
      nh.param("optimization/max_jerk", max_jer_, -1.0);
    nh.param("optimization/max_curv", max_curv_, -1.0);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector2i(500, 500)); // 2D for ground robot
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }

  void PolyTrajOptimizer::setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }

  void PolyTrajOptimizer::setUseMultitopologyTrajs(bool use_multitopology_trajs) { multitopology_data_.use_multitopology_trajs = use_multitopology_trajs; }



} // namespace ego_planner