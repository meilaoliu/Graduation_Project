#ifndef MPC_H_
#define MPC_H_

#include <qpOASES.hpp>
#include <Eigen/Eigen>
#include <vector>
#include "ros/ros.h"

class MPC_controller
{
public:
    /**
     * 线性化MPC求解，支持前馈参考输入与解耦权重
     * @param X_k 当前状态 [px, py, phi]
     * @param X_r 参考状态序列
     * @param U_r 参考输入序列 (由微分平坦计算)
     * @param N   预测步数
     * @return    最优控制序列 (2 x N)
     */
    Eigen::MatrixXd MPC_Solve_qp(Eigen::Vector3d X_k,std::vector<Eigen::Vector3d >X_r,std::vector<Eigen::Vector2d >U_r,const int N);
    void MPC_init(ros::NodeHandle &nh);
private:
    double v_max;
    double v_min;
    double w_max;
    double w_min;

    // 解耦权重参数
    double w_pos;    // 位置 (px, py) 权重
    double w_ang;    // 航向角 (phi) 权重
    double rho_N;    // 终端代价放大系数
    double r_v;      // 线速度输入权重
    double r_w;      // 角速度输入权重

    // 前馈开关
    bool use_feedforward;
};

#endif