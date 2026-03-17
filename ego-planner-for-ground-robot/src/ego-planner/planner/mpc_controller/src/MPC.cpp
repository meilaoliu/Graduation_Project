#include "MPC.hpp"
#include "math.h"
#include "iostream"

#define PI 3.1415926
#define T 0.03

using namespace Eigen;
using namespace std;
USING_NAMESPACE_QPOASES

MatrixXd MPC_controller::MPC_Solve_qp(Eigen::Vector3d X_k,std::vector<Eigen::Vector3d >X_r,std::vector<Eigen::Vector2d >U_r,const int N)
{
    vector<MatrixXd> A_r(N),B_r(N),A_multiply1(N);
    MatrixXd O_r(3*N,1);
    MatrixXd A_bar(3*N,3);
    MatrixXd X_ref(3*N,1);
    MatrixXd A_multiply2;
    MatrixXd B_bar = MatrixXd::Zero(3*N,2*N);
    MatrixXd C_bar = MatrixXd::Identity(3*N,3*N);
    MatrixXd A_r_init = MatrixXd::Zero(3,3);
    MatrixXd B_r_init = MatrixXd::Zero(3,2);
    MatrixXd eye_3 = MatrixXd::Identity(3,3);

    // 解耦 Q 矩阵: diag(w_pos, w_pos, w_ang) + 终端代价放大 rho_N
    MatrixXd Q = MatrixXd::Zero(3*N,3*N);
    for(int k = 0; k < N; k++)
    {
        double scale = (k == N - 1) ? rho_N : 1.0;
        Q(3*k,   3*k)   = w_pos * scale;
        Q(3*k+1, 3*k+1) = w_pos * scale;
        Q(3*k+2, 3*k+2) = w_ang * scale;
    }

    // 解耦 R 矩阵: diag(r_v, r_w)
    MatrixXd R = MatrixXd::Zero(2*N,2*N);
    for(int k = 0; k < N; k++)
    {
        R(2*k,   2*k)   = r_v;
        R(2*k+1, 2*k+1) = r_w;
    }

    for(int k=0;k<N;k++)
    {
        A_r[k] = A_r_init;
        B_r[k] = B_r_init;
        A_r[k](0,2) = -U_r[k](0)*sin(X_r[k](2));
        A_r[k](1,2) = U_r[k](0)*cos(X_r[k](2));
        Vector3d temp_vec = -T*A_r[k]*X_r[k];
        O_r.block<3,1>(k*3,0) = temp_vec;
        A_r[k] = eye_3+T*A_r[k];
        B_r[k](0,0) = cos(X_r[k](2))*T;
        B_r[k](1,0) = sin(X_r[k](2))*T;
        B_r[k](2,1) = T;
        X_ref.block<3,1>(k*3,0) = X_r[k];

        if(k==0) A_multiply1[k] = A_r[k];
        else A_multiply1[k] = A_multiply1[k-1]*A_r[k];
        A_bar.block<3,3>(3*k,0) = A_multiply1[k];
    }

    for(int k=0;k<N;k++)
    {
        B_bar.block<3,2>(3*k,2*k) = B_r[k];
        A_multiply2 = eye_3;
        for(int i=0;i<k;i++)
        {
            A_multiply2 = A_multiply2*A_r[k-i];
            C_bar.block<3,3>(3*k,3*(k-1-i)) = A_multiply2;
            B_bar.block<3,2>(3*k,2*(k-1-i)) = A_multiply2*B_r[k-1-i];
        }
    }

    MatrixXd E = A_bar*X_k + C_bar*O_r - X_ref;
    MatrixXd Hesse = 2*(B_bar.transpose()*Q*B_bar + R);
    VectorXd gradient = 2*B_bar.transpose()*Q*E;

    // 前馈参考输入: q 向量增加 -2*R*U_ref 偏移项
    // 使代价函数从 ||U||^2_R 变为 ||U - U_ref||^2_R
    if(use_feedforward)
    {
        VectorXd U_ref_vec(2*N);
        for(int k = 0; k < N; k++)
        {
            U_ref_vec(2*k)   = U_r[k](0);
            U_ref_vec(2*k+1) = U_r[k](1);
        }
        gradient -= 2*R*U_ref_vec;
    }

    real_t H[2*N*2*N],g[2*N],A[2*N],lb[2*N],ub[2*N],lbA[1],ubA[1];
    lbA[0] = N*(v_min+w_min);
    ubA[0] = N*(v_max+w_max);
    for(int i=0;i<2*N;i++)
    {
        g[i] = gradient(i);
        A[i] = 1;
        if(i%2==0)
        {
            lb[i] = v_min;
            ub[i] = v_max;
        }
        else
        {
            lb[i] = w_min;
            ub[i] = w_max;
        }
        for(int j=0;j<2*N;j++)
        {
            H[i*2*N+j] = Hesse(i,j);
        }
    }

    int_t nWSR = 800;

    QProblem mpc_qp_solver(2*N,1);
    mpc_qp_solver.init(H,g,A,lb,ub,lbA,ubA,nWSR);

    real_t x_solution[2*N];
    mpc_qp_solver.getPrimalSolution(x_solution);

    Vector2d u_k;
    MatrixXd U_result = MatrixXd::Zero(2,N);
    for(int i=0;i<N;i++)
    {
        u_k(0) = x_solution[2*i];
        u_k(1) = x_solution[2*i+1];
        U_result.col(i) = u_k;
    }
    return U_result;
}

void MPC_controller::MPC_init(ros::NodeHandle &nh)
{
    nh.getParam("/ego_planner_node/MPC/v_max", v_max);
    nh.getParam("/ego_planner_node/MPC/w_max", w_max);

    nh.param("/ego_planner_node/MPC/w_pos", w_pos, 1.0);
    nh.param("/ego_planner_node/MPC/w_ang", w_ang, 5.0);
    nh.param("/ego_planner_node/MPC/rho_N", rho_N, 10.0);
    nh.param("/ego_planner_node/MPC/r_v", r_v, 0.1);
    nh.param("/ego_planner_node/MPC/r_w", r_w, 0.1);
    nh.param("/ego_planner_node/MPC/use_feedforward", use_feedforward, true);

    ROS_INFO("[MPC] v_max=%.2f, w_max=%.2f, w_pos=%.2f, w_ang=%.2f, rho_N=%.1f, r_v=%.2f, r_w=%.2f, feedforward=%s",
             v_max, w_max, w_pos, w_ang, rho_N, r_v, r_w, use_feedforward ? "ON" : "OFF");
    w_min = -w_max;
    v_min = 0;
}
