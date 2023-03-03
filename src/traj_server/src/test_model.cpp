#include <iostream>
#include <lapacke.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <traj_controller/traj_controller.hpp>

using namespace Eigen;
struct ListNode {
    int val;  // 节点上存储的元素
    ListNode *next;  // 指向下一个节点的指针
    ListNode(int x) : val(x), next(NULL) {}  // 节点的构造函数
};
Eigen::Matrix3d matrix_ve(Eigen::Vector3d v)
    {
        Eigen::Matrix3d mat;
        mat << 0, -v(2), v(1), 
               v(2), 0, -v(0), 
               -v(1), v(0), 0;
        return mat;
    }
Eigen::Vector4d q_cheng(const Eigen::Vector4d &p, const Eigen::Vector4d &q){
  Eigen::Vector4d result;
  Eigen::Vector3d p_xyz,q_xyz;
  p_xyz(0) = p(1);p_xyz(1) = p(2);p_xyz(2) = p(3);
  q_xyz(0) = q(1);q_xyz(1) = q(2);q_xyz(2) = q(3);
  Eigen::Vector3d vect_xyz;
  Eigen::Matrix3d p0,q0;
  p0 << p(0),0,0,0,p(0),0,0,0,p(0);
  q0 << q(0),0,0,0,q(0),0,0,0,q(0);
  double result_w ;
  result_w = q(0)*p(0) - q.transpose() * p;
  vect_xyz = p_xyz.cross(q_xyz) + p0*q_xyz + q0*p_xyz;
  result(0) = result_w;
  result(1) = vect_xyz(0);
  result(2) = vect_xyz(1);
  result(3) = vect_xyz(2);
  return result;
}
Eigen::Vector3d log2tangent(const Eigen::Vector4d &p){
  Eigen::Vector3d p_xyz;
  Eigen::Vector3d log_xyz;
  Eigen::Matrix3d k_log;
  p_xyz(0) = p(1);p_xyz(1) = p(2);p_xyz(2) = p(3);
  k_log << 2.0,0,0,0,2.0,0,0,0,2.0;
  double p_w ;
  p_w = p(0);
  if(p_w > 0){
    log_xyz = k_log*p_xyz;
  }else if(p_w < 0){
    log_xyz = - k_log*p_xyz;
  }else{
    log_xyz << 0,0,0;
  }
  return log_xyz;
}
Eigen::Matrix3d rotmat_cur;
Eigen::Vector3d Angluar_v_cur ,mavVel_;
Eigen::Matrix<double, 9, 9> a_calculate(){
      Eigen::Matrix<double, 9, 9> result_a;
     Eigen::Matrix3d p_v,p_r,v_v,v_r,r_r;
      Eigen::Vector3d vel_base;
      Eigen::Vector3d e3_g;
      e3_g << 0 ,0 ,9.8;
      vel_base = rotmat_cur.transpose()*mavVel_;
      p_v = rotmat_cur;
      p_r = -rotmat_cur * matrix_ve(vel_base);
      v_v = - matrix_ve(Angluar_v_cur);
      v_r = matrix_ve(rotmat_cur.transpose()*e3_g);
      r_r = - matrix_ve(Angluar_v_cur);
      result_a.setZero();
      result_a.block<3, 3>(0, 3) = p_v;
      result_a.block<3, 3>(0, 6) = p_r;
      result_a.block<3, 3>(3, 3) = v_v;
      result_a.block<3, 3>(3, 6) = v_r;
      result_a.block<3, 3>(6, 6) = r_r;
      return result_a;
}

Eigen::Matrix<double, 9, 4> b_calculate(){
  Eigen::Matrix<double, 9, 4> result_b;
  Eigen::Vector3d v_s;
  Eigen::Matrix3d v_omega,r_omega;
  v_omega = matrix_ve(rotmat_cur.transpose()*mavVel_);
  r_omega << 1,0,0,0,1,0,0,0,1;
  v_s << 0, 0, -13.94; // 9.8/0.703
  result_b.setZero();
  result_b.block<3, 1>(3, 0) = v_s;
   result_b.block<3, 3>(3, 1) = v_omega;
   result_b.block<3, 3>(6, 1) = r_omega;
  return result_b;
}
Eigen::Vector3d mavPos_(1,24,3);
Eigen::Vector4d q_des(1,0,0,0.1);
Eigen::Matrix<double, 9, 9> error_state(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                                const Eigen::Vector3d &target_acc){
      Eigen::Matrix<double, 9, 9> state_err;
      Eigen::Vector3d pos_err, vel_err,q_err;
      Eigen::Vector4d q_ch,q_des_inv;
      pos_err = mavPos_ - target_pos;
      vel_err = mavVel_ - target_vel;
      //q_des = acc2quaternion(target_acc, mavYaw_);
      q_des_inv(0) = q_des(0);q_des_inv(1) = -q_des(1);q_des_inv(2) = -q_des(2);q_des_inv(3) = -q_des(3);
      q_ch = q_cheng(q_des_inv,q_des);
      q_err = log2tangent(q_ch);
      state_err(0) = pos_err(0);state_err(1) = pos_err(1);state_err(2) = pos_err(2);
      state_err(3) = vel_err(0);state_err(4) = vel_err(1);state_err(5) = vel_err(2);
      state_err(6) = q_err(0);  state_err(7) = q_err(1);  state_err(8) = q_err(2);
      return state_err;
}

Eigen::MatrixXd solveRiccatiArimotoPotter(const Eigen::MatrixXd& A,
                               const Eigen::MatrixXd& B,
                               const Eigen::MatrixXd& Q,
                               const Eigen::MatrixXd& R)
{
  MatrixXd P;
  const uint dim_x = A.rows();
  const uint dim_u = B.cols();
  // 哈密顿矩阵
  MatrixXd Ham = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
  Ham << A, -B* R.inverse() * B.transpose(), -Q, -A.transpose();
  // 计算特征值和特征向量
  EigenSolver<MatrixXd> Eigs(Ham);
  // 检查特征值
  //std::cout << "eigen values：\n" << Eigs.eigenvalues() << std::endl;
  //std::cout << "eigen vectors：\n" << Eigs.eigenvectors() << std::endl;
  // extract stable eigenvectors into 'eigvec'
  MatrixXcd eigvec = MatrixXcd::Zero(2 * dim_x, dim_x);
  int j = 0;
  for(int i = 0; i < 2 * dim_x; ++i) {
    if(Eigs.eigenvalues()[i].real() < 0.) {
      eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
      ++j;
    }
  }
  //std::cout << "eigvec==>\n" << eigvec  << std::endl;
  // calc P with stable eigen vector matrix
  MatrixXcd Vs_1, Vs_2;
  Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
  Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
  P = (Vs_2 * Vs_1.inverse()).real();
  return P ;
}

MatrixXd  solveRiccatiIterationD(const MatrixXd& Ad,
                            const MatrixXd& Bd, const MatrixXd& Q,
                            const MatrixXd& R,
                            const double& tolerance = 1.E-8,
                            const uint iter_max = 100000)
{
  MatrixXd P = Q; // initialize

  MatrixXd P_next;

  MatrixXd AdT = Ad.transpose();
  MatrixXd BdT = Bd.transpose();
  MatrixXd Rinv = R.inverse();

  double diff;
  for(uint i = 0; i < iter_max; ++i) {
    // -- discrete solver --
    P_next = AdT * P * Ad -
             AdT * P * Bd * (R + BdT * P * Bd).inverse() * BdT * P * Ad + Q;

    diff = fabs((P_next - P).maxCoeff());
    P = P_next;
    if(diff < tolerance) {
      std::cout << "iteration mumber = " << i << std::endl;
      return P;
    }
  }
  return P; // over iteration limit
}

lapack_int select_lhp(const double *real, const double *imag)
  {
    return *real < 0.0;
  }
MatrixXd solve(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R)
  {
    // Construct H

    MatrixXd H_;
    
    lapack_int sdim = 0;                 // Number of eigenvalues for which sort is true
    lapack_int info;
    const lapack_int dim_x = A.rows();
    Eigen::VectorXd WR(2*dim_x);
    Eigen::VectorXd WI(2*dim_x);
    MatrixXd Ham = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    MatrixXd schur_matrix = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    MatrixXd U11 = MatrixXd::Zero(dim_x, dim_x);
    MatrixXd U21 = MatrixXd::Zero(dim_x, dim_x);
    Ham << A, -B* R.inverse() * B.transpose(), -Q, -A.transpose();
    
    info = LAPACKE_dgees(LAPACK_COL_MAJOR,    // Eigen default storage order
                       'V',                 // Schur vectors are computed
                       'S', select_lhp, Ham.rows(),  Ham.data(),Ham.rows(),&sdim,
                       WR.data(), WI.data(), schur_matrix.data(), Ham.rows());
    // Schur decomposition of H
    
      U11 = schur_matrix.block(0, 0, dim_x, dim_x).transpose();
      U21 = schur_matrix.block(dim_x, 0, dim_x, dim_x).transpose();
    // Find P that solves CARE
      return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();
    
  }

int main()
{
//   bool adf;
//   Eigen::Matrix<double, 4, 4> a_,q_;
//   Eigen::Matrix<double, 4, 1> b_;
//   Eigen::Matrix<double, 1, 1> R;
//   a_ << 0 ,1,0,0,0,0,-1,0,0,0,0,1,0,0,9,0;
//   b_ << 0,0.1,0,-0.1;
//   q_ << 1,0,0,0,0,1,0,0,0,0,10,0,0,0,0,10;
//   R << 0.1;
//   Eigen::Matrix<double, 9, 9> a_cur, Q_;
//   Eigen::MatrixXd result_P,result_pp;
//   Eigen::Matrix<double, 9, 4> b_cur;
  
//     Eigen::Matrix<double, 4, 9> K_;
//   Eigen::Matrix<double, 4, 4> R_W;
//   rotmat_cur << 1,0,0,0,1,0,0,0,1;
//   Angluar_v_cur << 0.1,0.1,0.1;
//   mavVel_ << 0.05, 0.02, 0.1;
//   double max_pos_err_ = 0.2;
//   double max_vel_err_ = 0.1;
//   double max_ang_err_ = 0.2;
//   double max_throttle_err_ = 0.08;
//   double max_omega_err_ = 0.05;
     
     MatrixXd A = MatrixXd::Zero(2 , 2 );
MatrixXd P = MatrixXd::Zero(2 , 2 );
MatrixXd B = MatrixXd::Zero(2 , 1 );
MatrixXd Q = MatrixXd::Zero(2 , 2 );
MatrixXd R = MatrixXd::Zero(1 , 1 );
     A << 0,0, 1,0;
     B << 1,0;
     Q << 0,0,0,1;
     R<< 0.25;
     std::cout << "-v + w - v =\n" <<std::endl;
     P = solve(A,B,Q,R);
     std::cout << "-v + w - v =\n" <<  P  <<std::endl;
//   // Q_.setZero();
//   // R_W.setZero();
//   //adf = solveRiccatiIterationC(a_,b_,q_,R,result_P);
//   // Q_.block<3, 3>(0, 0) =
//   //     (1. / max_pos_err_) * Eigen::Matrix3d::Identity();
//   // Q_.block<3, 3>(3, 3) =
//   //     (1. / max_vel_err_) * Eigen::Matrix3d::Identity();
//   // Q_.block<3, 3>(6, 6) =
//   //     (1. / max_ang_err_) * Eigen::Matrix3d::Identity();
//   // R_W(0, 0) = (1. / max_throttle_err_);
//   // R_W.block<3, 3>(1, 1) =
//   //     (1. / max_omega_err_) * Eigen::Matrix3d::Identity();
// a_cur = a_calculate();
// b_cur = b_calculate();
// // std::cout << "a=\n" <<  a_cur  <<std::endl;
// // std::cout << "b=\n" <<  b_cur  <<std::endl;
// // std::cout << "q=\n" <<  Q_  <<std::endl;
// // std::cout << "r=\n" <<  R_W  <<std::endl;
//  result_pp = solveRiccatiArimotoPotter(a_,b_,q_,R);
//  //result_P = solveRiccatiIterationD(a_,b_,q_,R);
//  //jk = b_calculate();
//  std::cout << "-v + w - v =\n" <<  mavVel_  <<std::endl;

   return 0;
}

