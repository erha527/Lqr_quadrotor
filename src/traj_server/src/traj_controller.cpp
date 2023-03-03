#include <traj_controller/traj_controller.hpp>

void LQR_CONTROL::init_param(ros::NodeHandle &nh_)
{
    nh_.param("sim_enable", sim_enable_, false);
    imu_sub_ = nh_.subscribe("/mavros/imu/data", 10, &LQR_CONTROL::imuCallback, this);
    position_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &LQR_CONTROL::mavposeCallback, this);
    velocity_sub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &LQR_CONTROL::mavtwistCallback, this);
    mavstate_sub_ = nh_.subscribe("/mavros/state", 1, &LQR_CONTROL::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
    angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1); /////main
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    nh_.param<double>("max_pos_err_", max_pos_err_, 0.01);
    nh_.param<double>("max_vel_err_", max_vel_err_, 0.1);
    nh_.param<double>("max_ang_err_", max_ang_err_, 0.1);
    nh_.param<double>("max_omega_err_", max_omega_err_, 0.1);
    nh_.param<double>("max_throttle_err_", max_throttle_err_, 10);
    drone_mass = 1.6;
    gravity_ = 9.81;
    // J_ << 0.029125,0,0, 0,0.029125,0, 0,0,0.055225;
    J_ << 0.0347563, 0, 0, 0, 0.0458929, 0, 0, 0, 0.0977;
    e3_ << 0, 0, 1;
    sim_enable_ = false;
    // mav
    drone_p = Eigen::Vector3d::Zero();
    drone_v = Eigen::Vector3d::Zero();
    drone_Omega = Eigen::Vector3d::Zero();
    drone_q_b2i = Eigen::Vector4d::Zero();
    drone_q_i2b = Eigen::Vector4d::Zero();
    drone_R = Eigen::Matrix3d::Identity();
    R_d_last = Eigen::Matrix3d::Identity();
    // parameter
    
}
void LQR_CONTROL::publish_cmd(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                                const Eigen::Vector3d &target_acc){
	double time = ros::Time::now().toSec();
        Eigen::Matrix<double, 9, 1>  err_state = error_state(target_pos,target_vel,target_acc);
        Eigen::Matrix<double, 9, 9>  A = a_calculate();
        Eigen::Matrix<double, 9, 4>  B = b_calculate();
        Eigen::Vector4d U_d ;
        U_d << 1.6*9.8, 0, 0, 0;
        //std::cout << "1st "<< std::endl;
	Eigen::Matrix<double, 9, 9> Q;
	Eigen::Matrix<double, 4, 4> R;
	std::cout << "max_pos_err_ "<< max_pos_err_<< std::endl;
 	Q.block<3, 3>(0, 0) = (1. / max_pos_err_) * Eigen::Matrix3d::Identity();
 	Q.block<3, 3>(3, 3) = (1. / max_vel_err_) * Eigen::Matrix3d::Identity();
 	Q.block<3, 3>(6, 6) = (1. / max_ang_err_) * Eigen::Matrix3d::Identity();
 	R(0, 0) = (1. / max_throttle_err_);
	R.block<3, 3>(1, 1) = (1. / max_omega_err_) * Eigen::Matrix3d::Identity();
	//std::cout << "2nd "<< std::endl;
        MatrixXd  P = solve(A,B,Q,R);
	//std::cout << "p: " << P << std::endl;
        MatrixXd  K = R.inverse() * B.transpose()*P;
        Vector4d  U =  K*err_state+U_d;
	//std::cout << "K "<< K << std::endl;
	std::cout << "err_state "<< err_state << std::endl;
        pubRateCommands(U,drone_q_b2i);
	double time_ = ros::Time::now().toSec();
        std::cout << "U "<< U << std::endl;
	
        
                                                }
// update the imu data to calculate the rotation matrix
void LQR_CONTROL::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu imu_data_ = *msg;
    drone_q_b2i(0) = imu_data_.orientation.w;   //from body to inertial
    drone_q_b2i(1) = imu_data_.orientation.x;
    drone_q_b2i(2) = imu_data_.orientation.y;
    drone_q_b2i(3) = imu_data_.orientation.z;

    drone_R = quat2RotMatrix(drone_q_b2i);    //from body to inertial
    drone_q_i2b = rot2Quaternion(drone_R.inverse());  //from inertial to body
    drone_Omega(0) = imu_data_.angular_velocity.x;   ///body-fixed frame
    drone_Omega(1) = imu_data_.angular_velocity.y;
    drone_Omega(2) = imu_data_.angular_velocity.z;
}
// get local position.
void LQR_CONTROL::mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped pos_data = *msg;
    drone_p << pos_data.pose.position.x, pos_data.pose.position.y, pos_data.pose.position.z;//position in inertial
}

// get Velocity in the base_link frame.
void LQR_CONTROL::mavtwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    geometry_msgs::TwistStamped odom = *msg;   //vel in inertial

    drone_v(0) = odom.twist.linear.x;
    drone_v(1) = odom.twist.linear.y;
    drone_v(2) = odom.twist.linear.z;
    drone_v = drone_R.inverse() * drone_v;    // vel in body
}
// publish bodyrate.
void LQR_CONTROL::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude)
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = std::max(-2.0, std::min(2.0, cmd(1)));
    msg.body_rate.y = std::max(-2.0, std::min(2.0, cmd(2)));
    msg.body_rate.z = std::max(-2.0, std::min(2.0, cmd(3)));
    //msg.body_rate.x =0.01;msg.body_rate.y = 0.01;msg.body_rate.z = 0.01;
    msg.type_mask = 128; // Ignore orientation messages
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = std::max(0.0, std::min(0.95, 0.05*(cmd(0)-1.6*9.8)+0.567));//
    //msg.thrust = 0.9;
    double thru = msg.thrust;
    std::cout << "thru "<< thru << std::endl;
    angularVelPub_.publish(msg);
}
Eigen::Matrix<double, 9, 9> LQR_CONTROL::a_calculate()
{
    Eigen::Matrix<double, 9, 9> result_a;
    Eigen::Matrix3d p_v, p_r, v_v, v_r, r_r;
    //Eigen::Vector3d vel_base;
    Eigen::Vector3d e3_g;
    e3_g << 0, 0, 9.8;
    //vel_base = drone_R.transpose() * drone_v;
    p_v = drone_R;
    p_r = -drone_R * matrix_vex(drone_v);
    v_v = -matrix_vex(drone_Omega);
    v_r = matrix_vex(drone_R.transpose() * e3_g);
    r_r = -matrix_vex(drone_Omega);
    result_a.setZero();
    result_a.block<3, 3>(0, 3) = p_v;
    result_a.block<3, 3>(0, 6) = p_r;
    result_a.block<3, 3>(3, 3) = v_v;
    result_a.block<3, 3>(3, 6) = v_r;
    result_a.block<3, 3>(6, 6) = r_r;
    return result_a;
}

Eigen::Matrix<double, 9, 4> LQR_CONTROL::b_calculate()
{
    Eigen::Matrix<double, 9, 4> result_b;
    Eigen::Vector3d v_s;
    Eigen::Matrix3d v_omega, r_omega;
    //v_omega = matrix_vex(drone_R.transpose() * drone_v);
    v_omega = matrix_vex(drone_v);
    r_omega << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    v_s << 0, 0, -0.625; // 1/1.6
    result_b.setZero();
    result_b.block<3, 1>(3, 0) = v_s;
    result_b.block<3, 3>(3, 1) = v_omega;
    result_b.block<3, 3>(6, 1) = r_omega;
    return result_b;
}

/// state error(17)(18)(21)
Eigen::Matrix<double, 9, 1> LQR_CONTROL::error_state(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                                     const Eigen::Vector3d &target_acc)
{
    Eigen::Matrix<double, 9, 1> state_err;
    Eigen::Vector3d pos_err, vel_err, q_err;
    Eigen::Vector4d q_ch, q_des_inv,q_des;
    Eigen::Matrix3d R_des;
    Eigen::Vector3d e3_g;
    e3_g << 0, 0, 9.8;
    q_des = acc2quaternion(target_acc + e3_g, 0.0);
    R_des = quat2RotMatrix(q_des);    //from body to inertial
    q_des = rot2Quaternion(R_des.inverse());  //from inertial to body
    pos_err = drone_p - target_pos;
    vel_err = drone_v - R_des.inverse() * target_vel;

    q_des_inv(0) = q_des(0);
    q_des_inv(1) = -q_des(1);
    q_des_inv(2) = -q_des(2);
    q_des_inv(3) = -q_des(3);
std::cout << "drone_q_i2b "<< drone_q_i2b << std::endl;
std::cout << "q_des_inv "<< q_des_inv << std::endl;

    q_ch = q_cheng(q_des_inv, drone_q_i2b);std::cout << "q_ch "<< q_ch << std::endl;
    q_err = log2tangent(q_ch);

    state_err(0) = pos_err(0);
    state_err(1) = pos_err(1);
    state_err(2) = pos_err(2) ;
    state_err(3) = vel_err(0);
    state_err(4) = vel_err(1);
    state_err(5) = vel_err(2);
    state_err(6) = q_err(0);
    state_err(7) = q_err(1);
    state_err(8) = q_err(2);
    return state_err;
}

lapack_int select_lhp(const double *real, const double *imag)
{
    return *real < 0.0;
}
/// @brief ///////
/// @param A 
/// @param B 
/// @param Q 
/// @param R 
/// @return  P  (31)
MatrixXd LQR_CONTROL::solve(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R)
{
    // Construct H

    MatrixXd H_;

    lapack_int sdim = 0; // Number of eigenvalues for which sort is true
    lapack_int info;
    const lapack_int dim_x = A.rows();
    Eigen::VectorXd WR(2 * dim_x);
    Eigen::VectorXd WI(2 * dim_x);
    MatrixXd Ham = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    MatrixXd schur_matrix = MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    MatrixXd U11 = MatrixXd::Zero(dim_x, dim_x);
    MatrixXd U21 = MatrixXd::Zero(dim_x, dim_x);
    Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    info = LAPACKE_dgees(LAPACK_COL_MAJOR, // Eigen default storage order
                         'V',              // Schur vectors are computed
                         'S', select_lhp, Ham.rows(), Ham.data(), Ham.rows(), &sdim,
                         WR.data(), WI.data(), schur_matrix.data(), Ham.rows());
    // Schur decomposition of H

    U11 = schur_matrix.block(0, 0, dim_x, dim_x).transpose();
    U21 = schur_matrix.block(dim_x, 0, dim_x, dim_x).transpose();
    // Find P that solves CARE
    return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();
}

void LQR_CONTROL::pubSystemStatus()
{
    mavros_msgs::CompanionProcessStatus msg;

    msg.header.stamp = ros::Time::now();
    msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
    msg.state = (int)companion_state_;

    systemstatusPub_.publish(msg);
}

void LQR_CONTROL::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void LQR_CONTROL::set_px4_mode_func(string mode)
{
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = mode;
    if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent)
    {
        // ROS_INFO("set mode success!");
    }
    pubSystemStatus();
}

void LQR_CONTROL::arm_disarm_func(bool on_or_off)
{
    mavros_msgs::CommandBool arm_cmd;

    if (current_state_.armed)
    {
        if (!on_or_off)
        {
            arm_cmd.request.value = on_or_off;
            arming_client_.call(arm_cmd);
            if (arm_cmd.response.success)
            {
                ROS_INFO("vehicle disarming, success!");
            }
            else
            {
                ROS_INFO("vehicle disarming, fail!");
            }
        }
        else
        {
             ROS_INFO("vehicle already armed");
        }
    }
    else if (on_or_off)
    {
        arm_cmd.request.value = on_or_off;
        arming_client_.call(arm_cmd);
        if (arm_cmd.response.success)
        {
             ROS_INFO("vehicle arming, success!");
        }
        else
        {
             ROS_INFO("vehicle arming, fail!");
        }
    }
    else
    {
         ROS_INFO("vehicle already disarmed");
    }
}

Eigen::Vector4d LQR_CONTROL::rot2Quaternion(const Eigen::Matrix3d &Rot)
{
    Eigen::Vector4d uav_quat;
    double tr = Rot.trace();
    if (tr > 0.0)
    {
        double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
        uav_quat(0) = 0.25 * S;
        uav_quat(1) = (Rot(2, 1) - Rot(1, 2)) / S;
        uav_quat(2) = (Rot(0, 2) - Rot(2, 0)) / S;
        uav_quat(3) = (Rot(1, 0) - Rot(0, 1)) / S;
    }
    else if ((Rot(0, 0) > Rot(1, 1)) & (Rot(0, 0) > Rot(2, 2)))
    {
        double S = sqrt(1.0 + Rot(0, 0) - Rot(1, 1) - Rot(2, 2)) * 2.0; // S=4*qx
        uav_quat(0) = (Rot(2, 1) - Rot(1, 2)) / S;
        uav_quat(1) = 0.25 * S;
        uav_quat(2) = (Rot(0, 1) + Rot(1, 0)) / S;
        uav_quat(3) = (Rot(0, 2) + Rot(2, 0)) / S;
    }
    else if (Rot(1, 1) > Rot(2, 2))
    {
        double S = sqrt(1.0 + Rot(1, 1) - Rot(0, 0) - Rot(2, 2)) * 2.0; // S=4*qy
        uav_quat(0) = (Rot(0, 2) - Rot(2, 0)) / S;
        uav_quat(1) = (Rot(0, 1) + Rot(1, 0)) / S;
        uav_quat(2) = 0.25 * S;
        uav_quat(3) = (Rot(1, 2) + Rot(2, 1)) / S;
    }
    else
    {
        double S = sqrt(1.0 + Rot(2, 2) - Rot(0, 0) - Rot(1, 1)) * 2.0; // S=4*qz
        uav_quat(0) = (Rot(1, 0) - Rot(0, 1)) / S;
        uav_quat(1) = (Rot(0, 2) + Rot(2, 0)) / S;
        uav_quat(2) = (Rot(1, 2) + Rot(2, 1)) / S;
        uav_quat(3) = 0.25 * S;
    }
    return uav_quat;
}
////////formula (7)  /
Eigen::Vector4d LQR_CONTROL::q_cheng(const Eigen::Vector4d &p, const Eigen::Vector4d &q)
{
    Eigen::Vector4d result;
    Eigen::Vector3d p_xyz, q_xyz;
    p_xyz(0) = p(1);
    p_xyz(1) = p(2);
    p_xyz(2) = p(3);
    q_xyz(0) = q(1);
    q_xyz(1) = q(2);
    q_xyz(2) = q(3);
    Eigen::Vector3d vect_xyz;
    Eigen::Matrix3d p0, q0;
    p0 << p(0), 0, 0, 0, p(0), 0, 0, 0, p(0);
    q0 << q(0), 0, 0, 0, q(0), 0, 0, 0, q(0);
    double result_w;
    result_w = q(0) * p(0) - q_xyz.transpose() * p_xyz;
    vect_xyz = p_xyz.cross(q_xyz) + p0 * q_xyz + q0 * p_xyz;
    result(0) = result_w;
    result(1) = vect_xyz(0);
    result(2) = vect_xyz(1);
    result(3) = vect_xyz(2);
    return result;
}
////////formula (12) /////
Eigen::Vector3d LQR_CONTROL::log2tangent(const Eigen::Vector4d &p)
{
    Eigen::Vector3d p_xyz;
    Eigen::Vector3d log_xyz;
    Eigen::Matrix3d k_log;
    p_xyz(0) = p(1);
    p_xyz(1) = p(2);
    p_xyz(2) = p(3);
    k_log << 2.0, 0, 0, 0, 2.0, 0, 0, 0, 2.0;
    double p_w;
    p_w = p(0);
    if (p_w > 0)
    {
        log_xyz = k_log * p_xyz;
    }
    else if (p_w < 0)
    {
        log_xyz = -k_log * p_xyz;
    }
    else
    {
        log_xyz << 0, 0, 0;
    }
    return log_xyz;
}
/////////


Eigen::Matrix3d LQR_CONTROL::quat2RotMatrix(const Eigen::Vector4d &q)
{
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

    return rotmat;
}

Eigen::Vector3d LQR_CONTROL::vex_matrix(Eigen::Matrix3d S)
{
    Eigen::Vector3d v;

    v << 0.5 * (S(2, 1) - S(1, 2)), 0.5 * (S(0, 2) - S(2, 0)), 0.5 * (S(1, 0) - S(0, 1)) ;
    return v;
}

Eigen::Matrix3d LQR_CONTROL::matrix_vex(Eigen::Vector3d v)
{
    Eigen::Matrix3d mat;
    mat << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return mat;
}

Eigen::Vector4d LQR_CONTROL::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw)
{
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0; ///////////235ye

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
}
