#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <std_msgs/Empty.h>
#include <lapacke.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;
Eigen::Vector3d load_p_d,load_v_d,load_a_d;
Eigen::Vector3d traj_axis_;
Eigen::Vector3d traj_origin_;
Eigen::Vector3d traj_radial_;
double traj_radius_, traj_omega_;
double dt_;
double T_;
enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class LQR_CONTROL
{
        private:
        enum FlightState { WAITING_FOR_HOME_POSE, TAKE_OFF, MISSION_EXECUTION, LANDING, LANDED } node_state;
        MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;
        template <class T>
        void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
            ros::Rate pause(hz);
            ROS_INFO_STREAM(msg);
            while (ros::ok() && !(*pred)) {
                ros::spinOnce();
                pause.sleep();
            }  
        };
        geometry_msgs::Pose home_pose_;
        bool received_home_pose;
        public:
        ros::Subscriber imu_sub_,velocity_sub_,position_sub_, mavstate_sub_;
        ros::Publisher  pose_pub_, systemstatusPub_,angularVelPub_;
        ros::ServiceClient arming_client_, set_mode_client_;
        //parameter
        double drone_mass, gravity_;
        Eigen::Matrix3d J_;
        Eigen::Vector3d e3_;
        bool sim_enable_;
        mavros_msgs::State current_state_;
        mavros_msgs::CommandBool arm_cmd_;
        //mav
        Eigen::Vector3d drone_p, drone_v, drone_Omega;
        Eigen::Vector4d drone_q_b2i,drone_q_i2b;
        Eigen::Matrix3d drone_R;
        //controller
        Eigen::Matrix3d R_d_last;
        //controller output
        double thrust_;
        Eigen::Vector3d torque_;
        //controller useful
        double theta_x, theta_y, d_theta_x, d_theta_y;
        Eigen::Vector3d e_x, e_v;
        Eigen::Vector3d thrustVir_, Fn;
        Eigen::Vector3d intergral_xL;
        ros::Time t_now, t_last;
        //controller parameter
        double max_pos_err_ ;
  	double max_vel_err_ ;
  	double max_ang_err_ ;
 	double max_throttle_err_ ;
	double max_omega_err_ ;
        
        public:
        LQR_CONTROL(){}
        ~LQR_CONTROL(){}

        void init_param(ros::NodeHandle& nh_);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void mavposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void mavtwistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
        void statusloopCallback(const ros::TimerEvent &event);
        void pubSystemStatus();
        void arm_disarm_func(bool on_or_off);
        void set_px4_mode_func(string mode);
        Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &Rot);
        Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q);
        Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
        Eigen::Vector3d vex_matrix(Eigen::Matrix3d S);
        Eigen::Matrix3d matrix_vex(Eigen::Vector3d v);
        ////////////////////////////LQR////
        Eigen::Vector4d q_cheng(const Eigen::Vector4d &p, const Eigen::Vector4d &q);
        Eigen::Vector3d log2tangent(const Eigen::Vector4d &p);
        Eigen::Matrix<double, 9, 9> a_calculate();
        Eigen::Matrix<double, 9, 4> b_calculate();
        Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);         ////desired attitude;        
        MatrixXd solve(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R);
        Eigen::Matrix<double, 9, 1> error_state(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                                const Eigen::Vector3d &target_acc);
        void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
        void publish_cmd(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                                const Eigen::Vector3d &target_acc);
};
