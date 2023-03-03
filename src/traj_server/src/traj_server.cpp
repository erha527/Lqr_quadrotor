#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <traj_controller/traj_controller.hpp>
double last_yaw_ = 0.0;
bool receive_traj_ = false;

LQR_CONTROL lqr_control;


bool exe_traj() {
  double time = ros::Time::now().toSec();
  load_p_d << 0.0, 0.0, 1.0;
  load_v_d << 0.0, 0.0, 0.0;
  load_a_d << 0.0, 0.0, 0.0;
//   traj_origin_ = load_p_d;
//   traj_omega_ = 0.2;
//   traj_axis_ << 0.0, 0.0, 1.0;
//   traj_radial_ << 1.0, 0.0, 0.0;
//  traj_origin_ << 0.0, 0.0, 0.6;
//   T_ = 2 * 3.14 / traj_omega_;
//   double theta = traj_omega_ * time;
//   load_p_d = std::cos(theta) * traj_radial_ + std::sin(theta) * traj_axis_.cross(traj_radial_) +
//                  (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
//   load_v_d = traj_omega_ * traj_axis_.cross(load_p_d);
//   load_a_d = traj_omega_ * traj_axis_.cross(load_v_d);
 
    lqr_control.publish_cmd(load_p_d, load_v_d, load_a_d);
    return true;
}

void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
  ros::Time heartbeat_time_ = ros::Time::now();
}

void cmdCallback(const ros::TimerEvent &e) {
  lqr_control.set_px4_mode_func("OFFBOARD");
  lqr_control.arm_disarm_func(true);
  /*if (!receive_traj_) {
    geometry_control.position_cmd(load_p_d, 0.0);
    return;
  }*/
  ros::Time time_now = ros::Time::now();
  if (exe_traj()) {
    return;
  } else if (exe_traj()) {
    return;
  }
}

int main(int argc, char ** argv){
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh_ctrl("~");
  ros::Subscriber heartbeat_sub = nh_ctrl.subscribe("heartbeat", 10, heartbeatCallback);

  ros::Timer cmd_timer = nh_ctrl.createTimer(ros::Duration(0.005), cmdCallback);

  lqr_control.init_param(nh_ctrl);
  load_p_d << 0.0, 0.0, 0.2;
  load_v_d = Eigen::Vector3d::Zero();
  load_a_d = Eigen::Vector3d::Zero();

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
