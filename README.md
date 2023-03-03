# LQR_Quadrotor
Quadrotor control with Error-State LQR method
## Lapacke configuration
(https://www.zhang-hb.com/2021/04/28/%E5%9C%A8ubuntu%E7%BC%96%E8%AF%91%E5%AE%89%E8%A3%85lapack%E5%92%8Clapacke/)

## paper
M. Farrell, J. Jackson, J. Nielsen, C. Bidstrup and T. McLain, "Error-State LQR Control of a Multirotor UAV," 
2019 International Conference on Unmanned Aircraft Systems (ICUAS), Atlanta, GA, USA, 2019, pp. 704-711, doi: 10.1109/ICUAS.2019.8798359.
(https://ieeexplore.ieee.org/document/8798359)

## Getting Started
### Install PX4 SITL(Only to Simulate)
Follow the instructions as shown in the [ROS with Gazebo Simulation PX4 Documentation](https://dev.px4.io/master/en/simulation/ros_interface.html)
To check if the necessary environment is setup correctly, you can run the gazebo SITL using the following command
```
cd PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
cd launch
roslaunch mavros_posix_sitl.launch
```
### Running the code
```
roslaunch traj_server.launch
```
