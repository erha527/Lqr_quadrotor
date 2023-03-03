cmake_minimum_required(VERSION 2.8)
project(traj_server)

# set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    mavros_msgs
    tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS})

link_directories(
    ${PCL_LIBRARY_DIRS})

add_library( traj_controller 
    src/traj_controller.cpp 
)
target_link_libraries( traj_controller
    ${catkin_LIBRARIES} 
)  
add_executable (traj_server_node 
    src/traj_server.cpp)

target_link_libraries(traj_server_node
    traj_controller
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
liblapacke.a liblapack.a libblas.a libcblas.a librefblas.a libgfortran.a libquadmath.so)

add_executable (test_node 
    src/test_model.cpp)

target_link_libraries(test_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
liblapacke.a liblapack.a libblas.a libcblas.a librefblas.a libm.a libgfortran.a libquadmath.so)
