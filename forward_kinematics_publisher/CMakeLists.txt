cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} KDL_INSTALL)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(forward_kinematics_publisher src/for_kin_publisher.cpp)
#target_link_libraries(forward_kinematics_publisher /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir/lib/liborocos-kdl)
#target_link_libraries(forward_kinematics_publisher /opt/ros/fuerte/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir/lib/liborocos-kdl.so)

#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(KDL REQUIRED liborocos_kdl)
#include_directories(${KDL_INCLUDE_DIRS})
#link_directories(${KDL_LIBRARY_DIRS})
#target_link_libraries(forward_kinematics_publisher ${KDL_LIBRARIES})