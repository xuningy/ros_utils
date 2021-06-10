#pragma once

#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros_utils/msgs_utils.h>

struct State
{
  double t = 0.0;
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;

  Eigen::Vector3d rpy;
  Eigen::Vector3d ang_vel; // droll, dpitch, dyaw


  void fromOdometry(const nav_msgs::Odometry& msg)
  {
    pos = ros_utils::msgs::fromPointToEigen(msg.pose.pose.position);
    vel = ros_utils::msgs::fromVectorToEigen(msg.twist.twist.linear);
    rpy = ros_utils::msgs::fromQuatToEigenRPY(msg.pose.pose.orientation);
    ang_vel = ros_utils::msgs::fromVectorToEigen(msg.twist.twist.angular);

  }

  void print(std::string state = "", bool verbose = false)
  {
    if (verbose) {
      if (!state.empty()) printf("%s:", state);
      printf("t: %.2f | pos: %.2f, %.2f, %.2f | vel: %.2f, %.2f, %.2f | acc: %.2f, %.2f, %.2f | jerk: %.2f, %.2f, %.2f | rpy:  %.2f, %.2f, %.2f | ang_vel:  %.2f, %.2f, %.2f\n",
      t,
      pos(0), pos(1), pos(2),
      vel(0), vel(1), vel(2),
      acc(0), acc(1), acc(2),
      jerk(0), jerk(1), jerk(2),
      snap(0), snap(1), snap(2),
      rpy(0), rpy(1), rpy(2),
      ang_vel(0), ang_vel(1), ang_vel(2));
    }
    else {
      printf("t: %.2f | pos: %.2f, %.2f, %.2f | vel: %.2f, %.2f, %.2f | rpy:  %.2f, %.2f, %.2f\n",
      t,
      pos(0), pos(1), pos(2),
      vel(0), vel(1), vel(2),
      rpy(0), rpy(1), rpy(2));
    }
  } // end void print

};
