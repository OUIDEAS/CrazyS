#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  // Initialize the node
  ros::init(argc, argv, "altitude_control_");
  ros::NodeHandle altitude_control_;

  // Set up program to publish the position setpoint
  ros::Publisher pos = altitude_control_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  // Publish to terminal
  ROS_INFO("***ALTITUDE CONTROL***");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);

  while(!unpaused){
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  }
  // Loop at 1 Hz
  ros::Rate loop_rate(1);

  // Start the counter
  int count = 0;

  //Initialize variables
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  double desired_yaw = 0.0;

  //Give gazebo time to start up
  ros::Duration(5.0).sleep();
  int flightmode;
  // Loop through
  int z;
  while(ros::ok())
  {
    z = count + 1;

    if(count < 5){
      flightmode = 1;
    }else if(count > 10){
      flightmode = 2;
    }

    switch(flightmode)
    {
      case 1: count++;
      case 2: count = count - 1;
    }

      Eigen::Vector3d desired_position(0.0, 0.0, float(z));
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);

    pos.publish(trajectory_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;

}
