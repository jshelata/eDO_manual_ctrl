/** @file DataDisplay.h
 *  @brief Class definition for DataDisplay
 *  @author Ashwini Magar, Jack Shelata, Alessandro Piscioneri
 *  @date May 28, 2018
 */
#ifndef __DATA_DISPLAY_H__
#define __DATA_DISPLAY_H__

#include <ros/ros.h>
#include "edo_core_msgs/CartesianPose.h"
#include "edo_core_msgs/MachineState.h"
#include "edo_core_msgs/JointStateArray.h"
#include <iostream>

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class creates and stores ROS Subscribers to
 *  output the e.DO's Cartesian Position, Machine State, and Joint Angle Data.
 */
class DataDisplay {
  
public:

  // Constructor creates and initializes subscribers and bools for checking
  // for completion
  DataDisplay(ros::NodeHandle& nh_in);

  // Callback function to print CartesianPose message 
  void printPoseData(const edo_core_msgs::CartesianPose& pose);

  // Callback function to print MachineState message
  void printState(const edo_core_msgs::MachineState& state);

  // Callback function to print JointStateArray message
  void printJointPose(const edo_core_msgs::JointStateArray& pose);

  // Member function to tell whether cartesian data has been printed
  bool getCartesianPrinted();

  // Member function to tell whether cartesian data has been printed
  bool getStatePrinted();

  // Member function to tell whether joint data has been printed
  bool getJointPrinted();

private:

  ros::NodeHandle nh;                                 // ROS Node Handle
  ros::Subscriber cartesian_pose_sub;                 // ROS subscriber
  ros::Subscriber machine_state_sub;                  // ROS subscriber
  ros::Subscriber joint_pose_sub;                     // ROS subscriber
  bool cartesianPrinted, statePrinted, jointPrinted;  // True when printed
  
};


#endif
