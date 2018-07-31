/** @file StateChecker.h
 *  @brief Class definition for StateChecker
 *  @author Jack Shelata
 *  @date May 28, 2018
 */
#ifndef __STATE_CHECKER_H__
#define __STATE_CHECKER_H__

#include <ros/ros.h>
#include "edo_core_msgs/MachineState.h"
#include <iostream>

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class creates and stores a ROS Subscriber to get the state
 *  number of e.DO when the edo_manual_ctrl node is initially started
 */
class StateChecker {
  
public:

  StateChecker(ros::NodeHandle& nh_in);

  void stateCallback(const edo_core_msgs::MachineState& state);

  int getState();

  bool getStateReceived();

private:

  ros::NodeHandle nh;                     // 
  ros::Subscriber machine_state_sub;      // 
  int machine_state;                      // 
  bool stateReceived;                     // 
  
};


#endif
