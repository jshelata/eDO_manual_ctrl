/** @file InitialCheck.cpp
 *  @brief Class implementation for InitialCheck. Used to check the state of
 *  e.DO upon startup and start the necessary startup procedures like
 *  calibration.
 *  @author Ashwini Magar, Jack Shelata, Alessandro Piscioneri
 *  @date May 28, 2018
 */

#include "InitialCheck.h"

/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct InitialCheck object. Constructor creates and initializes
 *  subscriber to check the e.DO's Machine State upon the edo_manual_ctrl
 *  node startup so that the program can launch calibration or alert the user
 *  if e.DO is in an error state.
 *  @param nh_in - ROS NodeHandle object by reference to create ROS Subscriber
 *  @return InitialCheck object
 *  @exception None
 */
InitialCheck::InitialCheck(ros::NodeHandle& nh_in){
  nh = nh_in;
  machine_state_sub = nh.subscribe("/machine_state", 10,
      &InitialCheck::stateCallback, this);
  stateReceived = false;
}  // InitialCheck::InitialCheck()

/** @brief Callback function to get and save state number from "/machine_state"
 *  ROS topic.
 *  @param state - MachineState message type from "/machine_state" ROS topic
 *  @return void
 *  @exception None
 */
void InitialCheck::stateCallback(const edo_core_msgs::MachineState& state){
  stateReceived = true;
  machine_state = state.current_state;
}  // InitialCheck::stateCallback()

/** @brief Getter member function to return the saved machine state number.
 *  @param None
 *  @return int - 
 *  @exception None
 */
int InitialCheck::getState(){
  return machine_state;
}  // InitialCheck::getState()

/** @brief Getter member function to return the stateReceived bool
 *  @param None
 *  @return bool - Value of stateReceived (true if state has been received)
 *  @exception None
 */
bool InitialCheck::getStateReceived(){
  return stateReceived;
}  // InitialCheck::getStateReceived()
