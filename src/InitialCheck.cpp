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

InitialCheck::InitialCheck(ros::NodeHandle& nh_in){
	nh = nh_in;
	machine_state_sub = nh.subscribe("/machine_state", 10,
      &InitialCheck::stateCallback, this);
	stateReceived = false;
}  // InitialCheck::InitialCheck()

void InitialCheck::stateCallback(const edo_core_msgs::MachineState& state){
	stateReceived = true;
	machine_state = state.current_state;
}  // InitialCheck::stateCallback()

int InitialCheck::getState(){
	return machine_state;
}  // InitialCheck::getState()

bool InitialCheck::getStateReceived(){
	return stateReceived;
}  // InitialCheck::getStateReceived()
