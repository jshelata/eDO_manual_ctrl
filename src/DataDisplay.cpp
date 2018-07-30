/** @file DataDisplay.cpp
 *  @brief Class implementation for DataDisplay. Used to get cartesian, joint,
 *  and machine state data from e.DO.
 *  @author Ashwini Magar, Jack Shelata, Alessandro Piscioneri
 *  @date May 28, 2018
 */

#include "DataDisplay.h"

/***************************************************************
**                Function(s) Definition
****************************************************************/

// Constructor creates and initializes subscribers and bools for checking
// for completion
DataDisplay::DataDisplay(ros::NodeHandle& nh_in){
	nh = nh_in;
	cartesian_pose_sub = nh.subscribe("/cartesian_pose", 10,
      &DataDisplay::printPoseData, this);
	machine_state_sub = nh.subscribe("/machine_state", 10,
      &DataDisplay::printState, this);
	joint_pose_sub = nh.subscribe("machine_algo_jnt_state", 10,
      &DataDisplay::printJointPose, this);
	cartesianPrinted = false;
	statePrinted = false;
	jointPrinted = false;
}  // DataDisplay::DataDisplay()

// Callback function to print CartesianPose message	
void DataDisplay::printPoseData(const edo_core_msgs::CartesianPose& pose){
	if(!cartesianPrinted){
		std::cout << pose << "\n";
		cartesianPrinted = true;
	}
}  // DataDisplay::printPoseData()

// Callback function to print MachineState message
void DataDisplay::printState(const edo_core_msgs::MachineState& state){
	if(!statePrinted){
		std::cout << state << "\n";
		statePrinted = true;
	}
}  // DataDisplay::printState()

// Callback function to print JointStateArray message
void DataDisplay::printJointPose(const edo_core_msgs::JointStateArray& pose){
	if(!jointPrinted){
		std::cout << pose << "\n";
		jointPrinted = true;
	}
}  // DataDisplay::printJointPose()

// Member function to tell whether cartesian data has been printed
bool DataDisplay::getCartesianPrinted(){
	return cartesianPrinted;
}  // DataDisplay::getCartesianPrinted()

// Member function to tell whether machine state data has been printed
bool DataDisplay::getStatePrinted(){
	return statePrinted;
}  // DataDisplay::getStatePrinted()

// Member function to tell whether joint data has been printed
bool DataDisplay::getJointPrinted(){
	return jointPrinted;
}  // DataDisplay::getJointPrinted()
