/** @file DataDisplay.cpp
 *  @brief Class implementation for DataDisplay. Used to get cartesian, joint,
 *  and machine state data from e.DO.
 *  @author Jack Shelata
 *  @date May 28, 2018
 */

#include "DataDisplay.h"

/***************************************************************
**                Function(s) Definition
****************************************************************/

/** @brief Construct DataDisplay object. Constructor creates and initializes
 *  subscribers and bools for checking for completion.
 *  @param nh_in - ROS NodeHandle object by reference to create ROS Publishers
 *  and Subscribers
 *  @return DataDisplay object
 *  @exception None
 */
DataDisplay::DataDisplay(ros::NodeHandle& nh_in){
  nh = nh_in;
  cartesian_pose_sub = nh.subscribe("/cartesian_pose", 10,
      &DataDisplay::printPoseData, this);
  machine_state_sub = nh.subscribe("/machine_state", 10,
      &DataDisplay::printState, this);
  joint_pose_sub = nh.subscribe("/machine_algo_jnt_state", 10,
      &DataDisplay::printJointPose, this);
  cartesianPrinted = false;
  statePrinted = false;
  jointPrinted = false;
}  // DataDisplay::DataDisplay()

/** @brief Callback function to print CartesianPose message
 *  @param pose - CartesianPose message type from "/cartesian_pose" ROS topic
 *  @return void
 *  @exception None
 */
void DataDisplay::printPoseData(const edo_core_msgs::CartesianPose& pose){
  if(!cartesianPrinted){
    std::cout << pose << "\n";
    cartesianPrinted = true;
  }
}  // DataDisplay::printPoseData()

/** @brief Callback function to print MachineState message
 *  @param pose - MachineState message type from "/machine_state" ROS topic
 *  @return void
 *  @exception None
 */
void DataDisplay::printState(const edo_core_msgs::MachineState& state){
  if(!statePrinted){
    std::cout << state << "\n";
    statePrinted = true;
  }
}  // DataDisplay::printState()

/** @brief Callback function to print JointStateArray message
 *  @param pose - MachineState message type from "/machine_algo_jnt_state"
 *  ROS topic
 *  @return void
 *  @exception None
 */
void DataDisplay::printJointPose(const edo_core_msgs::JointStateArray& pose){
  if(!jointPrinted){
    std::cout << pose << "\n";
    jointPrinted = true;
  }
}  // DataDisplay::printJointPose()

/** @brief Getter member function to tell whether cartesian data has been
 *  printed
 *  @param None 
 *  @return bool - Value of cartesianPrinted (true if data was printed)
 *  @exception None
 */
bool DataDisplay::getCartesianPrinted(){
  return cartesianPrinted;
}  // DataDisplay::getCartesianPrinted()

/** @brief Getter member function to tell whether State data has been
 *  printed
 *  @param None 
 *  @return bool - Value of statePrinted (true if data was printed)
 *  @exception None
 */
bool DataDisplay::getStatePrinted(){
  return statePrinted;
}  // DataDisplay::getStatePrinted()

/** @brief Getter member function to tell whether Joint data has been
 *  printed
 *  @param None 
 *  @return bool - Value of jointPrinted (true if data was printed)
 *  @exception None
 */
bool DataDisplay::getJointPrinted(){
  return jointPrinted;
}  // DataDisplay::getJointPrinted()
