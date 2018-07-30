/** @file MovementCommandQueue.h
 *  @brief Class definition for MovementCommandQueue
 *  @author Ashwini Magar, Jack Shelata, Alessandro Piscioneri
 *  @date May 28, 2018
 */
#ifndef __MOVEMENT_COMMAND_QUEUE_H__
#define __MOVEMENT_COMMAND_QUEUE_H__

#include <ros/ros.h>
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include <iostream>
#include <queue>

/** @brief Struct to hold information in the MovementCommandQueue class object
 */ 
struct MovementCommandQueueItem {
  edo_core_msgs::MovementCommand message;
  edo_core_msgs::MovementFeedback status;
};

/***************************************************************
**                Class(es) Definition
****************************************************************/

/** @brief This class manages the timing of sending move commands to the e.DO's
 *  "/bridge_move" ROS topic by subscribing to the "/machine_movement_ack" ROS
 *  topic
 */ 
class MovementCommandQueue {
  
public:

  // Class constructor takes in existing NodeHandle reference
  MovementCommandQueue(ros::NodeHandle& nh_in);

  // Function to publish command to "/bridge_move"
  void sendMoveCommand(edo_core_msgs::MovementCommand cmd);

  // Function to manage sending cancel command which must precede move commands
  void pushMoveCommand(edo_core_msgs::MovementCommand cmd);

  // Callback function to manage queued commands based on MovementFeedback
  // messages from "/machine_movement_ack"
  // Mimics code found in ros.service.ts
  void moveAckCallback(const edo_core_msgs::MovementFeedback& fb);

  bool stillRunning();

private:

  ros::NodeHandle nh;	                                        // ROS node handle
  std::queue<MovementCommandQueueItem> pendingQueue;          // Pending msg q
  std::queue<MovementCommandQueueItem> waitingReceiveQueue;   // Received msg q
  std::queue<MovementCommandQueueItem> waitingExecutedQueue;  // Executed msg q
  ros::Subscriber move_ack_sub;                               // 
  ros::Publisher move_ctrl_pub;	                              //
  edo_core_msgs::MovementCommand resetCommand;                // Reset Command
	
};


#endif
