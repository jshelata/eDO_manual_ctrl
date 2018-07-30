/** @file MovementCommandQueue.cpp
 *  @brief Class implementation for MovementCommandQueue. Used to send move
 *  commands to the e.DO with proper timing as the Tablet HMI does.
 *  @author Ashwini Magar, Jack Shelata, Alessandro Piscioneri
 *  @date May 28, 2018
 */

#include "MovementCommandQueue.h"

/***************************************************************
**                Function(s) Definition
****************************************************************/

// Class constructor takes in existing NodeHandle reference
MovementCommandQueue::MovementCommandQueue(ros::NodeHandle& nh_in) {
	nh = nh_in;
	move_ack_sub = nh.subscribe("/machine_movement_ack", 100,
      &MovementCommandQueue::moveAckCallback, this);
	move_ctrl_pub = nh.advertise<edo_core_msgs::MovementCommand>("/bridge_move",
      10,true);
  resetCommand.move_command = 67;
}  // MovementCommandQueue::MovementCommandQueue()

// Function to publish command to "/bridge_move"
void MovementCommandQueue::sendMoveCommand(edo_core_msgs::MovementCommand cmd){
	move_ctrl_pub.publish(cmd);
}  // MovementCommandQueue::sendMoveCommand()

// Function to manage sending cancel command which must precede move commands
void MovementCommandQueue::pushMoveCommand(edo_core_msgs::MovementCommand cmd){
	MovementCommandQueueItem rosQueueItem;
	rosQueueItem.message = cmd;
	if(waitingExecutedQueue.size() == 0 && waitingReceiveQueue.size() == 0){
		MovementCommandQueueItem rosResetQueueItem;
		rosResetQueueItem.message = resetCommand;
		waitingReceiveQueue.push(rosResetQueueItem);
		waitingExecutedQueue.push(rosResetQueueItem);
		sendMoveCommand(rosResetQueueItem.message);
	}
	pendingQueue.push(rosQueueItem);
}  // MovementCommandQueue::pushMoveCommand()

//Callback function to manage queued commands based on MovementFeedback
//messages from "/machine_movement_ack"
//Mimics code found in ros.service.ts
void MovementCommandQueue::moveAckCallback(
                                    const edo_core_msgs::MovementFeedback& fb){
	switch(unsigned(fb.type)){
		
		case 0:			//Command Received
			
			if(!waitingReceiveQueue.empty()){
				waitingReceiveQueue.front().status.type = 0;
				waitingReceiveQueue.pop();
			}
			else{
				//std::cout << "Unexpected Ack 0\n";
			}
			break;

		case 1:			//Send next command if available

			if(!pendingQueue.empty()){
				waitingReceiveQueue.push(pendingQueue.front());
				waitingExecutedQueue.push(pendingQueue.front());
				sendMoveCommand(pendingQueue.front().message);
				pendingQueue.pop();
			}
			break;

		case 2:			//Command executed
			
			if(!waitingExecutedQueue.empty()){
				waitingExecutedQueue.front().status.type = 2;
				waitingExecutedQueue.pop();	
				if(pendingQueue.size() > 0 && waitingExecutedQueue.empty()){

					if(!pendingQueue.empty()){
						waitingReceiveQueue.push(pendingQueue.front());
						waitingExecutedQueue.push(pendingQueue.front());
						sendMoveCommand(pendingQueue.front().message);
						pendingQueue.pop();
					}
				}
			}
			break;
	}
}  // MovementCommandQueue::moveAckCallback()

bool MovementCommandQueue::stillRunning(){
	return !(pendingQueue.empty() && waitingReceiveQueue.empty() &&
      waitingExecutedQueue.empty());
} // MovementCommandQueue::stillRunning()

