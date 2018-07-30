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

/** @brief Construct MovementCommandQueue object. Constructor creates and
 *  initializes a ROS Publisher and Subsciber to manage sending move commands
 *  to the e.DO and sets the resetCommand value.
 *  @param nh_in - ROS NodeHandle object by reference to create ROS Publishers
 *  and Subscribers
 *  @return MovementCommandQueue object
 *  @exception None
 */
MovementCommandQueue::MovementCommandQueue(ros::NodeHandle& nh_in) {
  nh = nh_in;
  move_ack_sub = nh.subscribe("/machine_movement_ack", 100,
      &MovementCommandQueue::moveAckCallback, this);
  move_ctrl_pub = nh.advertise<edo_core_msgs::MovementCommand>("/bridge_move",
      10,true);
  resetCommand.move_command = 67;
}  // MovementCommandQueue::MovementCommandQueue()
 
/** @brief Function to publish command to "/bridge_move"
 *  @param cmd - MovementCommand message to be published
 *  @return void
 *  @exception None
 */
void MovementCommandQueue::sendMoveCommand(edo_core_msgs::MovementCommand cmd){
  move_ctrl_pub.publish(cmd);
}  // MovementCommandQueue::sendMoveCommand()

/** @brief Function manages pushing commands onto the pendingQueue.
 *  If the executed and receive queues are both empty, this function pushes
 *  the resetCommand onto both queues.
 *  @param cmd - MovementCommand message to be pushed onto the pendingQueue
 *  @return void
 *  @exception None
 */
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

/** @brief Callback function to manage queued commands based on MovementFeedback
 *  messages from "/machine_movement_ack". Mimics the structure found in
 *  the UI files in "ros.service.ts".
 *  @param fb - MovementFeedback message from "/machine_movement_ack" ROS
 *  topic.
 *  @return void
 *  @exception None
 */
void MovementCommandQueue::moveAckCallback(
                                    const edo_core_msgs::MovementFeedback& fb){
  switch(unsigned(fb.type)){
    
    case 0:     //Command Received
      
      if(!waitingReceiveQueue.empty()){
        waitingReceiveQueue.front().status.type = 0;
        waitingReceiveQueue.pop();
      }
      else{
        //std::cout << "Unexpected Ack 0\n";
      }
      break;

    case 1:     //Send next command if available

      if(!pendingQueue.empty()){
        waitingReceiveQueue.push(pendingQueue.front());
        waitingExecutedQueue.push(pendingQueue.front());
        sendMoveCommand(pendingQueue.front().message);
        pendingQueue.pop();
      }
      break;

    case 2:     //Command executed
      
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

/** @brief Function check returns true if all queues are empty.
 *  @param None
 *  @return bool - True if all queues are empty
 *  @exception None
 */
bool MovementCommandQueue::stillRunning(){
  return !(pendingQueue.empty() && waitingReceiveQueue.empty() &&
      waitingExecutedQueue.empty());
} // MovementCommandQueue::stillRunning()

