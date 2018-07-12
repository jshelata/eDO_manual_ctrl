#ifndef JETSON_JOG_CTRL_SRC_MOVE_CTRL_H_
#define JETSON_JOG_CTRL_SRC_MOVE_CTRL_H_

#include <queue>
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include <ros/ros.h>
#include <iostream>


class MovementCommandQueueItem {

public:

	MovementCommandQueueItem(edo_core_msgs::MovementCommand message_in);
	edo_core_msgs::MovementCommand message;
	edo_core_msgs::MovementFeedback status;

};

class MoveControl {

public:

	MoveControl(ros::NodeHandle&;
	void sendMoveCommand(edo_core_msgs::MovementCommand command);
	void pushMoveCommand(edo_core_msgs::MovementCommand command);
	void moveAckCallback(edo_core_msgs::MovementFeedback feedback);

private:
	
	queue<MovementCommandQueueItem> pendingQueue;
	queue<MovementCommandQueueItem> waitingReceiveQueue;
	queue<MovementCommandQueueItem> waitingExecutedQueue;	

};
























#endif /*JETSON_JOG_CTRL_SRC_MOVE_CTRL_H_*/
