/*
  Copyright (c) 2017, COMAU S.p.A.
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.
*/
/*
 * jog_ctrl.cpp
 *
 *  Created on: May 28, 2018
 *      Update: Jun 12, 2018
 *      Author: Comau
 *              Jack Shelata
 *              jack.shelata@external.comau.com
 *      Description: 
 *              Simple controller created to allow for basic eDO control in a
 *              command line interface.
 */
#include <ros/ros.h>
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/JointReset.h"
#include "edo_core_msgs/JointInit.h"
#include "edo_core_msgs/CartesianPose.h"
#include "edo_core_msgs/MachineState.h"
#include "edo_core_msgs/JointStateArray.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <iomanip>    //Used to set precision of output to 2 decimal places
#include <ncurses.h>	//Used in the jog function to allow for keystrokes to
                      //be captured without an enter press

/*

TODO

 - Continue to comment and organize
 - Force wait time in initial calibration
 - Make sure calibration is prompted at the beginning of the program
    - Will likely have to create another class for this
 - Add re-calibrate option

*/

//BELOW IS A DIRECT COPY OF THE ENUM CLASSES FOUND IN "EdoMsgType.h"
//These are not used below but are here as a reference
//The modes and types below use these ASCII characters converted to numbers

  enum E_MOVE_COMMAND {
    E_MOVE_COMMAND_MOVE       = 'M', /* Execute a command statement */
    E_MOVE_COMMAND_CANCEL     = 'C', /* cancella la move in esecuzione
                                        (se presente) */
    E_MOVE_COMMAND_PAUSE      = 'P', /* mette in pausa la move in esecuzione
                                        (se presente) */
    E_MOVE_COMMAND_RESUME     = 'R', /* avvia la move precedentemente messa
                                        in pausa (se presente) */
    E_MOVE_COMMAND_JOGMOVE    = 'J', /* Execute a JOG move */
    E_MOVE_COMMAND_JOGSTOP    = 'S'  /* Stop a JOG move */
  };
  enum E_MOVE_TYPE {
    E_MOVE_TYPE_JOINT    = 'J',
    E_MOVE_TYPE_LINEAR   = 'L',
    E_MOVE_TYPE_CIRCULAR = 'C'
  };
  enum E_MOVE_DEST_POINT {
    E_MOVE_POINT_JOINT     = 'J',
    E_MOVE_POINT_POSITION  = 'P',
    E_MOVE_POINT_XTND_POS  = 'X'
  };



//Declare createMove() function for use in MovementCommandQueue class
edo_core_msgs::MovementCommand createMove(int type, int delay);

//Structure to hold MovementCommand and MovementFeedback in the queue
//(same as in UI)
struct MovementCommandQueueItem {

	edo_core_msgs::MovementCommand message;
	edo_core_msgs::MovementFeedback status;
};

//Class to manage queues for received, executed, and pending commands.
//Mimics UI code found in ros.service.ts
class MovementCommandQueue {

public:
	//Class constructor takes in existing NodeHandle reference
	MovementCommandQueue(ros::NodeHandle& nh_in) {
		nh = nh_in;
		move_ack_sub = nh.subscribe("/machine_movement_ack", 100,
        &MovementCommandQueue::moveAckCallback, this);
		move_ctrl_pub = nh.advertise<edo_core_msgs::MovementCommand>("/bridge_move",
        10,true);
	}  //MovementCommandQueue()

	//Function to publish command to "/bridge_move"
	void sendMoveCommand(edo_core_msgs::MovementCommand command){
		move_ctrl_pub.publish(command);
	}  //sendMoveCommand()

	//Function to manage sending cancel command which must precede move commands
	void pushMoveCommand(edo_core_msgs::MovementCommand command){
		MovementCommandQueueItem rosQueueItem;
		rosQueueItem.message = command;
		if(waitingExecutedQueue.size() == 0 && waitingReceiveQueue.size() == 0){
			MovementCommandQueueItem rosResetQueueItem;
			rosResetQueueItem.message = createMove(-1,0);
			waitingReceiveQueue.push(rosResetQueueItem);
			waitingExecutedQueue.push(rosResetQueueItem);
			sendMoveCommand(rosResetQueueItem.message);
		}
		pendingQueue.push(rosQueueItem);
	}  //pushMoveCommand()
	
	//Callback function to manage queued commands based on MovementFeedback
  //messages from "/machine_movement_ack"
	//Mimics code found in ros.service.ts
	void moveAckCallback(const edo_core_msgs::MovementFeedback& feedback){
		switch(unsigned(feedback.type)){
			
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
	}  //moveAckCallback()

	bool stillRunning(){
		return !(pendingQueue.empty() && waitingReceiveQueue.empty() &&
        waitingExecutedQueue.empty());
	} //stillRunning()


private:
	ros::NodeHandle nh;	
	std::queue<MovementCommandQueueItem> pendingQueue;
	std::queue<MovementCommandQueueItem> waitingReceiveQueue;
	std::queue<MovementCommandQueueItem> waitingExecutedQueue;
	ros::Subscriber move_ack_sub;
	ros::Publisher move_ctrl_pub;	
	
};

//Class to handle displaying joint, cartesian, and machine state data
class DataDisplay {
	
public:
	//Constructor creates and initializes subscribers and bools for checking
  //for completion
	DataDisplay(ros::NodeHandle& nh_in){
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
	}  //DataDisplay()
	
	//Callback function to print CartesianPose message	
	void printPoseData(const edo_core_msgs::CartesianPose& pose){
		if(!cartesianPrinted){
			std::cout << pose << "\n";
			cartesianPrinted = true;
		}
	}  //printPoseData()
	
	//Callback function to print MachineState message
	void printState(const edo_core_msgs::MachineState& state){
		if(!statePrinted){
			std::cout << state << "\n";
			statePrinted = true;
		}
	}  //printState()
	
	//Callback function to print JointStateArray message
	void printJointPose(const edo_core_msgs::JointStateArray& pose){
		if(!jointPrinted){
			std::cout << pose << "\n";
			jointPrinted = true;
		}
	}  //printJointPose()
	
	//Member function to tell whether cartesian data has been printed
	bool getCartesianPrinted(){
		return cartesianPrinted;
	}  //getCartesianPrinted()
	
	//Member function to tell whether machine state data has been printed
	bool getStatePrinted(){
		return statePrinted;
	}  //getStatePrinted()
	
	//Member function to tell whether joint data has been printed
	bool getJointPrinted(){
		return jointPrinted;
	}  //getJointPrinted()

private:
	ros::NodeHandle nh;
	ros::Subscriber cartesian_pose_sub;
	ros::Subscriber machine_state_sub;
	ros::Subscriber joint_pose_sub;

	bool cartesianPrinted, statePrinted, jointPrinted;

};

//Class to handle initial start of node, check state, and start calibration
//procedure if necessary
class InitialCheck {

public:
	
	InitialCheck(ros::NodeHandle& nh_in){
		nh = nh_in;
		machine_state_sub = nh.subscribe("/machine_state", 10,
        &InitialCheck::stateCallback, this);
		stateReceived = false;
	}  //InitialCheck()

	void stateCallback(const edo_core_msgs::MachineState& state){
		stateReceived = true;
		machine_state = state.current_state;
	}  //stateCallback()

	int getState(){
		return machine_state;
	}  //getState()

	bool getStateReceived(){
		return stateReceived;
	}  //getStateReceived()

private:
	ros::NodeHandle nh;
	ros::Subscriber machine_state_sub;
	int machine_state;
	bool stateReceived;
};

//Function manages jog command creation. Fills in Jog type values and
//returns a message to be filled with velocity values on scale from -1.0 to 1.0
edo_core_msgs::MovementCommand createJog(){
	
	edo_core_msgs::MovementCommand msg;

	msg.move_command = 74;
	msg.move_type = 74;
	msg.ovr = 100;
	msg.delay = 0;
	msg.remote_tool = 0;
	msg.cartesian_linear_speed = 0;

	msg.target.data_type = 74;

	msg.target.cartesian_data.x = 0.0,
	msg.target.cartesian_data.y = 0.0,
	msg.target.cartesian_data.z = 0.0,
	msg.target.cartesian_data.a = 0.0,
	msg.target.cartesian_data.e = 0.0,
	msg.target.cartesian_data.r = 0.0,
	msg.target.cartesian_data.config_flags = "";

	msg.target.joints_mask = 127;
	msg.target.joints_data.resize(10, 0.0);

	msg.via.data_type = 0;

	msg.via.cartesian_data.x = 0.0;
	msg.via.cartesian_data.y = 0.0;
	msg.via.cartesian_data.z = 0.0;	
	msg.via.cartesian_data.a = 0.0;
	msg.via.cartesian_data.e = 0.0;
	msg.via.cartesian_data.r = 0.0;
	msg.via.cartesian_data.config_flags = "";

	msg.via.joints_mask = 0;
	msg.via.joints_data.clear();

	msg.tool.x = 0.0;	
	msg.tool.y = 0.0;	
	msg.tool.z = 0.0;	
	msg.tool.a = 0.0;	
	msg.tool.e = 0.0;	
	msg.tool.r = 0.0;

	msg.frame.x = 0.0;	
	msg.frame.y = 0.0;	
	msg.frame.z = 0.0;	
	msg.frame.a = 0.0;	
	msg.frame.e = 0.0;	
	msg.frame.r = 0.0;

	return msg;

} //createJog()

//Function manages creating a move command
edo_core_msgs::MovementCommand createMove(int type, int delay){
	
	edo_core_msgs::MovementCommand msg;
	//Joint movement to joint point
	if(type == 0){
		msg.move_command = 77;
		msg.move_type = 74;
		msg.ovr = 100;
		msg.delay = delay;
		msg.target.data_type = 74;
		msg.target.joints_mask = 63;
		msg.target.joints_data.resize(6, 0.0);
	}
	//Joint movement to cartesian point
	else if(type == 1){
		msg.move_command = 77;
		msg.move_type = 74;
		msg.ovr = 100;
		msg.delay = delay;
		msg.target.data_type = 88;
		msg.target.joints_mask = 63;
		msg.target.joints_data.resize(10, 0.0);
	}
	//Linear movement to joint point
	else if(type == 10){
		msg.move_command = 77;
		msg.move_type = 76;
		msg.ovr = 100;
		msg.delay = delay;
		msg.target.data_type = 74;
		msg.target.joints_mask = 63;
		msg.target.joints_data.resize(6, 0.0);
	}
	//Linear movement to cartesian point
	else if(type == 11){
		msg.move_command = 77;
		msg.move_type = 76;
		msg.ovr = 100;
		msg.delay = delay;
		msg.target.data_type = 88;
		msg.target.joints_mask = 63;
		msg.target.joints_data.resize(10, 0.0);
	}
	//Reset
	else if(type == -1){
		msg.move_command = 67;
		msg.move_type = 0;
		msg.ovr = 0;
		msg.delay = 0;
		msg.target.data_type = 0;
		msg.target.joints_mask = 0;
		msg.target.joints_data.clear();
	}

	msg.remote_tool = 0;
	msg.cartesian_linear_speed = 0;

	msg.target.cartesian_data.x = 0.0,
	msg.target.cartesian_data.y = 0.0,
	msg.target.cartesian_data.z = 0.0,
	msg.target.cartesian_data.a = 0.0,
	msg.target.cartesian_data.e = 0.0,
	msg.target.cartesian_data.r = 0.0,
	msg.target.cartesian_data.config_flags = "";

	msg.via.data_type = 0;

	msg.via.cartesian_data.x = 0.0;
	msg.via.cartesian_data.y = 0.0;
	msg.via.cartesian_data.z = 0.0;	
	msg.via.cartesian_data.a = 0.0;
	msg.via.cartesian_data.e = 0.0;
	msg.via.cartesian_data.r = 0.0;
	msg.via.cartesian_data.config_flags = "";

	msg.via.joints_mask = 0;
	msg.via.joints_data.clear();

	msg.tool.x = 0.0;	
	msg.tool.y = 0.0;	
	msg.tool.z = 0.0;	
	msg.tool.a = 0.0;	
	msg.tool.e = 0.0;	
	msg.tool.r = 0.0;

	msg.frame.x = 0.0;	
	msg.frame.y = 0.0;	
	msg.frame.z = 0.0;	
	msg.frame.a = 0.0;	
	msg.frame.e = 0.0;	
	msg.frame.r = 0.0;

	return msg;

}  //createMove()

//Function to carry out each button press jog command
void jogHelper(edo_core_msgs::MovementCommand& msg, int joint_number,
    ros::Publisher& jog_ctrl_pub, ros::Rate& loop_rate, double velocity){	
	msg.target.joints_data.clear();			
	msg.target.joints_data.resize(10,0.0);
	if(joint_number > 0){
    std::cout << "\rJoint " << joint_number << " + " << velocity << std::flush;
		msg.target.joints_data[joint_number - 1] = velocity;
	}
	else{
    std::cout << "\rJoint " << -1 * joint_number << " - "
              << velocity << std::flush;
		msg.target.joints_data[(-1 * joint_number) - 1] = -1 * velocity;
	}
	jog_ctrl_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
}  //jogHelper()

//Function manages sending jog commands using ncurses library for
//push-button key capturing
void jog(ros::NodeHandle& nh){

	ros::Publisher jog_ctrl_pub =
    nh.advertise<edo_core_msgs::MovementCommand>("/bridge_jog",10);
	ros::Rate loop_rate(100);
	edo_core_msgs::MovementCommand msg = createJog();
	char ch = '\n';

	std::cout << "-----\nJog Controls:\n"
      << "Joint 1 +/-: 'q'/'a'\n"
      << "Joint 2 +/-: 'w'/'s'\n" 
			<< "Joint 3 +/-: 'e'/'d'\n"
      << "Joint 4 +/-: 'r'/'f'\n" 
			<< "Joint 5 +/-: 't'/'g'\n"
      << "Joint 6 +/-: 'y'/'h'\n"
      << "Set Velocity +/-: 'u'/'j'\n"
      << "Exit: 'x'\n-----\n";
	while(ch != 'y'){
		std::cout << "Enter 'y' to continue: ";
		std::cin >> ch;
	}
	ch = '\n';
  bool last = false;
  double velocity = 1.0;
	initscr();		//ncurses function initializes key capture
	//timeout(0);		//ncurses function set to 0 forces getch() to return
  //ERR when no key is pressed instead of waiting for key
	//curs_set(0);		//ncurses makes the cursor invisible
	noecho();		//ncurses function hides pressed keys
	do {
		ch = getch();	//ncurses function returns char of key pressed
                  //returns ERR when no key press
		
		//Switch decides which joint to move and which direction
		//See above std::cout statements for details
		switch(ch) {

			case 'q':
			case 'Q':
				jogHelper(msg, 1, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
					
			case 'a':
			case 'A':
				jogHelper(msg, -1, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
						
			case 'w':
			case 'W':
				jogHelper(msg, 2, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
					
			case 's':
			case 'S':
				jogHelper(msg, -2, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;		
				
			case 'e':
			case 'E':
				jogHelper(msg, 3, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
					
			case 'd':
			case 'D':
				jogHelper(msg, -3, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;		
						
			case 'r':
			case 'R':
				jogHelper(msg, 4, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
					
			case 'f':
			case 'F':
				jogHelper(msg, -4, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
				
			case 't':
			case 'T':
				jogHelper(msg, 5, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
					
			case 'g':
			case 'G':
				jogHelper(msg, -5, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;		
	
			case 'y':
			case 'Y':
				jogHelper(msg, 6, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
					
			case 'h':
			case 'H':
				jogHelper(msg, -6, jog_ctrl_pub, loop_rate, velocity);
				last = true;
				break;
			
      case 'u':
      case 'U':
        if(velocity < 1.0){
          velocity += 0.05;
        }
        std::cout << "\rVelocity: " << velocity << std::flush;
        break;
      
      case 'j':
      case 'J':
        if(velocity > 0.05){
          velocity -= 0.05;
        }
        std::cout << "\rVelocity: " << velocity << std::flush;
        break;

			//Default case handles delay when key is pressed since held down
      //keys are not repeated immediately
			default:
				if(last){					
					for(int i = 0; i < 40; ++i){
						jog_ctrl_pub.publish(msg);
						ros::spinOnce();
						loop_rate.sleep();
					}
					last = false;
				}
				break;			
			
		}  //switch(choice)
	} while(ch != 'X' && ch != 'x');
	endwin();	//Ends ncurses window

}  //jog()

//Function handles move commands
void move(ros::NodeHandle& nh){

	MovementCommandQueue move_ctrl(nh);
			
	int anglesOrCartesian, numEntries = 0, delay = 0;
			
	std::cout << "Select move type:\n"
            << "0 - joint movement to joint point\n"
            << "1 - joint movement to cartesian point\n"
            << "10 - cartesian movement to joint point\n"
		        << "11 - cartesian movement to cartesian point\n";
	std::cin >> anglesOrCartesian;
	std::cout << "Enter number of entries: ";
	std::cin >> numEntries;
	std::cout << "Enter delay: ";
	std::cin >> delay;
	std::cout << "Enter ";
	if(anglesOrCartesian == 0 || anglesOrCartesian == 10){
		std::cout << "joint angles as follows and hit enter: J1 J2 J3 J4 J5 J6\n";
	}
	else{
		std::cout << "cartesian coordinates as follows and hit enter: X Y Z A E R\n";
	}
	std::vector<edo_core_msgs::MovementCommand> pointVec;
	
	for(int i = 0; i < numEntries; ++i){
		edo_core_msgs::MovementCommand msg = createMove(anglesOrCartesian, delay);
		if(anglesOrCartesian == 0 || anglesOrCartesian == 10){
			for(int x = 0; x < 6; ++x){
				scanf("%f", &msg.target.joints_data[x]);
			}
		}
		else{	
			scanf("%f", &msg.target.cartesian_data.x);
			scanf("%f", &msg.target.cartesian_data.y);
			scanf("%f", &msg.target.cartesian_data.z);
			scanf("%f", &msg.target.cartesian_data.a);
			scanf("%f", &msg.target.cartesian_data.e);
			scanf("%f", &msg.target.cartesian_data.r);
		}
		pointVec.push_back(msg);
	}
			
	int numLoops = 1; 
	std::cout << "Enter number of loops: ";
	std::cin >> numLoops;

			
			
	for(int i = 0; i < numLoops; ++i){
		std::vector<edo_core_msgs::MovementCommand>::iterator it = pointVec.begin();
		while(it != pointVec.end()){
			move_ctrl.pushMoveCommand(*it);
			it++;
		}
	}
	//While loop exits when move queue is done running
	while(ros::ok() && move_ctrl.stillRunning()){
		ros::spinOnce();				
	}

}  //move()

//Function handles initial callibration
void calibrate(ros::NodeHandle& nh, bool recalib){

	ros::Publisher calib_pub =
    nh.advertise<edo_core_msgs::JointCalibration>("/bridge_jnt_calib",10);
	
	ros::Rate loop_rate(100);

	edo_core_msgs::JointCalibration calib_msg;

	char proceed;

  if(!recalib){

    ros::Publisher reset_pub =
      nh.advertise<edo_core_msgs::JointReset>("/bridge_jnt_reset",10);
  	ros::Publisher init_pub =
      nh.advertise<edo_core_msgs::JointInit>("/bridge_init",10);	
    
    edo_core_msgs::JointReset reset_msg;
	  edo_core_msgs::JointInit init_msg;

	  std::cout << "Enter 'y' to initialize 6-Axis eDO w/o gripper: ";
	  std::cin >> proceed;
	  init_msg.mode = 0;
	  init_msg.joints_mask = 63;
	  init_msg.reduction_factor = 0.0;
	  init_pub.publish(init_msg);
	  ros::spinOnce();
	  loop_rate.sleep();
	

	  std::cout << "Wait at least 5 seconds...\n"
              << "Enter 'y' to disengage brakes: ";
	  std::cin >> proceed;
	  reset_msg.joints_mask = 63;
	  reset_msg.disengage_steps = 2000;
	  reset_msg.disengage_offset = 3.5;
	  reset_pub.publish(reset_msg);
	  ros::spinOnce();
	  loop_rate.sleep();
  }
	
	std::cout << "Calibration Procedure\n-----\n"
            << "Rotate joints so that each slot is aligned with its\n" 
			      << "corresponding white mark\n";
	std::cout << "Enter 'y' to enter JogMode and calibrate each joint: ";
	std::cin >> proceed;
	
	jog(nh);
	
	std::cout << "Enter 'y' to send calibration command: ";
	std::cin >> proceed;
	calib_msg.joints_mask = 63;
	calib_pub.publish(calib_msg);
	ros::spinOnce();
	loop_rate.sleep();
	
}  //calibrate()

//Function creates DataDisplay object and prints data at instant
void getData(ros::NodeHandle& nh){
	DataDisplay data(nh);
	while(ros::ok() && !(data.getCartesianPrinted() &&
        data.getStatePrinted() && data.getJointPrinted())){
		ros::spinOnce();
	}  //while()
}  //getData()

//Function to handle initial startup
void initialStartup(ros::NodeHandle& nh){
	ros::Rate loop_rate(100);
	InitialCheck check(nh);
	while(!check.getStateReceived()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	int state = check.getState();
	switch(state){

    case 0:
      std::cout << "eDO is in state INIT.\nLaunching calibration...\n";
      calibrate(nh, false);
      break;
  
    case 1:
      std::cout << "eDO is in state NOT_CALIBRATE.\nLaunching calibration...\n";
      calibrate(nh, false);
      break;

    case 2:
      std::cout << "eDO is in state CALIBRATE.\nNo need to calibrate.\n";
      break;
    
    case 3:
      std::cout << "eDO is in state MOVE.\nIs the tablet controller in use?\n";
      break;

    case 4:
      std::cout << "eDO is in state JOG.\nIs the tablet controller in use?\n";
      break;
 
    case 5:
      std::cout << "eDO is in state MACHINE_ERROR.\nRestart reccommended.\n";
      break;
    
    case 6:
      std::cout << "eDO is in state BRAKED.\n";
      break;

    case 255:
      std::cout << "eDO is in state COMMAND_STATE.\n";
      break;

    default:
      std::cout << "eDO is in an unknown state.\nRestart recommended.\n";
      break;

  }  //switch(state)

}  //initialStartup
/* Jack Shelata commented out 
bool readFromFile(std::string filename*COMMENTED->,
          ros::NodeHandle& nh<-COMMENTED*){
  // Create file stream in
  // Open file and check that it is open
  std::ifstream infile;
  infile.open(filename.c_str());
  if(!infile.is_open()){
    printf("Failed to open %s.\n", filename.c_str);
    return false;
  }

  //MovementCommandQueue move_ctrl(nh);
  double angle = 0.0;
  int jointNumber = 0;

  std::vector<edo_core_msgs::MovementCommand> pointVec; 

  edo_core_msgs::MovementCommand msg = createMove(0, 0); 
  
  while(infile >> angle)
        
    msg.target.joints_data[jointNumber] = angle;
    
    if(jointNumber == 2){
      pointVec.push_back(msg);
      for(int i = 0; i < 3; ++i){
        printf("Joint %i: %d\n", i, msg.target.joints_data[i]);
      }
      std::cout << "------\n";
      jointNumber = 0;
      msg = createMove(0, 0);
    }
    else{
      ++jointNumber;
    }
  }



  *COMMENTED->
  std::vector<edo_core_msgs::MovementCommand>::iterator it = pointVec.begin();
		while(it != pointVec.end()){
			move_ctrl.pushMoveCommand(*it);
			it++;
    }<-COMMENTED*

  infile.close();
  if(infile.is_open()){
    printf("Failed to close %s.\n", filename);    
    return false;
  }
  
  return true;

}  //readFromFile()
*/
int main(int argc, char **argv){
	
	ros::init(argc, argv, "jog_ctrl");

	ros::NodeHandle nh;

	std::cout << std::fixed;
	std::cout << std::setprecision(2);

  initialStartup(nh);
		
	/*
	TODO for Initial Calibration
		- When node is started, should check machine state
			~ In order to check, must create subscriber to grab a message
			~ Once message has been grabbed, decide what state and what to do
			~ Kill the subsciber
		- If in calibrate state, should not require a calibration
      though calibration should remain and option
		- If in error state, should restrict access to move/jog
      commands and provide useful output (and maybe quit node)
		- If in init state, should force full calibration including
      init, reset, and calibrate
		- Change calibrate to have two paths: One for initial
      calibration and one for re-calibration
		- Find a way to force the user to pause between sending
      init command and reset command (and maybe jog/calibrate commands)
	*/
	
	int choice = 0;

	do {
		std::cout << "1 for jog control\n"
              << "2 for move control\n"
              << "3 to re-calibrate\n"
              << "4 to print eDo data\n"
              << "5 to read commands from .txt file\n"
              << "-1 to exit: ";
		std::cin >> choice;

		switch(choice){

		case 1:
			jog(nh);
			break;
		
		case 2:
			move(nh);
			break;
			
		case 3:
			calibrate(nh, true);
			break;
		case 4:
			getData(nh);
			break;
    /* Jack Shelata Commented Out
    case 5:
      std::string filename;
      std::cout << "\nPlease enter filename: ";
      std::cin >> filename; 
      readFromFile(filename);
      break;	    
    */  
		} //switch(choice)
	} while(choice != -1);

	return 0;
}
