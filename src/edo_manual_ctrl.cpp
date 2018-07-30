/** @file edo_manual_ctrl.cpp
 *  @brief ROS node allows for e.DO to be controlled via Linux terminal.
 *  @author Jack Shelata
 *  @date May 28, 2018
 */
#include <ros/ros.h>

#include "MovementCommandQueue.h"
#include "DataDisplay.h"
#include "InitialCheck.h"

#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/JointReset.h"
#include "edo_core_msgs/JointInit.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <iomanip>    //Used to set precision of output to 2 decimal places
#include <ncurses.h>  //Used in the jog function to allow for keystrokes to
                      //be captured without an enter press

/** @brief Function manages jog command creation. Fills in Jog type values
 *  and returns a message to be filled with velocity values on scale from
 *  -1.0 to 1.0.
 *  @param None
 *  @return MovementCommand - Message defined in edo_core_msgs ROS package
 *  @exception None
 */  
edo_core_msgs::MovementCommand createJog(){
  edo_core_msgs::MovementCommand msg;
  msg.move_command = 74;
  msg.move_type = 74;
  msg.ovr = 100;
  msg.target.data_type = 74;
  msg.target.joints_mask = 127;
  msg.target.joints_data.resize(10, 0.0);
  return msg;

}  // createJog()

/** @brief Function manages creating a move command. 
 *  @param type - int type of movement according to e.DO move types defined
 *  in edo_core_pkg
 *  @param delay - int delay in seconds between arriving at waypoint and
 *  moving again
 *  @return MovementCommand - Message defined in edo_core_msgs ROS package
 *  @exception None
 */  
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
    msg.target.joints_data.clear();
  }
  return msg;
}  // createMove()

/** @brief Function to carry out each button press jog command
 *  @param msg - existing MovementCommand to be edited and sent
 *  @param joint_number - int number of the joint to be moved (1-6)
 *  @param jog_ctrl_pub - ROS Publisher to publish the jog message
 *  "/bridge_jog" ROS topic
 *  @param loop_rate - ROS Rate message for set sleep time
 *  @param velocity - double velocity value for jog
 *  @return void
 *  @exception None
 */  
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
}  // jogHelper()

/** @brief Function manages sending jog commands using ncurses library for
 *  push-button key capturing
 *  @param nh - ROS NodeHangle for creating jog publisher
 *  @return void
 *  @exception None
 */  
void jog(ros::NodeHandle& nh){

  ros::Publisher jog_ctrl_pub =
    nh.advertise<edo_core_msgs::MovementCommand>("/bridge_jog",10);
  ros::Rate loop_rate(100);
  edo_core_msgs::MovementCommand msg = createJog();
  char ch = '\n';

  std::cout << "-----\nJog Controls (Press and Hold):\n"
      << "Joint 1 +/-: 'q'/'a'\n"
      << "Joint 2 +/-: 'w'/'s'\n" 
      << "Joint 3 +/-: 'e'/'d'\n"
      << "Joint 4 +/-: 'r'/'f'\n" 
      << "Joint 5 +/-: 't'/'g'\n"
      << "Joint 6 +/-: 'y'/'h'\n"
      << "Set Velocity +/-: 'u'/'j'\n"
      << "Exit: 'x'\n-----\n"
      << "NOTE: VELOCITY STARTS AT 100%\n";
  while(ch != 'y'){
    std::cout << "Enter 'y' to continue: ";
    std::cin >> ch;
  }
  ch = '\n';
  bool last = false;
  double velocity = 1.0;
  doupdate();     //ncurses function to reset window after endwin() has been
                  //called
  initscr();      //ncurses function initializes key capture
  timeout(0);     //ncurses function set to 0 forces getch() to return
                  //ERR when no key is pressed instead of waiting for key
  curs_set(0);    //ncurses makes the cursor invisible
  noecho();       //ncurses function hides pressed keys
  do {
    ch = getch(); //ncurses function returns char of key pressed
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
      
    }  //switch(choice)
  } while(ch != 'X' && ch != 'x');
  endwin(); //Ends ncurses window

}  // jog()

/** @brief Function handles move commands
 *  @param nh - ROS NodeHangle for creating move publisher
 *  @return void
 *  @exception None
 */  
void move(ros::NodeHandle& nh){

  MovementCommandQueue move_ctrl(nh);
      
  int anglesOrCartesian, numEntries = 0, delay = 0;
      
  std::cout << "Select move type:\n"
            << "0 - joint movement to joint point\n"
            << "1 - joint movement to cartesian point\n"
            << "10 - cartesian movement to joint point\n"
            << "11 - cartesian movement to cartesian point\n";
  std::cin  >> anglesOrCartesian;
  std::cout << "Enter number of entries: ";
  std::cin  >> numEntries;
  std::cout << "Enter delay: ";
  std::cin  >> delay;
  std::cout << "Enter ";
  if(anglesOrCartesian == 0 || anglesOrCartesian == 10){
    std::cout << "joint angles as follows and press enter: J1 J2 J3 J4 J5 J6\n";
  }
  else{
    std::cout << "cartesian coordinates as follows and press enter: "
              << "X Y Z A E R\n";
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

}  // move()

/** @brief Function handles initial callibration
 *  @param nh - ROS NodeHandle for creating Publishers for initializaiton,
 *  reset, and calibration
 *  @param recalib - bool is true if the e.DO has aleady been calibrated and
 *  is being recalibrated so that initialization and reset are not repeated
 *  @return void
 *  @exception None
 */  
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
  
    std::chrono::milliseconds timespan(5000);
    std::this_thread::sleep_for(timespan);
    std::cout/* << "Wait at least 5 seconds...\n"*/
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
            << "corresponding white mark\nBE SURE TO MOVE AT LEAST ONE JOINT "
            << "BACKWARDS AND FORWARDS A FEW DEGREES!\nIF YOU DO NOT "
            << "THE EDO WILL NOT CALIBRATE CORRECTLY AND WILL NEED TO BE "
            << "RESET!\n";
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

/** @brief Function creates DataDisplay object and prints data at instant
 *  @param nh - ROS NodeHandle to pass to DataDisplay class constructor for
 *  ROS Subscriber creation
 *  @return void
 *  @exception None
 */
void getData(ros::NodeHandle& nh){
  DataDisplay data(nh);
  while(ros::ok() && !(data.getCartesianPrinted() &&
        data.getStatePrinted() && data.getJointPrinted())){
    ros::spinOnce();
  }  //while()
}  // getData()

/** @brief Function to handle initial startup
 *  @param nh - ROS NodeHandle to pass to InitialStartup class constructor for
 *  ROS Subscriber creation
 *  @return void
 *  @exception None
 */
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

}  // initialStartup()

int main(int argc, char **argv){

  bool cursesActive = false;

  ros::init(argc, argv, "edo_manual_ctrl");

  ros::NodeHandle nh;

  std::cout << std::fixed;
  std::cout << std::setprecision(2);

  initialStartup(nh);
    
  /*
  TODO for Initial Calibration
    - If in error state, should restrict access to move/jog
      commands and provide useful output (and maybe quit node)
    - If in init state, should force full calibration including
      init, reset, and calibrate
    - Find a way to force the user to pause between sending
      init command and reset command (and maybe jog/calibrate commands)
  */
  
  int choice = 0;

  do {
    std::cout << "1 for jog control\n"
              << "2 for move control\n"
              << "3 to re-calibrate\n"
              << "4 to print eDO data\n"
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
 
    } //switch(choice)
  } while(choice != -1);

  return 0;
}  // main() 
