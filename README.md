# e.DO Manual Control

This is the Doxygen documentation for the "edo_manual_ctrl" ROS package that can be used to control the COMAU e.DO educational robot. It provides the ability to initialize, calibrate, and operate the e.DO from the Linux terminal without the use of the Android tablet application. The program supports jog and move commands and can output data from the e.DO to the terminal. It can be helpful in understanding how the e.DO Robot can be controlled by any ROS compatible program.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

This package has only been run on the NVIDIA Jetson TX2 running Ubuntu 16.04 LTS and ROS Kinetic. In order to use the jog mode, the Ncurses library must be installed on your Linux machine.

### Installing

Clone/save this repository into the "src" folder in a catkin workspace directory. Use catkin_make from your catkin workspace directory to build.

Clone/save directory into src folder

```
cd catkin_ws/src/
git clone ...
```

Build the package

```
cd catkin_ws
catkin_make
```

In order to run the program, connect to your e.DO's WiFi network.

You must set the ROS_MASTER_URI of your machine to the e.DO's IP address.

```
export ROS_MASTER_URI=http://192.168.12.1:11311
```

Then set the ROS_IP of your machine to your own IP address on the e.DO's Wifi network.

```
export ROS_IP=192.168.12.68
```

Source your setup.bash file within your catkin workspace.

```
source devel/setup.bash
```

Run the node

```
rosrun edo_manual_ctrl edo_manual_ctrl
```


## Build Dependencies

* [Ncurses](https://www.cyberciti.biz/faq/linux-install-ncurses-library-headers-on-debian-ubuntu-centos-fedora/) - Used for asynchronous jog control
* C++11 - Used to sleep to give time for e.DO to process commands


## Authors

* **Jack Shelata** - [jack@jackshelata.com](mailto:jack@jackshelata.com)

## License

This project is licensed under the MIT License - see [LICENSE.md](LICENSE.md) file for details


