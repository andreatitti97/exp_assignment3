# __Third Assignment__
## **Table Of Contents**
  - [__Introduction__](#introduction)
  - [__Knowledge Representation__](#knowledge-representation)
  - [__Finite State Machine__](#finite-state-machine)
  - [__Software Architecture__](#software-architecture)
    - [Description](#description) 
  - [**Contacts**](#contacts)

## __Introduction__ 
The project represent the final assignment for the course of Experimental Robotics Laboratory of Robotics Engineering degree of University of Genova. It's an upgrade of previous works done during the course releated to mobile robots and autonomous navigation, with the intent to create an house robot capable of autonomous navigation and interaction with human and the environment surrounding him. The environment is represented by a human and by an house with different colored balls inside each room, check the figure below for having an idea. 
![World](https://github.com/andreatitti97/exp_assignment3/tree/main/documentation/figures/Immagine.jpg)

The scope of the assignment is to equipe the robot with sensors and design an appropiate architecture such is can be able to:
- Navigate autonomously while mapping the area.
- Detect colored balls.
- Interact with the human.
- Detect and avoid obstacles.
More precisely the logic of the program should have different states to handle various possible behaviours of the robot that can happend during the interaction with the environment or the human.
### Robot Model
The model used is a simple mobile robot (implemented in the previous assignments of the course) but now the possibility of move the head has been set aside since it was a problem for safety and movements, so the head is fixed. In the figure below we can see the robot, is a differential drive robot with two fixed wheels and equipped with:
- LIDAR SENSOR: Best choice for use SLAM pkg and gmapping pks for autonomous navigation.
- RGB CAMERA: Fundamental for color detection and ball tracking.

![Robot](https://github.com/andreatitti97/exp_assignment3/tree/main/documentation/figures/robot.jpg)

## __Knoweldege Representation__
Before moving to the logic and the architecture of the program we must introduce the concept of knowledge representation, how to represent information with high level of abstraction and connect them in a way that the robot can have a knowledge of the environment surrounding him. The base idea is to assign to each colored ball a room of the house, in particularly in the project the correspondences are the following:

 ```
 self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"Leavingroom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"Bathroom",'color': "magenta", "x":0, "y":0, 'detected':False},
        {'name':"Bedroom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x": -5,"y":7, 'detected':True}
        ]
]
```
In the project it's used a python class inside the script "knowledgeRep.py" for connect the each ball to a specific room which has associated a specific position not yet known a-priori. So if the robot detect a colored ball reached it and save the position, for adding the room to the knowledge and be able to reach the room again after discovered it. The idea is that after discovered all colored balls the robot has a knowledge of the environment knowing each room position.

## __Finite State Machine__
The logic of the program is based upon 5 different states in which the robot do different stuff, in the figure we can see the complete FSM.

![FSM](https://github.com/andreatitti97/exp_assignment3/tree/main/documentation/figures/FSM.jpg)

- __NORMAL STATE__: The robot should move randomly in the house, if a new ball is detected switch to TRACK STATE, if a play command is received from the user should switch to PLAY state. After a while without external events that affect the FSM the robot should go to SLEEP STATE.
- __SLEEP STATE__: The robot should return to the user (fixed position) and wait for a random amount of time before return to NORMAL STATE.
- __PLAY STATE__: This state is entered after a user command, and we can call it the interaction state in which the robot perform a task given by the human. In particularly the robot after the "PLAY" command should return to the user (fixed position) and wait for a room to go, if the room is previously detected the robot know the position so can go for it. Otherwise should switch to FIND STATE. After some iterations return to NORMAL STATE:
- __TRACK STATE__: This state is entered if a new ball is detected, and implement a tracking algorithm for reach the ball, after that the robot save his position so now the room is added to knowledge since we can associated a position to it. When the ball is reached it switch back to NORMAL STATE.
- __FIND STATE__: This state occurs if during PLAY state the user make a request of reaching an unknown room. So the robot start to navigate randomly for finding a ball, if this happend the FSM switch to TRACK STATE and after reaching the ball and save the position return to find state and check if the new room found correspond to the user request, if this happend the FSM switch back to PLAY state, if not simply save the new room inside the knoweldge and keep exploring for some iterations before switch back to PLAY STATE.

## __Software Architeture__
The software architecture of the project is made up by two main parts, the "navigation architecture" which rely on stable and tested ROS packages such as: move_base, navigation, gmapping, SLAM; and the "control architecture" which control the robot and the logic of the system. In the next figure we can see the entire architecture, below is explained in detail the control part of the architecture designed for the project.   

![FSM](https://github.com/andreatitti97/exp_assignment3/tree/main/documentation/figures/rosgraph.jpg)

### Description
- [cmdManager](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/cmdManager.py) = This is the core of the architecture, contains the logic of the architecture since implement the FSM. It receive inputs from the human interface and the ball detection nodes and can make request to two server, "the _move_base_ action server" to reach a certain position and the "_track_ action server" to reach a detected ball.
- [ballDetection](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/ballDetection.py) = This script implement an OpenCV algorithm for color detection, when it started (subscribing to the camera topic) each image is processed until a ball is detected (at least one contours of the given colors is found). It sends to the command manager the color of the ball detected publishing in the "/color_found" topic. It also subscribe to another topic "/detection_state" in which a boolean is send to start or stop the detection according to the state in which the command manager is. 
- [ballTracking](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/ballTracking.py) = This script implement an action server for tracking the previous detected ball. It receive from the command manager a color to track, that is the goal of the action server, then after the detection it starts to send command to the robot for reaching the ball in a smooth way(the node subscribe to "/cmd_vel" topic). This is done using OpenCV (the node subscribe to the camera topic), the idea is that we want to have the ball inside the camera image with a certain radius and centered with the image, we need to apply this two conditions for consider the ball as reached. After reached the ball the position of the robot is saved and assigned to the room just discovered, for doing this it subscribe to the "/odom" topic. 
There are also optional features added for improve the tracking since during several test some problems arised when the robot approach the ball.
- The first problem is releated to the possibility that the robot lost the ball while tracking it, this is MAINLY CAUSED by the latency between the image processed and the tracking, can happend that the robot see instantly the ball in few frame and start track it but then lost information about it (a corner, a color detection error, the robot was turning ... I noticed there where different causes for lost the ball). The solution was found implementing a simple algorithm which happend only if the OpenCV algorithm lost the ball and make the robot turn a little bit left and right for found the ball, after a while the track action is aborted and the logic switch to NORMAL STATE.
- The second main problem  is releated to the fact that during the tracking the move_base server is stopped, so we cannot use all the features of autonomous navigation implemented during the NORMAL STATE for example. Since the safety is important, we need to ensure an obstacle avoidance algorithm to guarantee that during the approach of the ball the robot not collide with people or objects. This is done with a simple "BUG ALGORITHM" [BUG](http://msl.cs.uiuc.edu/~lavalle/cs497_2001/book/uncertain/node3.html#:~:text=The%20BUG%20algorithms%20make%20the,obstacles%20are%20unknown%20and%20nonconvex.&text=This%20allows%20the%20robot%20to,Euclidean%20distance%20to%20the%20goal.) using the lidar. The node subscribe to the /LaserScan topic, the data from the lidar are split between five possible regions: front, front-right, right, front-left, left; then if an obstacle is detected inside one of the regions the node publish to the /cmd_vel topic an appropiate angular velocity for not collide.
- [humanInterface](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/humanInterface.py) = This script implement the human interface, it allows the user to interact with the robot entering the PLAY STATE when the user digit "PLAY" in the shell. The node simply wait for an user digit, if is "PLAY" the logic of the architecture switch to PLAY STATE; if is "GoTo <room_name>" the robot reach the room or start the FIND routine, ONLY IF the robot is in PLAY STATE otherwise the command are simply rejected.

## **Package and File List**
The final assignment package provides the following directory:
- **action** = definitionof the tracking action server
- **config** = contain the Rviz configuration file, it can be modified as needed.
- **launch** = contain launch files of the project, see the [Run the Project](#run) section for more.
- **param** = contain configuration files for the parameters of move_base pkg, gmapping pkg, navigation pkg, SLAM pkg (see [Navigation settings](#navigation-settings)).
- **scripts** = contains all the developed code (see [Software Architecture](#software-architecture)).
- **urdf** = contains the urdf models files of the robot and the human.
- **world** = contains the world used in Gazebo.

## **Installation**
Tested on ubuntu 16.04 and ros-kinetic. 

### **Packages** 
Download the *gmapping* and *move_base* packages, for install from source: 
- **gmapping =**  https://github.com/CarmineD8/SLAM_packages.git
- **move_base =** https://github.com/CarmineD8/planning.git
For automatic installation:
```
sudo apt-get install ros-kinetic-openslam-gmapping
sudo apt-get install ros-kinetic-navigation
```
Then build the workspace.

```
catkin_make
```
NB: Remember to source each shell, is preffered to modifying the .bashrc file.

## **Run the Project**

Launch the project:
```
roslaunch exp_assignment3 robot.launch
```
Run the human interface:
```
rosrun exp_assignment3 human_interface.py
```