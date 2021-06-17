# __Third Assignment__
## **Table Of Contents**
  - [__Introduction__](#introduction)
    - [*Robot Model*](#robot_model)
  - [__Knowledge Representation__](#knowledge-representation)
  - [__Finite State Machine__](#finite-state-machine)
  - [__Software Architecture__](#software-architecture)
    - [*Description*](#description) 
    - [**Ros messages and actions**](#ros-messages-and-actions)
  - [**System's Features**](#systems-features) 
  - [**Move Base and Gmapping settings**](#move-base-and-gmapping-settings)
  - [**Package and File List**](#package-and-file-list)
  - [__System's Limitation__](#systems-limitations)
  - [__Future Improvements__](#future-improvements)
  - [__Authors and Contacts__](#authors-and-contacts)

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
In the project it's used a python class inside the script [knowledgeRep](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/knowledgeRep.py) for connect the each ball to a specific room which has associated a specific position not yet known a-priori. So if the robot detect a colored ball reached it and save the position, for adding the room to the knowledge and be able to reach the room again after discovered it. The idea is that after discovered all colored balls the robot has a knowledge of the environment knowing each room position.

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
  - The second main problem  is releated to the fact that during the tracking the move_base server is stopped, so we cannot use all the features of autonomous navigation implemented during the NORMAL STATE for example. Since the safety is important, we need to ensure an obstacle avoidance algorithm to guarantee that during the approach of the ball the robot not collide with people or objects. This is done with a simple [BUG ALGORITHM](http://msl.cs.uiuc.edu/~lavalle/cs497_2001/book/uncertain/node3.html#:~:text=The%20BUG%20algorithms%20make%20the,obstacles%20are%20unknown%20and%20nonconvex.&text=This%20allows%20the%20robot%20to,Euclidean%20distance%20to%20the%20goal.) using the lidar. The node subscribe to the /LaserScan topic, the data from the lidar are split between five possible regions: front, front-right, right, front-left, left; then if an obstacle is detected inside one of the regions the node publish to the /cmd_vel topic an appropiate angular velocity for not collide.
- [humanInterface](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/humanInterface.py) = This script implement the human interface, it allows the user to interact with the robot entering the PLAY STATE when the user digit "PLAY" in the shell. The node simply wait for an user digit, if is "PLAY" the logic of the architecture switch to PLAY STATE; if is "GoTo <room_name>" the robot reach the room or start the FIND routine, ONLY IF the robot is in PLAY STATE otherwise the command are simply rejected.
- [knowledgeRep](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/knowledgeRep.py) = Contain the class *Rooms()* over which the knoweldege representation is build see section [__Knowledge Representation__](#knowledge-representation) for more. Here is importan to highlight that contains also importan function used in the command manager:
  - *room_check()* : Check if a detected room (color) was already known.
  - *new_room()* : Add a new room found to the knowledge.
  - *room_position()* : Return the room position given the <room_name>.
  - *room_color()* : Return the room color given an input room name.
  - *room_name()* : Return the room name given an input position.
  - *random_pos()* : Generate random positions (inside our environemnt).
  - *room_range()* : Returns an array contains neighborhoods of a given position.
  - *room_explore()* : Returns a random position away from the rooms already visited to ease the exploration of new rooms. This is done checking if each random position generated belongs to the neighborhood of some already visited rooms (computed thanks to *room_range()*), in case the position is discarded. 
### **ROS Messages and Actions**
In the architecture are used some custom ROS messages of the "std_msgs" library :
| Topic | Msg|
| ---- | ----- |
| **Interface_chatter** | where *String* messages are send from the human interface to the *cmd_manager*, the messages are the command for interaction *play* and *GoTo*. |
| **detection_State** | where the *cmd_manager* publishes *Bool* which notify the status of the ball detection (active or not)|
| **color_found** | where a *String* msgs containing the color of the ball detected it sends from the ball_detection to the *cmd_manager*.|

Other important component is the definition of the costum action server used for ball tracking:
```
# Goal
string color
---
# Result
float64 x
float64 y
---
# Feedback
string state
```
The goal of the action server is simply the color to detect, while the results which are sended back to che command manager are simply the X and Y position of the robot.

## **System's Features** 
### **Move Base and Gmapping settings**
- The **_gmapping_** parameters, contained in the [gmapping.launch](https://github.com/andreatitti97/exp_assignment3/tree/main/launch/gmapping.launch) file, are tuned for a better mapping, below the changes done:
  - The **maxUrange** parameterwas increased for extend the range of the laser in order to map deeper the environment, this was done expecially for the big rooms where the robot can't detect any wall and localization errors are obviously introduced. 
  - The **lsigma** and **ogain** parameter for a smooth resampling effects.
  - Increased the **particles** parameter to increase the ability of the robot to close the loop.
  - The **_move_base_** parametersm contained in [move_base.launch](https://github.com/andreatitti97/exp_assignment3/tree/main/launch/move_base.launch) file, are tuned for a better navigation, below the changes done:
- In the [global_costmap_params.yaml](https://github.com/andreatitti97/exp_assignment3/tree/main/param/global_costmap_params.yaml) were increased the **update_frequency** and **publish_frequency** for a more reactive planner. Also the **inflation_radius** , **cost_scaling_factor** and the **cost_scaling_factor** were tuned to make the robots enter correctly rooms (away from the walls, which are a source of navigation  problems) and to ensure steeper curves near tight corners.
- In the [base_local_planner_params.yaml](https://github.com/andreatitti97/exp_assignment3/tree/main/param/base_local_planner_params.yaml) the parameters increased are **max_vel_x**, **min_vel_x**, **acc_lim_x** and **acc_lim_theta** for a faster navigation. Also the **sim_time** was increased in order to improve the local planning simulation since sometimes the trajectory choosen seems not to be consistent with the environment and the global path trajectory. 
- In the [local_cost_map.yaml](https://github.com/andreatitti97/exp_assignment3/tree/main/param/localcostmap_params.yaml) for improving the local mapping the **with** and **height** parameters were increased, also to ad avoid strange trajecoty.
- In the [costmap_common_params.yaml](https://github.com/andreatitti97/exp_assignment3/tree/main/param/costmap_common_params.yaml) the **obstacle_range** was reduced in order to limit the error described in the [System's limitations](#systems-limitations) section and **robot_radius** was increased in order to keep the robot away from obstacles (expecially because sometime the robot struggle near corners and entrances).


## **Package and File List**
The package of the third assignment (exp_assignment3) provides the following directories:
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

## __System Limitations__
- Wrong exploration in FIND STATE, since now works but can happend that any new room is found if the position produced are in between rooms or near the wall and also the algorithm cannot work with possible different environmnets.
- The magenta ball for now is removed from the detection since the simple OpenCV algorithm have problems with shade on the walls which are confused with magenta color, a better color detection can be implemented, but for now I preferred to remove the magenta color because wrong tracking can be dangerouse for the safety, for the system logic and the knowledge representation.
- The obstacle avoidance in TRACK STATE is very simple, BUG algorithm, another better algorithm can be implemented.
- If sometimes random position are generated near walls or obstacles the robot cannot found a global path. 

## __Future Improvments__
- The exploration in find state can be improved using the "explore-lite" package. 
- Reduce the latency between the image processing and the tracking for achieve better results.
- Improve the obstacle avoidance in TRACK STATE.
- Resolve any problems related to the fact that random positions are produced inside walls or near obstacles.
- Realize a better human interface, more human friendly with the possibility to have a GUI.
- Interact with the robot using voice, so equip the robot model with microphones.

## __Authors and Contacts__

- Author: Andrea Tiranti
- ID: 4856315
- University of Genova
- Robotics Engineering
- Contacts: andrea.tiranti97@gmail.com
- Number: 3487563580
