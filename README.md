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
``` ]

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
- [cmdManager](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/cmdManager.py) = is the core of the architecture as in the previous assignments. It takes input from the _UI_ node and the _roomsDetector_ . Based on the state in which it is, this node can make requests to:
  -  the _move_base_ action server to reach a certain position.
  -  the _track_ action server to reach a detected room (ball).
- [ballDetection](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/ballDetection.py) = is a simple openCV algorithm that analyzes the camera images to detect the balls (rooms). After that this node sends the color of the detected ball to the _commandManager_. Finally it interrupts the subscription to the camera topic. Thus it doesn't process any images until the _commandManager_ awaken it again.
- [ballTracking](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/ballTracking.py) = is a action server that tracks a ball of a given color. The algorithm of tracking it's very similar to the _ball_track_ of the previous assignment. When the robot reaches the ball it will read its own position and send it back to the _commandManager_ so that it can store the position of the discovered room. 
  
  If for some reason the ball is no longer detected,then the robot will turn on itself in both directions in an attempt to see the ball again. If after some time it has not succeeded, it switches back to the appropriate state.

  Finally, it was implemented a very simple **obstacle_avoidance** algorithm using the laser scan data. It was necessary since when the robot starts to track a ball, the _move_base_ algorithm is deactivated by the _commadManager_ and its integrated obstacle avoidance as well. Basically, this algorithm evaluates the obstacle positions and apply an angular velocity directly to the robot differential motors in order to avoid them. If there are no free spaces to reach the ball, the mission is aborted and the robot returns to the _NORMAL_ state.    
- [humanInterface](https://github.com/andreatitti97/exp_assignment3/tree/main/scripts/humanInterface.py) = is a very simple user interface that allows the user to switch in the _PLAY_ mode and enter a room to reach.
Notice that the _move_base_ goal is aborted every time a ball is detected or the play command is typed.
For more details regarding the scripts, see the doxygen documentation.

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
roslaunch final_assignment robot.launch
```
Run the human interface:
```
rosrun exp_assignment3 human_interface.py
```