# __Third Assignment__
## **Table Of Contents**
  - [__Introduction__](#introduction)
  - [__Knowledge Representation__](#knowledge-representation)
  - [__Finite State Machine__](#finite-state-machine)
  - [__Software Architecture__](#software-architecture) 
    - [**Move Base and Gmapping settings**](#move-base-and-gmapping-settings)
    - [**Robot Model & Knowledge Rappresentation**](#robot-model--knowledge-rappresentation)
    - [__FSM Description__](#fsm-description)
    - [**Explore**](#explore) 
  
  - [**Package and File List**](#package-and-file-list)
  - [**Installation**](#installation)
  - [**Run**](#run)
  - [**System's Features**](#systems-features)
  - [**System's Limitation**](#systems-limitations)
  - [**Possible Technical Improvements**](#possible-technical-improvements)
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
'''
self.ROOMS = [ 
       {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
       {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
       {'name':"Leavingroom",'color': "green", "x":0, "y":0, 'detected':False},
       {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
       {'name':"Bathroom",'color': "magenta", "x":0, "y":0, 'detected':False},
       {'name':"Bedroom",'color':"black","x":0,"y":0, 'detected':False},
       {'name':"Home",'color':"","x": -5,"y":7, 'detected':True}
       ]
'''
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
