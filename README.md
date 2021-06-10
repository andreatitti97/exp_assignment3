REOSITORY FOR THE 3RD ASSIGNMENT OF EXPERIMENTAL ROBOTICS LABORATORY COURSE - Andrea Tiranti 4856315

PROBLEMS TO FIX: 

- handle tracking of more than one object ( track the first detected )
- Normal state continusly publish random position even outside of the state
- Change Magenta color range since sometimes the orange walls are detected instead of the ball
- Fix human interface strange error at hte beginning (?)
- ADD FIND STATE

FOR THE REST THE WORK IS NEARLY DONE

RUN 

type in a sourced shell for launch the all project

roslaunch exp_assignment3 robot.launch

type in another sourced shell for human interface (INTERACT IN PLAY STATE)

rosrun exp_assignment3 human_interface.py 
