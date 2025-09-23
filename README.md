## Turtle Catcher Robot
### This project contains to main parts:
#### 1. Turtle Spawner: A node that spawns new turtles on a parameterized frequancy and also kills the caught turtles 
#### 2. Turtle Controller: A node resposible to make the master turtle (/turtlesim) catch the spawned turtles. This node controls the motion of the master turtle node using a P controller to generate suitable linear and angular velocities at each timestamp. 

## How to run:
### The running process is as simple as launching the project using the content of the launch folder provided in the my_robot_bringup folder. The config file is also accessable under the config folder in my_robot_bringup to configure the parameters os spawn frequancy and the initial position and orientation of the master turtle. 
