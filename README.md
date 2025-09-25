## Turtle Catcher Robot
### This project contains to main parts:
#### 1. Turtle Spawner: A node that spawns new turtles on a parameterized frequancy and also kills the caught turtles 
#### 2. Turtle Controller: A node resposible to make the master turtle (/turtlesim) catch the spawned turtles. This node controls the motion of the master turtle node using a P controller to generate suitable linear and angular velocities at each timestamp. There is also a targeting mechanism that the master turtle finds the nearest neighbour using the Euclidean distance measure and locks on the target to catch. 

## How to run:
### The running process is as simple as launching the project using the content of the launch folder provided in the my_robot_bringup folder. The config file is also accessable under the config folder in my_robot_bringup to configure the parameters os spawn frequancy and the initial position and orientation of the master turtle. 
<img width="1920" height="1080" alt="Screenshot_2025-09-23_23-14-04" src="https://github.com/user-attachments/assets/142f9b10-cc84-4d21-902f-572d31c9e8b0" />
