##Maze solving bot in ROS and Gazebo
The goal of this project was to simulate a two wheeled differential drive robot in Gazebo simulator and apply various maze-solving algorithms to solve any given maze. The problem was divided into two broad parts:
1. Control of robot motion: Usage of PID control algorithms to precisely control linear and angular velocity of the bot.
2. Path planning: We tested various path planning algorithms and eventually settled for Breadth First Search (BFS) algorithm

Detailed installation instructions are given in the inner directories.

To run the simulation:
1. roslaunch pkg_techfest_imc final.launch
2. Navigate to scripts folder and run: python motion_3.py

