# TurtleBotProject

This is the code for the Turtle Leader-Follower Project. All the nodes that contain the algorithm for tracking, moving, and controlling the TurtleBot3 are in the directory catkin_ws/src/assignment/src.

Note when using the robot:

Access catkin_ws/src/assignment/src. From here, you can run teleop_key_for_robot_1.py to manually control the turtle bot with the marker. 
Run the Matlab file "tracking_node.m"
After launching the environment, run manual control and Matlab tracking node. You can manually move the guiding turtlebot so the turtlebot Waffle will follow. 
Please note that due to some constraints in the code. It's recommended that the guiding turtlebot accelerate to 0.1 m/step and stay constant at this speed most of the time. 
Furthermore, it's recommended that the guiding turtle bot not turn more than 100 degrees quickly, which could make the SURF function lose track of the AR tag and stop the Matlab node completely. 

# Contribution

Dinh Nam Luu 40%

Joshua Muzik 30%

Dan Pham 30%

 
