# ROS_projects
This is the repository for robotics course projects.

For more infomantion please go to https://xiangzhuo-ding.github.io/ROS_Projects.html

## Requirements:
1. Environment: Ubuntu 16.04
2. ROS Kinetic
3. xterm

## Set up the workspace:
1. Go to each project.
2. Run ```catkin_make```

## Let's run it!
1. Run ```source devel/setup.bash```
2. Run the simulator 
    ```roslaunch robot_sim kuka_lwr.launch cycle_move:=true rviz:=true```
    or
    ```roslaunch robot_sim ur5.launch cycle_move:=true rviz:=true```
3. Now send out cmmmand
    ```rosrun run forward_kinematics.py```
