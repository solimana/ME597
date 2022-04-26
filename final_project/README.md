# ME_597_final_project_env

To spawn the dynamic obstacles:

`rosrun final_project dynamic_obstacles.py`

### To watch videos of the tasks:

https://purdue0-my.sharepoint.com/:f:/g/personal/solimana_purdue_edu/ErUW64bX51VOv2TGLcsSDB8BFTtCcpPCPXbHNXF9EJn2GA?e=cHKNJH



## To run task 1  : 
1. Make sure you have downloaded and compiled ros_autonomous_slam and turtlebot3 packages attached in this submission. 
2. Run : roslaunch final_project task1.launch 
3. Hit the top left , bottom left, bottom right, top right corners, then the white squre around turtlebot to start mapping.

## To run task 2  : 
1. Run : roslaunch final_project task2.launch 
2. Run : rosrun final_project navigation.py
3. Have rviz subscribe to global_plan topic 


## To run task 3  : 
1. Run : roslaunch final_project task3launch 
2. Run : rosrun final_project dynamic_obstacles.py
3. Run : rosrun final_project navigation3.py
