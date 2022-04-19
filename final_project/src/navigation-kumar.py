#!/usr/bin/env python3

from pickle import FALSE, TRUE
import re
import sys
import os
import numpy as np

import math
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry

from Controlllers import PidController
from astar_map import trigger
from nav_msgs.msg import OccupancyGrid
from numpy import floor
from math import pow, atan2, radians, sqrt


mapData=OccupancyGrid()



class Navigation:
    """! Navigation node class.
    This class should serve as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """
    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.current_heading = 0
        self.goal_heading = 0


    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(100)
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=50)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=50)
        rospy.Subscriber('/map', OccupancyGrid, self.mapCallBack)

        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=50)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)


    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        print("Goal pose x: ", self.goal_pose.pose.position.x)
        print("Goal pose y: ", self.goal_pose.pose.position.y)

        q0 = self.goal_pose.pose.orientation.x
        q1 = self.goal_pose.pose.orientation.y
        q2 = self.goal_pose.pose.orientation.z
        q3 = self.goal_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.goal_heading = math.degrees( math.atan2(n,d) )
        print("Goal yaw: ", self.goal_heading)

    def rotate(self,p, origin=(0, 0), degrees=0):
        angle = np.deg2rad(degrees)
        R = np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle),  np.cos(angle)]])
        o = np.atleast_2d(origin)
        p = np.atleast_2d(p)
        return np.squeeze((R @ (p.T-o.T) + o.T).T)


    def mapCallBack(self,data):
       
        global mapData
        mapData=data

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle.
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        # TODO: MAKE SURE YOUR POSITION ESTIMATE IS GOOD ENOUGH.
        self.ttbot_pose = data.pose
        cov = data.pose.covariance

        print("Current bot x: ", self.ttbot_pose.pose.position.x)
        print("Current bot y: ", self.ttbot_pose.pose.position.y)

        """
        adding new functionality to get the current heading of the ttbot
        converting quaternion to euler
        """
        q0 = self.ttbot_pose.pose.orientation.x
        q1 = self.ttbot_pose.pose.orientation.y
        q2 = self.ttbot_pose.pose.orientation.z
        q3 = self.ttbot_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.current_heading = math.degrees( math.atan2(n,d) )
        print("Current bot yaw: ", self.current_heading)

    def callibrate(self):
        """
        callibrate ttbot to localize in rviz
        """
        cmd_vel = Twist()
        if (not(self.current_heading>-30 and self.current_heading<-5)): #callibration will fail if we start in [-5,-30] heading
            cmd_vel.angular.z = 0.25
            self.cmd_vel_pub.publish(cmd_vel)
            print("Callibrating, please wait")
            return 1 #we are still callibrating, continue to stay in while loop
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Callibration done")
            return 0 #we are done callibrating, exit while loop after setting yaw to 0


    def calib_align(self, target=0):
        cmd_vel = Twist()
        dt = 1/100
        error = abs(target-self.current_heading)

        if (error>1.5):
            PID_obj_1 = PidController(0.01, 0.0000, 0.0001, dt, 0.0, 1.5)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("calib Aligning, please wait")
            cmd_vel.linear.x = 0
            self.cmd_vel_pub.publish(cmd_vel)
            return 1
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Done aligning")
            return 0


    def align(self, target=0):
        cmd_vel = Twist()
        dt = 1/100
        error = abs(target-self.current_heading)
        print("heading err   : " , error)
        print("desired heading   : " , target)
        print("current heading   : " , self.current_heading)
        if (error>1.5):
            PID_obj_1 = PidController(0.01, 0.0000, 0.0001, dt, 0.0, 0.5)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("Aligning, please wait")
            cmd_vel.linear.x = 0
            self.cmd_vel_pub.publish(cmd_vel)
            return 1
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Done aligning")
            return 0

    def move(self, target_y=0, target_x=0):
        
        cmd_vel = Twist()
        dt = 1/100
        # d = ( (target_y-self.ttbot_pose.pose.position.y)**2 + (target_x-self.ttbot_pose.pose.position.x)**2)**(0.5)
        # error = abs(d)
        d = sqrt(pow((target_y - self.ttbot_pose.pose.position.y), 2) + pow((target_x - self.ttbot_pose.pose.position.x), 2))
        error =d
        print("errrrr : ",error)
        
        # steering_angle = -90 + math.degrees(atan2(self.ttbot_pose.pose.position.y-target_y,  self.ttbot_pose.pose.position.x-target_x))
        # error_s = abs(heading-self.current_heading)
        # error_s2 = (heading-self.current_heading)
        # print("error_s : ",error_s)
        if (error>0.1):
            PID_obj_2 = PidController(0.1, 0.000001, 0.0001, dt, 0.0, 0.2)
            cmd_vel.linear.x = PID_obj_2.step(error)
            self.cmd_vel_pub.publish(cmd_vel) 
            # print("xvel",cmd_vel.linear.x)
            # cmd_vel.linear.z   =  1.5 * (steering_angle - (self.current_heading))
            # cmd_vel.linear.z = np.clip(cmd_vel.linear.z, 0.0, 0.4) # clip function to not exceed min/max soft bounds

            print("Moving, please wait")
            # print("y desired : " ,target_y)
            # cmd_vel.linear.x = 0
            # self.cmd_vel_pub.publish(cmd_vel)
            return 1
        else:
            cmd_vel.linear.x = 0
            # cmd_vel.linear.z   = 0
            # self.cmd_vel_pub.publish(cmd_vel)
            print("Done moving to position")
            self.cmd_vel_pub.publish(cmd_vel) 
            return 0

        # if (error_s>0.1):
        #     PID_obj_3 = PidController(0.1, 0.00000000001, 0.0000000001, dt, 0.0, 0.2)
        #     cmd_vel.angular.z = PID_obj_3.step(error_s)
        #     print("zvel = ",cmd_vel.linear.z)
        #     # cmd_vel.linear.z  = 0
        # else : 
        #     cmd_vel.angular.z  = 0
        #     # return


        # self.cmd_vel_pub.publish(cmd_vel)        




    
    def a_star_path_planner(self):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        global mapData
        y_rviz_goal, x_rviz_goal = self.goal_pose.pose.position.y, self.goal_pose.pose.position.x
        y_rviz_current, x_rviz_current = self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x

        resolution = mapData.info.resolution
        Xstartx = mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y
        width = mapData.info.width
        print("width , " ,width)
        print("Xstartx , " ,Xstartx)
        print("Xstarty , " ,Xstarty)
        print("y_rviz_goal , " ,y_rviz_goal)
        print("x_rviz_goal , " ,x_rviz_goal)

        Data = mapData.data

        # index = int(	(floor((y_rviz_goal-Xstarty)/resolution) *
        #           width)+(floor((x_rviz_goal-Xstartx)/resolution)))
        # width = 250
        index_y_g = int(	(floor((y_rviz_goal-Xstarty)/resolution))*200/480)
        index_X_g = int(	(floor((x_rviz_goal-Xstartx)/resolution))*200/480)

        index_y_c = int(	(floor((y_rviz_current-Xstarty)/resolution))*200/480)
        index_X_c = int(	(floor((x_rviz_current-Xstartx)/resolution))*200/480)

        # index_y_g = mapData.info.origin.position.y + \
        # (y_rviz_goal//mapData.info.width)*mapData.info.resolution


        # # downsample
        # index_y_g = int(floor(index_y_g * 200/480))
        # index_x_g = int(floor(index_x_g * 200/480))
        # index_y_c = int(floor(index_y_c * 200/400))
        # index_x_c = int(floor(index_x_c * 200/400))

        

        origin=(100,100)
        
        new_points_g = self.rotate([(0,index_y_g)], origin=origin, degrees=180)

        new_points_c = self.rotate([(0,index_y_c)], origin=origin, degrees=180)
        # print('nnnmmmmmmmmmmmmmmmmmmmmmmmm : ',new_points_c)

        index_y_c = int(new_points_c[1])

        index_y_g = int(new_points_g[1])

        st_pt = str(index_y_c) + ',' + str(index_X_c)
        end_pt = str(index_y_g) + ',' + str(index_X_g)

        print("Goal pose x: ", self.goal_pose.pose.position.x)
        print("Goal pose y: ", self.goal_pose.pose.position.y)
        print("index_y_G :  ,index_X_g: ",index_y_g,index_X_g)
        print("index_y_c :  ,index_X_c: ",index_y_c,index_X_c)


        # st_pt = "143,54"
        # end_pt = "92,107"               

        path = trigger(st_pt,end_pt) 




        # y_pgm = int(90 + 200*(y_rviz_goal-y_rviz_current))
        # x_pgm = int(90 + 200*(x_rviz_goal-x_rviz_current))
        # end_pt = str(y_pgm) + ',' + str(x_pgm)


 

        # print("index :  ",index)
        # print("xpgm :  ",x_pgm)


        #we should ideally pass end_pt as argument
        #because we don't know the  ping b/w pixel and gazebo data
        #we are not passing end_pt
        # path = trigger("200,50","150,125") 
 

        return path

    def convert_waypoints(self, path):
        '''
        transforming from pixel to gazebo waypoints
        loop thru path_as
        convert y_pgm to y_rviz
        convert x_pgm to x_rviz
        calculate heading needed to move to next point
            initial heading = 0 after my callibration
            final heading = self.goal_heading #decided by rviz/user
            intermediate heading = 90 - math.degrees(math.atan2( (x2-x1) ,(y1-y2)) ,in rviz frame
        '''
        global mapData
        # y_rviz_goal, x_rviz_goal = self.goal_pose.pose.position.y, self.goal_pose.pose.position.x
        # y_rviz_current, x_rviz_current = self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x

        resolution = mapData.info.resolution
        Xstartx = mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y
        width = mapData.info.width
        
        
        transformed_path = []

        # for X in path:
        #     y1 = X[0]
        #     x1 = X[1]
        #     y2 = path[X+1]

        #     # print("y : " , y1)

        #     origin=(100,100)

        #     # rotation to correct incorrect y-axis
        #     new_points_g = self.rotate([(0,y1)], origin=origin, degrees=180)
        #     y1 = int(new_points_g[1])
        #     # x2 = x1
        #     # x1 = y1
        #     # y1 =x2

        #     # print(resolution)
            
        #     y_rviz = (y1*480/200)*resolution+Xstarty
        #     x_rviz = (x1*480/200)*resolution+Xstartx
        #     heading = self.goal_heading
        
        #     transformed_path.append((y_rviz, x_rviz, heading))




        # y_rviz = self.goal_pose.pose.position.y
        # x_rviz = self.goal_pose.pose.position.x
        # heading = self.goal_heading
        # transformed_path.append((y_rviz, x_rviz, heading))


        # for index in range(len(path)-1):
        #     y1, x1 = path[index]
        #     y2, x2 = path[index+1]
        #     origin=(100,100)

        #     # rotation to correct incorrect y-axis
        #     new_points_g = self.rotate([(0,y1)], origin=origin, degrees=180)
        #     y1 = int(new_points_g[1])
        #     new_points_g = self.rotate([(0,y2)], origin=origin, degrees=180)
        #     y2 = int(new_points_g[1])


        #     # offset_y_pgm = y2 -y1
        #     # offset_x_pgm = x2 -x1

        #     # new_points_g = self.rotate([(0,y1)], origin=origin, degrees=180)

        #     # y1 = int(new_points_g[1])
        # # index_y_c = int(new_points_c[1])

        #     # y_rviz = int(new_points_g[:,1])

        #     print(resolution)
        #     y_rviz = (y1*480/200)*resolution+Xstarty
        #     x_rviz = (x1*480/200)*resolution+Xstartx

        #     # new_points_g = self.rotate([(0,y_rviz)], origin=origin, degrees=180)


        # # index_y_c = int(new_points_c[1])

        #     # y_rviz = int(new_points_g[:,1])

        #     # x_rviz = -0.2 + 0.05*offset_x_pgm
        #     heading = -90 + math.degrees(math.atan2( (y1-y2),(x2-x1) ))
        #     transformed_path.append((y_rviz, x_rviz, heading))

        # y_rviz = self.goal_pose.pose.position.y
        # x_rviz = self.goal_pose.pose.position.x
        # heading = self.goal_heading
        # transformed_path.append((y_rviz, x_rviz, heading))

        # print(transformed_path)


        for index in range(len(path)-1):
            y1, x1 = path[index]
            y2, x2 = path[index+1]
            origin=(100,100)

            # rotation to correct incorrect y-axis
            new_points_g = self.rotate([(0,y1)], origin=origin, degrees=180)
            y1 = int(new_points_g[1])
            new_points_g = self.rotate([(0,y2)], origin=origin, degrees=180)
            y2 = int(new_points_g[1])


            # print(resolution)
            y_rviz = (y1*480/200)*resolution+Xstarty
            x_rviz = (x1*480/200)*resolution+Xstartx

            heading =  math.degrees(math.atan2( (y2-y1),(x2-x1) ))
            transformed_path.append((y_rviz, x_rviz, heading))
            

        y_rviz = self.goal_pose.pose.position.y
        x_rviz = self.goal_pose.pose.position.x
        heading = self.goal_heading
        transformed_path.append((y_rviz, x_rviz, heading))


        

        return transformed_path
        

    def path_follower(self,transformed_path):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        for y,x,heading in transformed_path:
            while self.align(heading):
                 pass
            print("done align")
            # self.move(y, x)
            while self.move(y, x):
                pass
            # # try:
                
            #     self.move(y, x,heading)
            #     # print("here")
            # except rospy.ROSInterruptException:
            #     print("program interrupted before completion", file=sys.stderr)
        


    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        path_complete = False
        timeout = False
        path = self.a_star_path_planner() #call only once
        transformed_path = self.convert_waypoints(path)
   
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        for X in transformed_path:
            pose = PoseStamped()
            pose.pose.position.x = X[1]
            pose.pose.position.y = X[0]
            msg.poses.append(pose)

        self.path_pub.publish(msg)



        while not rospy.is_shutdown():
            self.path_follower(transformed_path)
            # print(ts2)

            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()

    while (nav.callibrate()):
        pass
    while(nav.calib_align(0)):
        pass

    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)