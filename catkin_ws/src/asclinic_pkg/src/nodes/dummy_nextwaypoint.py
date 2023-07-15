#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt32
from asclinic_pkg.msg import Path2,LeftRightFloat32,TemplateMessage
from geometry_msgs.msg import PoseArray,Pose,Twist,Vector3
import numpy as np
import math as MATH
from enum import Enum

X_THRESHOLD = 0.3
Y_THRESHOLD = 0.15
class WaypointTracking:
    def __init__(self):
        # Initialise a publisher
        self.waypoint_publisher = rospy.Publisher("/asc/next_waypoint",Twist,queue_size=10)
        


        # Initialise a subscriber
        self.currentWayPointArray = PoseArray()
        self.nextWayPointArray = PoseArray()

        self.currentWayPoint = Pose()
        self.nextWayPoint = Pose()
        self.current_pose = Twist()
        
        self.current_wayPoint = Twist()
        self.next_wayPoint = Twist()
        

        self.counter = 0
        self.published_counter = 1
        self.end = False

        self.error_y = 1.0
        self.error_x = 1.0

        self.new_angle = 0
        self.error_heading = 0
        self.stop_counter = 0
        self.flag = 0
        #rospy.loginfo(self.current_pose)
        rospy.Subscriber("/dummy/reference_target", PoseArray, self.referenceCallback)
        rospy.Subscriber("/asc/fused_pose", Twist, self.fusedPoseSubscriberCallback)
        #Initialise a timer
        #self.counter = 0
        rospy.Timer(rospy.Duration(2.0), self.timerCallbackForpublishing)
     


    def timerCallbackForpublishing(self,event):
         if ((self.stop_counter) == 0):
            if (self.flag == 1) :
                self.waypoint_publisher.publish(self.next_wayPoint)
                self.published_counter = self.counter
                self.flag = 0
            #self.waypoint_publisher .publish(self.next_wayPoint)    
        
    def fusedPoseSubscriberCallback(self,msg):
            if ((self.stop_counter) == 0):
                self.current_pose.linear.x = msg.linear.x
                self.current_pose.linear.y = msg.linear.y 
                self.current_pose.angular.z = msg.linear.z

                self.current_pose.linear.x = self.current_pose.linear.x
                self.current_pose.linear.y = self.current_pose.linear.y
                #self.current_pose.angular.z = self.current_pose.angular.z + self.angle_check( (self.twist.angular.z))

                self.currentWayPoint = self.currentWayPointArray.poses[self.counter]
                self.nextWayPoint = self.currentWayPointArray.poses[self.counter + 1]

                #(gradient,crossover) = self.line_generate(self.currentWayPoint.position.x,self.currentWayPoint.position.y,self.nextWayPoint.position.x,self.nextWayPoint.position.y)
                #rospy.loginfo(gradient)
                #rospy.loginfo(crossover)
                #(error_x1,error_y1) = self.calculate_threshold_boundary(gradient,crossover)
                #self.error_x = error_x1
                #self.error_y = error_y1
                #rospy.loginfo(self.error_x)
                #rospy.loginfo(self.error_y)
                self.error_x = np.sqrt(pow(self.currentWayPoint.position.x - self.current_pose.linear.x,2) + pow(self.currentWayPoint.position.y - self.current_pose.linear.y,2) )
                if (((abs(self.error_x) <= X_THRESHOLD)) or (self.counter == 0)) :
                    rospy.loginfo("H")
                    rospy.loginfo(len(self.currentWayPointArray.poses))
                    if (self.counter > (len(self.currentWayPointArray.poses))):
                        self.stop_counter = 1
                    else:
                        self.counter = self.counter + 1
                        self.flag = 1
                        self.next_wayPoint.linear.x = self.nextWayPoint.position.x
                        self.next_wayPoint.linear.y = self.nextWayPoint.position.y
                else:
                    rospy.loginfo(len(self.currentWayPointArray.poses))
                    rospy.loginfo("MOVING TO CURRENT WAYPOINT")
                    self.next_wayPoint.linear.x = self.currentWayPoint.position.x
                    self.next_wayPoint.linear.y = self.currentWayPoint.position.y
                    #rospy.loginfo(self.currentWayPoint)
                    rospy.loginfo(self.next_wayPoint)
                    #self.waypoint_publisher.publish(self.next_wayPoint)
            
      
    def calculate_threshold_boundary(self,gradient,crossover):
        y_current_pose = gradient*self.current_pose.linear.x + crossover
        #rospy.loginfo(y_current_pose)
        error_y1 = abs(self.current_pose.linear.y) - abs(y_current_pose)
        if (gradient == 0):
            gradient = 0.0001
        new_line_trajectory_gradient = -1/gradient
        new_line_crossover = self.nextWayPoint.position.y - new_line_trajectory_gradient*self.nextWayPoint.position.x

        x_current_pose = new_line_trajectory_gradient*self.current_pose.linear.y + new_line_crossover

        error_x1 = abs(self.current_pose.linear.x) - abs(x_current_pose)
        #self.error_x = np.sqrt(pow(self.nextWayPoint.position.x - self.current_pose.linear.x,2) + pow(self.nextWayPoint.position.y - self.current_pose.linear.y,2) )
        return (error_x1,error_y1)
    
    def angle_check(self,angle):
        
        if angle > MATH.pi:
            angle -= 2*MATH.pi
        elif angle < -MATH.pi:
            angle += -2*MATH.pi
        
        return angle
    
    def line_generate(self,point1x,point1y,point2x,point2y):
        p_x = point2x - point1x
        if ((point2x - point1x) == 0 or (point2y - point1y) == 0):
            p_x = 1

        gradient = (point2y - point1y)/p_x
        crossover = point2y - gradient*point2x

        return (gradient,crossover)

    def referenceCallback(self,msg):
        self.currentWayPointArray = msg
        #self.init_pose = self.currentWayPointArray.poses[0]
        


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "next_waypoint"
    rospy.init_node(node_name)
    template_py_node = WaypointTracking()
    # Spin as a single-threaded node
    rospy.spin()
