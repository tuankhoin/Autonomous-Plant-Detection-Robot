#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt32
from asclinic_pkg.msg import Path2,LeftRightFloat32
from geometry_msgs.msg import PoseArray,Pose,Twist,Vector3
import numpy as np
import math as MATH

WHEEL_RADIUS = 0.07176
HALF_WHEEL_BASE = 0.2157/2

MAX = 1.0
MIN = -1.0
DISTANCE_PARAMETER = 0.15
ORIENTATION_THRESHOLD = MATH.pi/4
class WaypointTracking:

    def __init__(self):
        # Initialise a publisher
        self.counter_publisher = rospy.Publisher("/Current_Waypoint",UInt32,queue_size=3)
        rospy.Timer(rospy.Duration(1.0), self.updatedcountepublishing)
        self.velocity_publisher = rospy.Publisher("/asc/target_velocity/", LeftRightFloat32, queue_size=10)
        

        # Initialise a timer
        #self.counter = 0
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing)
        # Initialise a subscriber
        self.currentWayPointArray = Pose()
        self.nextWayPointArray = Pose()
        self.currentWayPoint = Pose()
        self.nextWayPoint = Pose()
        self.current_pose = Pose()
        
        self.cmdvel = LeftRightFloat32()

        self.Kp = 1.1
        self.Ki = 0.35
        self.Kd = 0.27

        self.Kpla = 0.12
        self.Kila = 0.01
        self.Kdla = 0.48
        self.T = 0.1
        self.prev_perpendicular_error = 0
        self.prev_parallel_error = 0 
        self.mode = "perpendicular"
        self.sum_perpendicular_error = 0
        self.sum_parallel_error = 0
        self.updated_counter = 0
        self.end = False
        
        self.dx1 = 0
        self.dy1 = 0

        self.init_orientation = Pose()
        self.init_pose = Pose()
        rospy.Subscriber("/asc/odom_pose",Twist,self.poseCallback)

        
        rospy.Subscriber("/dummy/reference_target", PoseArray, self.referenceCallback)
        

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        #self.counter += 1
        self.velocity_publisher.publish(self.cmdvel)

    def updatedcountepublishing(self,event):
        self.counter_publisher .publish(self.updated_counter
                                        )
    def poseCallback(self,msg):
        self.init_pose = self.currentWayPointArray.poses[0]
        self.new_twist = Twist()
        self.new_twist = msg

        self.current_pose.position.x = self.new_twist.linear.x + self.init_pose.position.x
        self.current_pose.position.y = self.new_twist.linear.y + self.init_pose.position.y
        self.current_pose.orientation.z = self.new_twist.angular.z 
        if (self.updated_counter == 0):
            self.current_pose.orientation.z = np.radians(self.new_twist.angular.z + self.init_orientation.orientation.z)

        
        if (self.updated_counter > len(self.currentWayPointArray.poses)):
            return 
        
        self.currentWayPoint = self.currentWayPointArray.poses[self.updated_counter]
        self.nextWayPoint = self.currentWayPointArray.poses[self.updated_counter+1]

        if (np.sqrt(pow(self.dx1,2)+pow(self.dy1,2))<= DISTANCE_PARAMETER):
            self.updated_counter = self.updated_counter + 1
        
        if (self.end):
            rospy.loginfo("Robot has complete the path")
            self.cmdvel.left = 0
            self.cmdvel.right = 0
            return

        dx = self.nextWayPoint.position.x - self.currentWayPoint.position.x
        dy = self.nextWayPoint.position.y - self.currentWayPoint.position.y
        
        self.dx1 = self.nextWayPoint.position.x - self.current_pose.position.x
        self.dy1 = self.nextWayPoint.position.y - self.current_pose.position.y
        n = np.asarray([self.dx1,self.dy1])

        dx2 = self.currentWayPoint.position.x - self.current_pose.position.x
        dy2 = self.currentWayPoint.position.y - self.currentWayPoint.position.y
        m = np.asarray([dx2,dy2])

        dist = np.hypot(dy2,dx2)
        orient = self.current_pose.orientation.z

        x_relav = np.cos(orient)*dx2 + np.sin(orient)*dy2
        y_relav = -np.sin(orient)*dx2 + np.cos(orient)*dy2

        perpendicular_error = np.arctan2(y_relav,x_relav)
        #direc = self.turn_direction(n,m)

        #t = np.sqrt(pow(dx,2)+pow(dy,2))
        #if (t == 0):
            #distance_to_next = 0
        #else:
            #distance_to_next = np.abs(((dx*dy2) - (dx2*dy)))/t

        #distance_to_next = np.sqrt(pow(dx1,2)+pow(dy1,2))

        #rospy.loginfo(np.abs(((dx*dy1) - (dx1*dy))))
        #rospy.loginfo(np.sqrt(pow(dx,2)+pow(dy,2)))
        ## Perpendicular Error for Orientation Control
        #perpendicular_error = direc*(distance_to_next)

        
        self.sum_perpendicular_error += perpendicular_error
        perpendicular_changes = self.prev_perpendicular_error - perpendicular_error
        self.prev_perpendicular_error = perpendicular_error

        ## Orietnation Control 
        P_orient = self.Kpla * perpendicular_error
        I_orient = self.Kila * self.sum_perpendicular_error*self.T
        D_orient = self.Kdla * perpendicular_changes

        if (I_orient) > 0.25:
            I_orient = 0.25
        elif (I_orient < -0.25):
            I_orient = -0.25
        
        w = P_orient + I_orient + D_orient

        angle_to_next = MATH.atan2(dy,dx)
        angle_to_next = self.angle_check(angle_to_next)

        #parallel_error = MATH.cos(angle_to_next - self.current_pose.orientation.z) * distance_to_next
        
        parallel_error = np.tanh(x_relav)
        self.sum_parallel_error += parallel_error
        parallel_changes = (self.prev_parallel_error - parallel_error)/self.T
        self.prev_parallel_error = parallel_error

        ## Translation Control 
        P_translate_orient = self.Kp * parallel_error
        I_translate_orient = self.Ki * self.sum_parallel_error*self.T
        D_translate_orient = self.Kd * parallel_changes/self.T

        if (I_translate_orient) > 0.25:
            I_translate_orient = 0.25
        elif (I_translate_orient < -0.25):
            I_translate_orient = -0.25
        
        v = P_translate_orient + I_translate_orient + D_translate_orient

        if abs(perpendicular_error) < ORIENTATION_THRESHOLD:
            self.mode = 'parallel'
            #rospy.loginfo("Rotation")
        else:
            self.mode = 'perpendicular'
            #rospy.loginfo("Translation")
        
        if (self.mode == 'parallel'):
            v = 0
            w = 1
        else:
            v = v
            w = w

        a = (v/WHEEL_RADIUS) 
        b = ((HALF_WHEEL_BASE*w)/WHEEL_RADIUS)
        rospy.loginfo(v)
        rospy.loginfo(w)
        theta_left =  a - b
        theta_right = a + b
        
        if ((theta_left > 1)):
            theta_left = MAX
        elif (theta_left < -1):
            theta_left = MIN

        if ((theta_right > 1) ):
            theta_right = MAX
        elif (theta_right < -1):
            theta_right = MIN
        
        rospy.loginfo(theta_left)
        rospy.loginfo(theta_right)
        self.cmdvel.left = -theta_left
        self.cmdvel.right= theta_right


    def turn_direction(self,n,m):
        cross_prod = n[0]*m[1] - n[1]*m[0]
        if cross_prod >= 0:
            return -1
        return 1 
    def angle_check(self,angle):
        
        if angle > MATH.pi:
            angle -= 2*MATH.pi
        elif angle < -MATH.pi:
            angle += -2*MATH.pi
        
        return angle
    
    def referenceCallback(self,msg):
        self.currentWayPointArray = msg
        #rospy.loginfo("Rnning")
        #rospy.loginfo(self.currentWayPointArray)
        self.nextWayPointArray = self.currentWayPointArray.poses[1:]
        #self.init_pose = self.currentWayPointArray.poses[0]
        #rospy.loginfo(self.nextWayPointArray)
        #self.wayPoint_calculate()


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "trajectory_tracking"
    rospy.init_node(node_name)
    template_py_node = WaypointTracking()
    # Spin as a single-threaded node
    rospy.spin()
