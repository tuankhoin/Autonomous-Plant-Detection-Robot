#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt32
from asclinic_pkg.msg import Path2,LeftRightFloat32,TemplateMessage
from geometry_msgs.msg import PoseArray,Pose,Twist,Vector3
import numpy as np
import math as MATH
from enum import Enum

WHEEL_RADIUS = 0.07176
HALF_WHEEL_BASE = 0.2157/2
LENGTH_ROBOT = 0.6

MAX = 1 #max speed rad/s
MIN = -1
ERRORX_PARAMETER = 0.15
ERRORY_PARAMETER = 0.1
ORIENTATION_THRESHOLD = MATH.pi/4

class Current_State(Enum):
    START = "START"
    NEXT_WAYPOINT = "REQUEST_NEXT"
    MOVING_WAYPOINT = "TRAVERSING"
    FINE_MOVE = "FINE_MOVE"

class Robot_move(Enum):
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    TURNLEFT = "TURN LEFT"
    TURNRIGHT = "TURN RIGHT"
    STOP = "STOP"
    FINELEFT = "FINE LEFT"
    FINERIGHT = "FINE RIGHT"

class WaypointTracking:
    def __init__(self):
        # Initialise a publisher
        self.counter_publisher = rospy.Publisher("/Dummy",Pose,queue_size=3)
        rospy.Timer(rospy.Duration(1.0), self.updatedcountepublishing)
        self.velocity_publisher = rospy.Publisher("/asc/target_velocity/", LeftRightFloat32, queue_size=10)
        

        # Initialise a timer
        #self.counter = 0
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing)
        # Initialise a subscriber
        self.currentWayPointArray = PoseArray()
        self.nextWayPointArray = PoseArray()
        self.currentWayPoint = Pose()
        self.nextWayPoint = Pose()
        self.current_pose = Pose()
        
        self.cmdvel = LeftRightFloat32()

        # self.Kp = 1.1
        # self.Ki = 0.35
        # self.Kd = 0.27

        self.error_x = 0
        self.error_y = 0

        # self.Kpla = 0.12
        # self.Kila = 0.01
        # self.Kdla = 0.48
        # self.T = 0.1
        # self.prev_perpendicular_error = 0
        # self.prev_parallel_error = 0 
        # self.mode = "perpendicular"
        # self.sum_perpendicular_error = 0
        # self.sum_parallel_error = 0
        self.updated_counter = 0
        #self.new_counter = TemplateMessage
        self.end = False

        self.v = 0
        self.w = 0
        self.theta_left = 0
        self.theta_right = 0
        self.pathstate = Current_State.START
        self.robotmove = Robot_move.STOP
        self.init_orientation = Pose()
        self.init_pose = Pose()
        self.new_angle = 0
        self.error_heading = 0
        rospy.Subscriber("/dummy/reference_target", PoseArray, self.referenceCallback)

        #self.current_pose = self.currentWayPointArray.poses[0]
        #self.currentWayPoint = self.currentWayPointArray.poses[self.updated_counter]
        #self.nextWayPoint = self.currentWayPointArray.poses[self.updated_counter+1]

        #rospy.loginfo(self.current_pose)
        rospy.Subscriber("/asc/odom_pose",Twist,self.poseCallback)

        
        
        

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        #self.counter += 1
        self.velocity_publisher.publish(self.cmdvel)

    def updatedcountepublishing(self,event):
        #self.new_counter = self.updated_counter
        rospy.loginfo("CURRENT WAYPOINT")
        rospy.loginfo(self.currentWayPoint)

        rospy.loginfo("NEXT WAYPOINT")
        rospy.loginfo(self.nextWayPoint)

        rospy.loginfo("Current Position")
        rospy.loginfo(self.current_pose)
        rospy.loginfo("THETA LEFT")
        rospy.loginfo(self.cmdvel.left)
        rospy.loginfo("THETA RIGHT")
        rospy.loginfo(self.cmdvel.right)
        rospy.loginfo(self.pathstate)
        rospy.loginfo(self.robotmove)

        rospy.loginfo("HEADING ERROR")
        rospy.loginfo(self.error_heading)
        self.counter_publisher .publish(self.current_pose)
    def poseCallback(self,msg):
        self.init_pose = self.currentWayPointArray.poses[0]
        self.new_twist = Twist()
        self.new_twist = msg

        self.current_pose.position.x = self.new_twist.linear.x + self.init_pose.position.x
        self.current_pose.position.y = self.new_twist.linear.y + self.init_pose.position.y
        self.current_pose.orientation.z =self.angle_check( (self.new_twist.angular.z))

        if (self.updated_counter == 0):
            self.current_pose.orientation.z = (self.angle_check((self.new_twist.angular.z + (self.init_orientation.orientation.z))))
            #rospy.loginfo(self.init_orientation.orientation.z)
            #rospy.loginfo(self.angle_check(np.radians(self.new_twist.angular.z)))

        if (self.updated_counter > len(self.currentWayPointArray.poses)):
            v = 0
            w = 0
            return #Set speeds to zero here
        
        
        self.currentWayPoint = self.currentWayPointArray.poses[self.updated_counter]
        self.nextWayPoint = self.currentWayPointArray.poses[self.updated_counter+1]

        (gradient,crossover) = self.line_generate(self.currentWayPoint.position.x,self.currentWayPoint.position.y,self.nextWayPoint.position.x,self.nextWayPoint.position.y)
        (self.error_x,self.error_y) = self.calculate_error(gradient,crossover)
        #new_angle = self.angle_check(np.arctan2(self.current_pose.position.y - self.nextWayPoint.position.y,self.current_pose.position.x-self.nextWayPoint.position.x))
        
        
        (gradient1,crossover1) = self.line_generate(self.nextWayPoint.position.x,self.nextWayPoint.position.y,self.current_pose.position.x,self.current_pose.position.y)
        alpha = self.angle_check(np.arctan2(self.nextWayPoint.position.y - self.current_pose.position.y,self.nextWayPoint.position.x - self.current_pose.position.x))

        lookahead_dist = np.sqrt(pow(self.nextWayPoint.position.y - self.current_pose.position.y,2)+pow(self.nextWayPoint.position.x - self.current_pose.position.x,2))
        steering_angle = self.angle_check(np.arctan2(2*LENGTH_ROBOT*np.sin(alpha),lookahead_dist))
        self.error_heading = self.heading_error(steering_angle)
        if (((np.abs(self.error_y <= 0.3)) and (np.abs(self.error_x <= 0.1)))):
            # REQUEST NEXT WAYPOINT 
            self.pathstate = Current_State.FINE_MOVE
            #self.robotmove = Robot_move.STOP
        else:
            
            # CONTINUE MOVING 
            if (((self.error_x) > 0.15) and self.pathstate != Current_State.FINE_MOVE):
                self.pathstate = Current_State.MOVING_WAYPOINT
                self.robotmove = Robot_move.FORWARD
            elif (((self.error_x) < -0.15)and self.pathstate != Current_State.FINE_MOVE):
                self.pathstate = Current_State.MOVING_WAYPOINT
                self.robotmove = Robot_move.BACKWARD
            elif (((self.error_y > 0.3) or (self.error_heading > 0.8)) and self.pathstate != Current_State.FINE_MOVE):
                # ROTATE CCW # Rotate RIght
                self.pathstate = Current_State.MOVING_WAYPOINT
                self.robotmove = Robot_move.TURNLEFT
            elif(((self.error_y < -0.3) or (self.error_heading < -0.8))  and self.pathstate != Current_State.FINE_MOVE):
                # ROTATE CW 
                self.pathstate = Current_State.MOVING_WAYPOINT 
                self.robotmove = Robot_move.TURNRIGHT
 
        

        if (self.pathstate == Current_State.FINE_MOVE):
            # your next waypoint at this state is the one your are close to
            # align yourself with the next waypoint heading from this one
            #self.new_angle = self.angle_check(np.arctan2(self.nextWayPoint.position.y - self.current_pose.position.y ,self.nextWayPoint.position.x - self.current_pose.position.x))
            lookahead_dist = np.sqrt(pow(self.nextWayPoint.position.y - self.current_pose.position.y,2)+pow(self.nextWayPoint.position.x - self.current_pose.position.x,2))
            steering_angle = self.angle_check(np.arctan2(2*LENGTH_ROBOT*np.sin(alpha),lookahead_dist))
            self.error_heading = self.heading_error(steering_angle) 
            #rospy.loginfo(error_heading)
            if (abs(self.error_heading) <= 0.8):
                self.pathstate = Current_State.NEXT_WAYPOINT
                self.robotmove = Robot_move.STOP
                self.w = 0
                self.v = 0
            elif (self.error_heading > 0.8):
                self.w = 0.25
                self.v = 0
                self.robotmove = Robot_move.TURNLEFT
            elif (self.error_heading < -0.8):
                self.w = -0.25 
                self.v = 0
                self.robotmove = Robot_move.TURNRIGHT

        if (self.pathstate == Current_State.NEXT_WAYPOINT):
            # Request update
            self.updated_counter = self.updated_counter+1

        if (self.pathstate == Current_State.MOVING_WAYPOINT):
            if ((self.robotmove) == Robot_move.TURNRIGHT):
                # Turn right w positive 
                self.w = 1
                self.v = 0
            elif ((self.robotmove) == Robot_move.TURNLEFT):
                self.w = -1
                self.v = 0
            elif ((self.robotmove) == Robot_move.FORWARD):
                self.v = 1
                self.w = 0
            elif ((self.robotmove) == Robot_move.BACKWARD):
                self.v = -1
                self.w = 0
        
        if ((self.robotmove) == Robot_move.STOP):
             w = 0
             v = 0

        a = (self.v/WHEEL_RADIUS) 
        b = ((HALF_WHEEL_BASE*self.w)/WHEEL_RADIUS)
        #rospy.loginfo(v)
        #rospy.loginfo(w)
        self.theta_left =  a - b
        self.theta_right = a + b
        
        if ((self.theta_left >= MAX)):
            self.theta_left = MAX
        elif (self.theta_left <= MIN):
            self.theta_left = MIN

        if ((self.theta_right >= MAX) ):
            self.theta_right = MAX
        elif (self.theta_right <= MIN):
            self.theta_right = MIN




        self.cmdvel.left = self.theta_left
        self.cmdvel.right= self.theta_right
        


        #rospy.loginfo(self.current_pose)
        #self.current_pose = msg
        #rospy.loginfo(self.current_pose)

        ## Calculate Error


        
        


    def heading_error(self,new_angle):
      
        self.error_heading = self.angle_check(self.currentWayPoint.orientation.z - new_angle)
        #rospy.loginfo("ANGLE ROBOT AND WAYPOINT")
        #rospy.loginfo(new_angle)

        #rospy.loginfo("WAYPOINT1 AND WAYPOINT")
        #rospy.loginfo(self.currentWayPoint.orientation.z)

        #rospy.loginfo("ERROR HEADING")
        #rospy.loginfo(error_heading)
        return self.error_heading
    
    def calculate_error(self,gradient,crossover):
        y_current_pose = gradient*self.current_pose.position.x + crossover

        error_y = self.current_pose.position.y - y_current_pose

        new_line_trajectory_gradient = -1/gradient
        new_line_crossover = self.nextWayPoint.position.y - new_line_trajectory_gradient*self.nextWayPoint.position.x

        x_current_pose = new_line_trajectory_gradient*self.current_pose.position.y + new_line_crossover

        error_x = self.current_pose.position.x - x_current_pose

        return (error_y,error_x)

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
    
    def line_generate(self,point1x,point1y,point2x,point2y):
        p_x = point2x - point1x
        if ((point2x - point1x) == 0):
            p_x = 1
        
        gradient = (point2y - point1y)/p_x
        crossover = point2y - gradient*point2x

        return (gradient,crossover)

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
