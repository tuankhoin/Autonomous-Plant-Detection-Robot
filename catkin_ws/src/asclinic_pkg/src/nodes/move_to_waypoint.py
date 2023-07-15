#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32

from enum import Enum
 
class State(Enum):
    WAYPOINT_FINDING = 1
    PLANT_FINDING = 2

# Maximum linear error to the waypoint allowed. 
# Robot has to be within this radius of the waypoint to have "arrived"
MAX_LINEAR_ERROR_METRES = 0.1

# Maximum angular error to the waypoint allowed. 
# Robot has to be within this angle of the waypoint to have "arrived"
MAX_ANGLE_ERROR_DEG = 5

# Constant of proportionality (multiplier) to go from linear displacement to linear velocity v
LIN_DISP_TO_VEL_MULTIPLIER = 0.1

# Constant of proportionality (multiplier) to go from angular displacement to angular velocity w
ANG_DISP_TO_VEL_MULTIPLIER = 0.65

# Convert angle phi (radians) to angle in the range -pi to pi
def convertAngleRange(phi):
    # Range from 0 to 2*pi
    if (phi < 0):
        new_phi  = (360 - math.fmod((abs(phi)/(math.pi))*180, 360.0)) * math.pi/180
    else:
        new_phi  = (math.fmod((phi/(math.pi))*180, 360.0) * math.pi/180)
    
    # Range from -pi (exclusive) to pi (inclusive)
    if (new_phi <= math.pi):
        new_phi = new_phi
    elif (new_phi > math.pi):
        new_phi = new_phi - 2*math.pi

    return new_phi

class MoveToWaypoint:

    def __init__(self):
        # Initialise a timer
        self.counter = 0
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing)
        # State of the robot (WAYPOINT_FINDING or PLANT_FINDING)
        self.robot_state = UInt32()
        # Initialise goal pose twist message
        self.goal_pose = Twist()
        # Initialise target robot velocity twist message
        self.target_robot_velocity = Twist()
        # Initialise a publisher
        self.target_robot_velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # Initialise a subscriber
        rospy.Subscriber("next_waypoint", Twist, self.nextWaypointSubscriberCallback)
        rospy.Subscriber("fused_pose", Twist, self.fusedPoseSubscriberCallback)
        #rospy.Subscriber("camera_pose", Twist, self.cameraPoseSubscriberCallback)
        rospy.Subscriber("robot_state", UInt32, self.robotStateSubscriberCallback)

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        self.counter += 1

        if self.counter > 15:
            self.counter = 0

    # Subscriber to robot_state topic
    def robotStateSubscriberCallback(self, msg):
        self.robot_state = msg

    # Subscriber to camera_pose topic
    #def cameraPoseSubscriberCallback(self, msg):
        # # # Publish the linear and angular velocity
        # # if self.robot_state.data == State.WAYPOINT_FINDING.value:
        # #     self.target_robot_velocity_publisher.publish(self.target_robot_velocity)

        # # Publish the linear and angular velocity
        # if self.robot_state.data == State.WAYPOINT_FINDING.value:
        #     if self.counter <= 1:
        #         # Pause the robot
        #         self.target_robot_velocity.linear.x = 0
        #         self.target_robot_velocity.angular.z = 0
        #         self.target_robot_velocity_publisher.publish(self.target_robot_velocity)

    # Subscriber to fused_pose topic
    def fusedPoseSubscriberCallback(self, msg):
        # Goal heading in range (-pi,pi] i.e. the heading the robot needs to face the goal
        goal_heading = math.atan2(self.goal_pose.linear.y - msg.linear.y, self.goal_pose.linear.x - msg.linear.x)
        # Error between the goal heading and current heading (taking into account direction of rotation)
        angle_error = goal_heading - msg.angular.z
        rospy.loginfo(angle_error)
        # Straight line distance between the goal position and current position
        linear_error = math.sqrt(math.pow(msg.linear.x - self.goal_pose.linear.x, 2) + math.pow(msg.linear.y - self.goal_pose.linear.y, 2))

        #---> Rotate with angular velocity to correct the heading unless close enough to the goal position
        if linear_error > MAX_LINEAR_ERROR_METRES:
            # Rotate in the other direction if the rotation angle is less
            if angle_error > math.pi:
                # Smaller angle in the negative direction
                angle_error =  angle_error - 2*math.pi
            elif angle_error < - math.pi:
                # Smaller angle in the positive direction
                angle_error =  2*math.pi - math.fabs(angle_error)       
            # Scale the angular velocity by the angle error
            self.target_robot_velocity.angular.z = ANG_DISP_TO_VEL_MULTIPLIER*(angle_error)

        #---> Drive forward with linear velocity if close enough to the goal heading
        if math.fabs(angle_error) < math.radians(MAX_ANGLE_ERROR_DEG):
            # Scale the linear velocity by the linear error
            self.target_robot_velocity.linear.x = LIN_DISP_TO_VEL_MULTIPLIER*(linear_error)


        # # Publish the linear and angular velocity
        # if self.robot_state.data == State.WAYPOINT_FINDING.value:
        #     self.target_robot_velocity_publisher.publish(self.target_robot_velocity)

        # Publish the linear and angular velocity
        if self.robot_state.data == State.WAYPOINT_FINDING.value:
            if self.counter <= 1:
                # Pause the robot
                self.target_robot_velocity.linear.x = 0
                self.target_robot_velocity.angular.z = 0
                self.target_robot_velocity_publisher.publish(self.target_robot_velocity)
            if self.counter > 1:
                # Move to waypoint
                self.target_robot_velocity_publisher.publish(self.target_robot_velocity)


        # # Drive with linear velocity v if close enough to the goal heading
        # if math.fabs(angle_error) < math.radians(10):
        #     linear_error = math.sqrt(math.pow(msg.linear.x - self.goal_pose.linear.x, 2) + math.pow(msg.linear.y - self.goal_pose.linear.y, 2))
        #     # Scale the linear velocity by the linear error
        #     self.target_robot_velocity.linear.x = 0.05*(linear_error)
        #     # Publish the linear velocity
        #     self.target_robot_velocity_publisher.publish(self.target_robot_velocity)

        # # Otherwise correct heading with angular velocity w
        # else:
        #     angle_error = self.goal_pose.angular.z - msg.angular.z
        #     if angle_error > math.pi:
        #         # Smaller angle in the negative direction
        #         angle_error =  angle_error - 2*math.pi
        #     elif angle_error < - math.pi:
        #         # Smaller angle in the positive direction
        #         angle_error =  2*math.pi - math.fabs(angle_error)       
        #     # Scale the angular velocity by the angle error
        #     self.target_robot_velocity.angular.z = 0.65*(angle_error)
        #     # Publish the angular velocity
        #     self.target_robot_velocity_publisher.publish(self.target_robot_velocity)


    # Subscriber to next_waypoint topic
    def nextWaypointSubscriberCallback(self, msg):
        self.goal_pose.linear.x = msg.linear.x 
        self.goal_pose.linear.y = msg.linear.y
        #self.goal_pose.angular.z = convertAngleRange(msg.angular.z)


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "move_to_waypoint"
    rospy.init_node(node_name)
    # Display the namespace of the node handle
    rospy.loginfo("MOVE_TO_WAYPOINT NODE] namespace of node = " + rospy.get_namespace())
    template_py_node = MoveToWaypoint()
    # Spin as a single-threaded node
    rospy.spin()
