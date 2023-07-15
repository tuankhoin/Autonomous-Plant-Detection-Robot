#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
#from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist
from asclinic_pkg.msg import LeftRightInt32

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

class Kalman:

    def __init__(self):
        # Odometry update constants 
        self.r = 0.07176; # metres
        self.b = 0.2157; # metres
        self.cpr = 1120; # counts per revolution
        # Odometry update
        self.delta_s = 0.0
        self.delta_phi = 0.0
        self.delta_theta_l = 0.0
        self.delta_theta_r = 0.0
        # Initial starting pose
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        self.x = rospy.get_param('~x')
        self.y = rospy.get_param('~y')
        self.phi = rospy.get_param('~phi')
        # Initialise fused pose twist message
        self.fused_pose_msg = Twist()
        # Initialise a publisher
        self.fused_pose_publisher = rospy.Publisher("fused_pose", Twist, queue_size=10)
        # Timed publisher
        rospy.Timer(rospy.Duration(0.1), self.timerCallbackForPublishing)
        # Initialise a subscriber
        rospy.Subscriber("encoder_counts", LeftRightInt32, self.encoderCountsSubscriberCallback)
        rospy.Subscriber("camera_pose", Twist, self.cameraPoseSubscriberCallback)

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        # Retrieve pose message
        self.fused_pose_msg.linear.x  = self.x
        self.fused_pose_msg.linear.y = self.y
        self.fused_pose_msg.angular.z = convertAngleRange(self.phi)
        #self.fused_pose_msg.angular.z = self.phi

        # Publish pose message
        self.fused_pose_publisher.publish(self.fused_pose_msg)

    # Subscriber to camera_pose topic
    def cameraPoseSubscriberCallback(self, msg):
        # Assuming the robot is always on the ground and there is no slip, 
        # do not use camera pose estimate unless robot wheels are not moving
        #if abs(self.delta_theta_l) < 0.05 and abs(self.delta_theta_r) < 0.05: 
        if np.abs(self.delta_theta_l) == 0 and np.abs(self.delta_theta_r) == 0: 
            # Assign to overall pose
            self.x = msg.linear.x
            self.y = msg.linear.y
            self.phi = msg.angular.z
            rospy.loginfo("Using camera_pose")

    # Subscriber to encoder_counts topic
    def encoderCountsSubscriberCallback(self, msg):
        # Convert from left and right wheel encoder to change in pose
        self.delta_theta_l = (float(msg.left)/float(self.cpr))*2.0*math.pi
        self.delta_theta_r = (float(msg.right)/float(self.cpr))*2.0*math.pi
        rospy.loginfo("delta_theta_l")
        rospy.loginfo(self.delta_theta_l)
        rospy.loginfo("delta_theta_r")
        rospy.loginfo(self.delta_theta_r)
        self.delta_s = (self.r/2.0)*(self.delta_theta_l + self.delta_theta_r)
        self.delta_phi = (self.r/self.b)*(self.delta_theta_r - self.delta_theta_l)

        # Add the change in pose to overall pose
        self.x += self.delta_s*math.cos(self.phi + self.delta_phi/2.0)
        self.y += self.delta_s*math.sin(self.phi + self.delta_phi/2.0)
        self.phi += self.delta_phi


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "kalman"
    rospy.init_node(node_name)
    # Display the namespace of the node handle
    rospy.loginfo("KALMAN NODE] namespace of node = " + rospy.get_namespace())
    template_py_node = Kalman()
    # Spin as a single-threaded node
    rospy.spin()
