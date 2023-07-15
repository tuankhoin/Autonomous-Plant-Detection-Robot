#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import Bool, Float32, UInt8
from asclinic_pkg.msg import Bbox, PlantInfo, Path2, LeftRightFloat32
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

IMG_SIZE_X = 960#640#1920
IMG_SIZE_Y = 540#360#1080
MAP_X = 10.00
MAP_Y = 10.10
OFFSET_X = 10.105

MAP = '/home/asc07/Testing/map.png'

class Streamer:
    def __init__(self):
        self.cv_bridge = CvBridge()
        # TODO: Publisher/subscribers
        self.image_publisher = rospy.Publisher("object_detection_feed", Image, queue_size=1)
        self.map_publisher = rospy.Publisher("map_feed", Image, queue_size=1)

        self.left_publisher = rospy.Publisher("left_wheel_pwm", Float32, queue_size=1)
        self.right_publisher = rospy.Publisher("right_wheel_pwm", Float32, queue_size=1)

        self.image = None
        self.map = cv2.imread(MAP)#, cv2.IMREAD_GRAYSCALE)

        self.destination = (-1,-1)

        rospy.Subscriber("/asc/camera_image", Image, self.imgCallback)
        rospy.Subscriber("asc/plant_check", PlantInfo, self.plantCallback)
        rospy.Subscriber("asc/human_check", Bbox, self.humanCallback)
        rospy.Subscriber("asc/fused_pose", Twist, self.PoseCallback)
        rospy.Subscriber("/actualPath", Path2, self.pathCallback)
        rospy.Subscriber("next_waypoint", Twist, self.nextWaypointSubscriberCallback)
        rospy.Subscriber("asc/current_motor_duty_cycle", LeftRightFloat32, self.wheelCallback)
        rospy.Timer(rospy.Duration(0.4), self.feedCallback)

        # Log
        rospy.loginfo("[STREAMER] Initialisation complete")

    def wheelCallback(self, data):
        self.left_publisher.publish(data.left)
        self.right_publisher.publish(data.right)

    def imgCallback(self,data):
        '''TODO: docstring
        '''
        self.image = cv2.resize(self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough'),(IMG_SIZE_X,IMG_SIZE_Y))[:,:,::-1]
        self.image = np.ascontiguousarray(self.image, dtype=np.uint8)
        #rospy.loginfo(f"[STREAMER] Img read with shape {self.image.shape}")

    def feedCallback(self,event):
        self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.image))

    def nextWaypointSubscriberCallback(self, msg):
        self.map = cv2.circle(self.map,
                        (int((self.destination[0]-OFFSET_X)*self.map.shape[0]/MAP_X),-int(self.destination[1]*self.map.shape[1]/MAP_Y)), 
                        10, (100,255,100), -1)
        self.destination = msg.linear.x, msg.linear.y

    def pathCallback(self,p):
        for pt in p.path:
            self.map = cv2.circle(self.map,
                                (int((pt.x1-OFFSET_X)*self.map.shape[0]/MAP_X),-int(pt.y1*self.map.shape[1]/MAP_Y)), 
                                10, (100,100,255), -1)
        self.map_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.map))

    def plantCallback(self,info):
        top_left = [int(info.left), int(info.top)]
        bottom_right = [int(info.right), int(info.bottom)]
        cv2.rectangle(self.image, 
                      top_left, 
                      bottom_right, 
                      (255,0,0), 2)
        #rospy.loginfo(f"[STREAMER] Exproting img size {self.image.shape}")
        self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.image))

    def humanCallback(self,info):
        top_left = [int(info.left), int(info.top)]
        bottom_right = [int(info.right), int(info.bottom)]
        cv2.rectangle(self.image, 
                      top_left, 
                      bottom_right, 
                      (60,60,255), 2)
        #rospy.loginfo(f"[STREAMER] Exproting img size {self.image.shape}")
        self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.image))
    
    def PoseCallback(self,pose):
        self.map = cv2.circle(self.map,
                              (int((pose.linear.x-OFFSET_X)*self.map.shape[0]/MAP_X),-int(pose.linear.y*self.map.shape[1]/MAP_Y)), 
                              5, (255,100,100), -1)
        self.map_publisher.publish(self.cv_bridge.cv2_to_imgmsg(self.map))

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "stream"
    rospy.init_node(node_name)
    od = Streamer()
    rospy.spin()
    pass
