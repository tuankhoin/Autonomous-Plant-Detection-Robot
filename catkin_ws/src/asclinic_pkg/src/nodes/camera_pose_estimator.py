#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import UInt32
from geometry_msgs.msg import TwistStamped,PoseArray,Vector3, TransformStamped,Twist
import numpy as np
import math as MATH
import tf2_ros
from tf.transformations import quaternion_from_euler


import cv2 as CV
MARKER_CENTRE_HEIGHT_WORLD_FRAME = 0.202

DICT_ARUCO = {}

ARUCO_ORIENTATION = []

ARUCO_POS = []

NEW_ORIENT = []
FLAG = []
class CameraPoseEstimation:

    def __init__(self):
        # Initialise a publisher
        self.PoseAruco = PoseArray()
        self.CommonArucoID = PoseArray()
        #self.world_msg = Vector3()
        self.world_msg = Twist()
        self.PlantArucoID = PoseArray()
        rospy.Subscriber("/asc"+"/all_aruco_id", PoseArray, self.ArucoIDSubscriberCallback)
        rospy.Subscriber("/asc"+"/pose_update",TwistStamped,self.PoseArraySubsriberCallback)
        #self.world_frame_publisher = rospy.Publisher("camera"+"/camera_pose", Vector3, queue_size=10)
        self.world_frame_publisher = rospy.Publisher("/asc/camera_pose", Twist, queue_size=10)
        # Initialise a timer
        self.counter = 0
        self.orientation = 0
        #rospy.Timer(rospy.Duration(0.1), self.timerCallbackForPublishing)
        # Initialise a subscriber
        
    
    # Respond to timer callback
    #def timerCallbackForPublishing(self, event):
    #    self.counter += 1
    #    self.world_frame_publisher.publish(self.world_msg)

    # Respond to subscriber receiving a message
    def ArucoIDSubscriberCallback(self, msg):
        self.CommonArucoID = msg.poses
        # Convert 180 degrees around y-axis to quaternion

        for i in range(len(self.CommonArucoID)):
            r = self.CommonArucoID[i]
            self.orientation = MATH.radians(r.position.z)
            FLAG.append(r.orientation.y)
            #T_adash_a = [[MATH.cos(orientation),0, -MATH.sin(orientation),0],[0,1,0,0],[MATH.sin(orientation),0, MATH.cos(orientation),0],[0,0,0,1]]
            #T_a_odash = [[MATH.cos(MATH.pi/2), MATH.sin(MATH.pi/2), 0,0],[-MATH.sin(MATH.pi/2), MATH.cos(MATH.pi/2), 0,0],[0,0,1,0],[0,0,0,1]]
            T_a_adash = np.array([[MATH.cos(self.orientation),0, MATH.sin(self.orientation)],[0,1,0],[-MATH.sin(self.orientation),0, MATH.cos(self.orientation)]])
            T_odash_a = np.array([[MATH.cos(MATH.pi/2), -MATH.sin(MATH.pi/2), 0],[MATH.sin(MATH.pi/2), MATH.cos(MATH.pi/2), 0],[0,0,1]])
            T_w_odash = np.array([[MATH.cos(MATH.pi/2),0, MATH.sin(MATH.pi/2)],[0,1,0],[-MATH.sin(MATH.pi/2),0, MATH.cos(MATH.pi/2)]])
            T_w_adash = np.matmul(T_a_adash,np.matmul(T_odash_a,T_w_odash))
            DICT_ARUCO[i] = np.transpose(T_w_adash)
            ARUCO_ORIENTATION.append(r.position.z)
            #rospy.loginfo(r.position.x)
            #rospy.loginfo(r.position.y)
            ARUCO_POS.append([r.position.x,r.position.y])
            NEW_ORIENT.append((r.position.z))
     
        #rospy.loginfo(DICT_ARUCO[int(2)])

        

    def PoseArraySubsriberCallback(self,msg):
        #We have the position of Aruco ID relative to world frame 
        # we also have the position of Aruco ID relative to camera 
        # we need to find camera relative to world frame through robot and 
        new_Pose = TwistStamped()
        new_Pose.twist.angular.x = msg.twist.angular.x
        new_Pose.twist.angular.y = msg.twist.angular.y
        new_Pose.twist.angular.z = msg.twist.angular.z
        rvec = [[msg.twist.angular.x],[msg.twist.angular.y],[msg.twist.angular.z]]
        tvec = [[msg.twist.linear.x],[msg.twist.linear.y],[msg.twist.linear.z]]
        rvec = np.array([msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z])
        
        #rospy.loginfo(rvec)

        #[tvec_new_x,tvec_new_y,tvec_new_z] = tvec + Rmat*tvec
        #rospy.loginfo([tvec_new_x,tvec_new_y,tvec_new_z])


        
        #rospy.loginfo(rmat)
        #ypr = CV.RQDecomp3x3(rmat)
        #rvec_flipped = rvec*-1
        #tvec_flipped = tvec*-1
        Rmat = CV.Rodrigues(rvec)
        rmat = Rmat[0]
        new_str = ''
        for i in range(len(msg.header.frame_id)):
            new_str = new_str + msg.header.frame_id[i]
        aruco_ID = int(new_str[1:len(msg.header.frame_id)-1])
        stop_flag = 0
        if (aruco_ID == 17) or (FLAG[aruco_ID-1] == 0) or (aruco_ID > 31):
            stop_flag = 1

        if (stop_flag == 0):
            #rospy.loginfo(int(aruco_ID))
            P = np.hstack((rmat, tvec))
            euler_angles_degrees = -CV.decomposeProjectionMatrix(P)[6]

            # This is the rotation o the camera relative to the aruco marker
            relative_yaw = euler_angles_degrees[1, 0]
            # This is the equation needed to convert the relaive yaw into a world frame yaw.
            #rover_yaw_WorldFrame = ARUCO_ORIENTATION[int(aruco_ID)] - 180 + relative_yaw
            #rover_yaw_WorldFrame = NEW_ORIENT[int(aruco_ID)] -180 + relative_yaw
            x = np.deg2rad(180)
            T_cam_obj = np.array([[MATH.cos(x),0, -MATH.sin(x)],[0,1,0],[MATH.sin(x),0, MATH.cos(x)]])
            x_dash = np.matmul(T_cam_obj,euler_angles_degrees)
            relative_yaw = x_dash[1, 0]
            rover_yaw_WorldFrame = (NEW_ORIENT[int(aruco_ID)-1] +180 + relative_yaw)
            rospy.loginfo(NEW_ORIENT[int(aruco_ID)-1])
            rospy.loginfo("ARUCO")
                        #rospy.loginfo(rover_yaw_WorldFrame)
            translation = -CV.decomposeProjectionMatrix(P)[2]
            r_ArucoFrame = [[translation[0]], [
                            translation[1]], [translation[2]]]/translation[3]
                        # position of camera relative to aruco frame
            a_r_homo = [r_ArucoFrame[0][0],r_ArucoFrame[1][0],r_ArucoFrame[2][0]]
            #rospy.loginfo(a_r_homo)
            pos_worldFrame_homo = np.matmul(
                            DICT_ARUCO[int(aruco_ID)-1], a_r_homo)
            pos_worldFrame_XY = [
            pos_worldFrame_homo[0]-ARUCO_POS[int(aruco_ID)-1][0], pos_worldFrame_homo[1]-ARUCO_POS[int(aruco_ID)-1][1]]
            self.world_msg.linear.x = -pos_worldFrame_XY[0] - 0.07
            self.world_msg.linear.y = pos_worldFrame_XY[1]
            self.world_msg.angular.z = (angle_check(np.radians(rover_yaw_WorldFrame)))
            # if (self.world_msg.angular.z == -MATH.pi):
            #     self.world_msg.angular.z = MATH.pi 

            self.world_frame_publisher.publish(self.world_msg)
        else:
            stop_flag = 0
    # Checks if a matrix is a valid rotation matrix.

        # Convert angle phi (radians) to angle in the range -pi to pi

def angle_check(angle):
        
        if angle > MATH.pi:
            angle -= 2*MATH.pi
        elif angle < -MATH.pi:
            angle += -2*MATH.pi
        
        return angle

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

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "camera_pose_estimator"
    rospy.init_node(node_name)
    template_py_node = CameraPoseEstimation()
    # Spin as a single-threaded node
    rospy.spin()
