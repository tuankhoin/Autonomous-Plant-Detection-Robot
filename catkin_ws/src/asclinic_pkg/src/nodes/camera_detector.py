#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
#import rospkg
import torch
import time

from mmdet.apis import inference_detector, init_detector
# Import the standard message types
from std_msgs.msg import UInt32, UInt8
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, Twist
from asclinic_pkg.msg import Bbox, PlantInfo, Occuplant
from collections import defaultdict
# Import numpy
import numpy as np
import cv2
import cv2.aruco as aruco

import math as MATH
from cv_bridge import CvBridge



# DEFINE THE PARAMETERS
# > For the number of the USB camera device
#   i.e., for /dev/video0, this parameter should be 0
USB_CAMERA_DEVICE_NUMBER = 0

# > Properties of the camera images captured

DESIRED_CAMERA_FRAME_HEIGHT = 480
DESIRED_CAMERA_FRAME_WIDTH = 640
DESIRED_CAMERA_FPS = 30

# > For the size of the aruco marker, in meters
MARKER_SIZE = 0.250

# > For where to save images captured by the camera
#   Note: ensure that this path already exists
#   Note: images are only saved when a message is received
#         on the "request_save_image" topic.
SAVE_IMAGE_PATH = "/home/asc07/saved_camera_images/"

# > A flag for whether to display the images captured
SHOULD_SHOW_IMAGES = False

# > A flag for whether to publish the images captured
SHOULD_PUBLISH_CAMERA_IMAGES = False

# > Object detection
CONFIG = '~/Testing/configs/yolo/yolov3_d53_320_273e_coco.py'
WEIGHTS = '~/Testing/yolov3_d53_320_273e_coco-421362b6.pth'

VASE = 75
PLANT = 58
PERSON = 0
CHAIR = 56
CONFIDENCE = 4
CONFIDENT_THERSHOLD = 0.7

PIXEL_THRESH = 20
SIZE_X_THRESH = 300
SIZE_Y_THRESH = 500
STATUS = {1:'Plant found', 0:'Not found yet', -1:'Pot only (Missing plant)'}

# function to detect center alignment
alignment_offset_x = lambda r: (r[0]+r[2])/2 - IMG_SIZE_X/2
alignment_offset_y = lambda r: (r[1]+r[3])/2 - IMG_SIZE_Y/2
size_x = lambda r: r[2]-r[0]
size_y = lambda r: r[3]-r[1]
area = lambda r: size_x(r)*size_y(r)

class ArucoDetector:

    def __init__(self):
        
        # Initialise a publisher for the images
        self.image_publisher = rospy.Publisher("/asc"+"/camera_image", Image, queue_size=10)
        self.pose_publisher = rospy.Publisher("/asc" + "/pose_update",TwistStamped,queue_size=10)
        self.image_publisher = rospy.Publisher("object_detection_feed", Image, queue_size=10)
        self.human_publisher = rospy.Publisher("human_check", Bbox, queue_size=5)
        self.chair_publisher = rospy.Publisher("chair_check", Bbox, queue_size=5)
        self.plant_publisher = rospy.Publisher("plant_check", PlantInfo, queue_size=5)
        self.occuplant_publisher = rospy.Publisher("plant_status", Occuplant, queue_size=1, latch=True)
        #rospy.Timer(rospy.Duration(1/FPS), self.detectCallback)
        rospy.Subscriber("/asc/fused_pose", Twist, self.PoseCallback)
        rospy.Subscriber("update_occuplant", UInt8, self.occuplantCallback)
        rospy.Subscriber("/asc"+"/request_save_image", UInt32, self.requestSaveImageSubscriberCallback)

        self.save_image_counter = 0
        self.should_save_image = False
        self.camera_frame_width  = DESIRED_CAMERA_FRAME_WIDTH
        self.camera_frame_height = DESIRED_CAMERA_FRAME_HEIGHT
        self.camera_fps = DESIRED_CAMERA_FPS
        self.camera_setup = USB_CAMERA_DEVICE_NUMBER

        self.cam=cv2.VideoCapture(0)

        # Object detection + plant check
        self.device = torch.device('cuda:0')
        self.model = init_detector(CONFIG, WEIGHTS, device=self.device)
        self.occuplant = defaultdict(int)
        self.loc = (0,0,0)

        # Display the properties of the camera upon initialisation
        print("\n[CAMERA DETECTOR] Camera properties upon initialisation:")
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
        print("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
        print("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
        print("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # Set the camera properties to the desired values
        # > Frame height and  width, in [pixels]
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_frame_height)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,  self.camera_frame_width)
        self.cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cam.set(cv2.CAP_PROP_FOCUS, 0)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Display the properties of the camera after setting the desired values
        print("\n[ARUCO DETECTOR] Camera properties after initialisation:")
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
        print("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
        print("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
        print("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # The frame per second (fps) property cannot take any value,
        # hence compare the actural value and display any discrepancy
        camera_actual_fps = self.cam.get(cv2.CAP_PROP_FPS)
        if not(camera_actual_fps==self.camera_fps):
            rospy.logwarn("[ARUCO DETECTOR] The camera is running at " + str(camera_actual_fps) + " fps, even though " + str(self.camera_fps) + " fps was requested.")
            rospy.logwarn("[ARUCO DETECTOR] The fps discrepancy is normal behaviour as most cameras cannot run at arbitrary fps rates.")
            rospy.logwarn("[ARUCO DETECTOR] Due to the fps discrepancy, updated the value: self.camera_fps = " + str(camera_actual_fps))
            self.camera_fps = camera_actual_fps

        # Initlaise the OpenCV<->ROS bridge
        self.cv_bridge = CvBridge()

        # Get the ArUco dictionary to use
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Create an parameter structure needed for the ArUco detection
        self.aruco_parameters = aruco.DetectorParameters()
        # > Specify the parameter for: corner refinement
        self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        # Create an Aruco detector object
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_parameters)

        # Define the marker corner point in the marker frame
        marker_size_half = 0.5 * MARKER_SIZE
        self.single_marker_object_points = np.array([  \
                [-marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half,-marker_size_half, 0.0], \
                [-marker_size_half,-marker_size_half, 0.0]  \
                ], dtype=np.float32 )

        # Specify the intrinsic parameters of the camera
        self.intrinic_camera_matrix = np.array( [[ 1419.74, 0, 963.032], [0, 1415, 534.979],[0, 0, 1 ]], dtype=float)
        self.intrinic_camera_distortion  = np.array( [[ 26.0444, 91.5814, -0.00380966, -0.00208639, 4.77382, 26.0752, 89.9129, 9.68585, 0, 0, 0, 0, 0, 0 ]], dtype=float)

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        _ , current_frame = self.cam.read()
        # > Get the dimensions of the frame
        dimensions = current_frame.shape
        # > Display the dimensions
        rospy.loginfo("[ARUCO DETECTOR] As a double check of the camera properties set, a frame captured just now has dimensions = " + str(dimensions))
        # > Also check the values
        if (not(dimensions[0]==self.camera_frame_height) or not(dimensions[1]==self.camera_frame_width)):
            rospy.logerr("[ARUCO DETECTOR] ERROR: frame dimensions do NOT match the desired values.")
            # Update the variables
            self.camera_frame_height = dimensions[0]
            self.camera_frame_width  = dimensions[1]

        # Display the status
        rospy.loginfo("[ARUCO DETECTOR] Initialisation complete")

        # Initialise a timer for capturing the camera frames
        rospy.Timer(rospy.Duration(1/self.camera_fps), self.timerCallbackForCameraRead)

    def yaw_decomposition(self,xa,ya,xb,yb,xc,yc,xd,yd):
        t = abs(ya - yb)
        v = abs(yd - yc)
        S_a = (abs(xa-xd) + abs(xc-xb))/2
        S_i = t
        if (t <= v):
            yaw_angle = abs(MATH.acos(S_a/S_i))
        else:
            yaw_angle = -(abs(MATH.acos(S_a/S_i)))
    
    def PoseCallback(self,pose):
        self.loc = (pose.linear.x, pose.linear.y, pose.angular.z)
    
    def occuplantCallback(self,zone):
        # If plant missing
        if zone.data>69:
            self.occuplant[zone.data-70] = -1
            rospy.loginfo(f"[OBJECT DETECTOR] Occuplant checklist updated: Zone {zone.data-70} = {self.occuplant[zone.data]}")
        else:
            self.occuplant[zone.data] = 1
            rospy.loginfo(f"[OBJECT DETECTOR] Occuplant checklist updated: Zone {zone.data} = {self.occuplant[zone.data]}")
        occ = Occuplant()
        occ.plant_1 = STATUS[self.occuplant[1]]
        occ.plant_2 = STATUS[self.occuplant[2]]
        occ.plant_3 = STATUS[self.occuplant[3]]
        occ.plant_4 = STATUS[self.occuplant[4]]
        occ.plant_5 = STATUS[self.occuplant[5]]
        occ.plant_6 = STATUS[self.occuplant[6]]
        self.occuplant_publisher.publish(occ)

    def get_zone(self,p):
        # TODO: Detect the zone based on alignment_offset_x(p), self.loc
        zone = 1
        return zone, self.occuplant[zone]

    # Respond to timer callback
    def timerCallbackForCameraRead(self, event):
        # Read the camera frame
        #rospy.loginfo("[ARUCO DETECTOR] Now reading camera frame")
        return_flag , current_frame = self.cam.read()

        # Check if the camera frame was successfully read
        if return_flag:
            # Convert the image to gray scale
            current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers from the frame
            aruco_corners_of_all_markers, aruco_ids, aruco_rejected_img_points = self.aruco_detector.detectMarkers(current_frame_gray)

            # Process any ArUco markers that were detected
            if aruco_ids is not None:
                # Display the number of markers found
                if (len(aruco_ids)==1):
                    rospy.loginfo("[ARUCO DETECTOR] Found " + "{:3}".format(len(aruco_ids)) + " marker  with id  = " + str(aruco_ids[:,0]))
                else:
                    rospy.loginfo("[ARUCO DETECTOR] Found " + "{:3}".format(len(aruco_ids)) + " markers with ids = " + str(aruco_ids[:,0]))
                # Outline all of the markers detected found in the image
                current_frame_with_marker_outlines = aruco.drawDetectedMarkers(current_frame.copy(), aruco_corners_of_all_markers, aruco_ids, borderColor=(0, 220, 0))

                # Iterate over the markers detected
                for i_marker_id in range(len(aruco_ids)):
                    new_msg = TwistStamped()
                    # Get the ID for this marker
                    this_id = aruco_ids[i_marker_id]
                    new_msg.header.frame_id = str(this_id)
                    # Get the corners for this marker
                    corners_of_this_marker = np.asarray(aruco_corners_of_all_markers[i_marker_id][0], dtype=np.float32)
                    # Estimate the pose of this marker
                    solvepnp_method = cv2.SOLVEPNP_IPPE_SQUARE
                    success_flag, rvec, tvec = cv2.solvePnP(self.single_marker_object_points, corners_of_this_marker, self.intrinic_camera_matrix, self.intrinic_camera_distortion, flags=solvepnp_method)

                    # Draw the marker's axes onto the image
                    current_frame_with_marker_outlines = cv2.drawFrameAxes(current_frame_with_marker_outlines, self.intrinic_camera_matrix, self.intrinic_camera_distortion, rvec, tvec, 0.5*MARKER_SIZE)
                    Rmat = cv2.Rodrigues(rvec)

                    # A vector expressed in the maker frame coordinates can now be
                    # rotated to the camera frame coordinates as:
                    #[x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}
                    new_msg.twist.linear.x = tvec[0]
                    new_msg.twist.linear.y = tvec[1]
                    new_msg.twist.linear.z = tvec[2]
                    new_msg.twist.angular.x = rvec[0]
                    new_msg.twist.angular.y = rvec[1]
                    new_msg.twist.angular.z = rvec[2]
                    self.pose_publisher.publish(new_msg)

                    # Display the rvec and tvec
                    rospy.loginfo("[ARUCO DETECTOR] for id = " + str(this_id) + ", tvec = [ " + str(tvec[0]) + " , " + str(tvec[1]) + " , " + str(tvec[2]) + " ]" )
                    rospy.loginfo("[ARUCO DETECTOR] for id = " + str(this_id) + ", rvec = [ " + str(rvec[0]) + " , " + str(rvec[1]) + " , " + str(rvec[2]) + " ]" )
                    
                    # ============================================
                    # TO BE FILLED IN FOR WORLD FRAME LOCALISATION
                    # ============================================
                    # Based on the known location and rotation of
                    # marker relative to the world frame, compute
                    # an estimate of the camera's location within
                    # the world frame, and hence an estimate of
                    # robot's pose on which the camera is mounted. 
                    #
                    # ADD YOUR CODE HERE
                    #
                    # PUBLISH THE ESTIMATE OF THE ROBOT'S POSE
                    # FOR USE BY OTHER ROS NODES
                    #
                    # ============================================


            else:
                current_frame_with_marker_outlines = current_frame_gray

            # Publish the camera frame
            if (SHOULD_PUBLISH_CAMERA_IMAGES):
                #rospy.loginfo("[ARUCO DETECTOR] Now publishing camera frame")
                self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame))

            # Save the camera frame if requested
            if (self.should_save_image):
                # Increment the image counter
                self.save_image_counter += 1
                # Write the image to file
                temp_filename = SAVE_IMAGE_PATH + "aruco_image" + str(self.save_image_counter) + ".jpg"
                cv2.imwrite(temp_filename,current_frame_with_marker_outlines)
                # Display the path to where the image was saved
                rospy.loginfo("[ARUCO DETECTOR] Save camera frame to: " + temp_filename)
                # Reset the flag to false
                self.should_save_image = False

            # Display the camera frame if requested
            if (SHOULD_SHOW_IMAGES):
                #rospy.loginfo("[ARUCO DETECTOR] Now displaying camera frame")
                cv2.imshow("[ARUCO DETECTOR]", current_frame_with_marker_outlines)

        else:
            # Display an error message
            rospy.loginfo("[ARUCO DETECTOR] ERROR occurred during \"self.cam.read()\"")



    # Respond to subscriber receiving a message
    def requestSaveImageSubscriberCallback(self, msg):
        rospy.loginfo("[ARUCO DETECTOR] Request received to save the next image")
        # Set the flag for saving an image to true
        self.should_save_image = True



if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "aruco_detector"
    rospy.init_node(node_name)
    aruco_detector_object = ArucoDetector()
    # Spin as a single-threaded node
    rospy.spin()

    # Release the camera
    aruco_detector_object.cam.release()
    # Close any OpenCV windows
    cv2.destroyAllWindows()
