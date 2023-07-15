#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Python node to detect ArUco markers in the camera images
#
# ----------------------------------------------------------------------------



# ----------------------------------------------------------------------------
# A FEW USEFUL LINKS ABOUT ARUCO MARKER DETECTION
#
# > This link is most similar to the code used in this node:
#   https://aliyasineser.medium.com/aruco-marker-tracking-with-opencv-8cb844c26628
#
# > This online tutorial provdes very detailed explanations:
#   https://www.pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
#
# > This link is the main Aruco website:
#   https://www.uco.es/investiga/grupos/ava/node/26
# > And this is the documentation as linked on the Aruco website
#   https://docs.google.com/document/d/1QU9KoBtjSM2kF6ITOjQ76xqL7H0TEtXriJX5kwi9Kgc/edit
#
# > This link is an OpenCV tutorial for detection of ArUco markers:
#   https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
#
# > This link is an OpenCV explanation of the "solvePnP" function:
#   https://docs.opencv.org/4.7.0/d5/d1f/calib3d_solvePnP.html
#
# > As starting point for details about Rodrigues representation of rotations
#   https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
#
# ----------------------------------------------------------------------------



# Import the ROS-Python package
import rospy
#import rospkg

# Import the standard message types
from std_msgs.msg import UInt32
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
# Import numpy
import numpy as np

# Import opencv
import cv2

# Import aruco
import cv2.aruco as aruco

import math as MATH
# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge



# DEFINE THE PARAMETERS
# > For the number of the USB camera device
#   i.e., for /dev/video0, this parameter should be 0
USB_CAMERA_DEVICE_NUMBER = 0

# > Properties of the camera images captured
DESIRED_CAMERA_FRAME_HEIGHT = 1080
DESIRED_CAMERA_FRAME_WIDTH = 1920
DESIRED_CAMERA_FPS = 5

#DESIRED_CAMERA_FRAME_HEIGHT = 480
#DESIRED_CAMERA_FRAME_WIDTH = 640
#DESIRED_CAMERA_FPS = 30

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


class ArucoDetector:

    def __init__(self):
        
        # Initialise a publisher for the images
        self.image_publisher = rospy.Publisher("/asc"+"/camera_image", Image, queue_size=10)
        self.pose_publisher = rospy.Publisher("/asc" + "/pose_update",TwistStamped,queue_size=10)
        # Initialise a subscriber for flagging when to save an image
        rospy.Subscriber("/asc"+"/request_save_image", UInt32, self.requestSaveImageSubscriberCallback)
        # > For convenience, the command line can be used to trigger this subscriber
        #   by publishing a message to the "request_save_image" as follows:
        #
        # rostopic pub /asc/request_save_image std_msgs/UInt32 "data: 1" 
        

        # Initialise variables for managing the saving of an image
        self.save_image_counter = 0
        self.should_save_image = False

        # Specify the details for camera to capture from

       # > Put the desired video capture properties into local variables
        self.camera_frame_width  = DESIRED_CAMERA_FRAME_WIDTH
        self.camera_frame_height = DESIRED_CAMERA_FRAME_HEIGHT
        self.camera_fps = DESIRED_CAMERA_FPS

        # > For capturing from a USB camera:
        #   > List the contents of /dev/video* to determine
        #     the number of the USB camera
        #   > If "v4l2-ctl" command line tool is installed then list video devices with:
        #     v4l2-ctl --list-devices
        self.camera_setup = USB_CAMERA_DEVICE_NUMBER

        # > For capture from a camera connected via the MIPI CSI cable connectors
        #   > This specifies the gstreamer pipeline for video capture
        #   > sensor-id=0 for CAM0 and sensor-id=1 for CAM1
        #   > This should work; it is not "optimized"; precise details depend on the camera connected
        #self.camera_setup = 'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1920, height=1080, framerate=12/1, format=MJPG ! nvvidconv flip-method=0 ! video/x-raw, width = 800, height=600, format =BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'

        # Initialise video capture from the camera
        self.cam=cv2.VideoCapture(0)

        # Display the properties of the camera upon initialisation
        # > A list of all the properties available can be found here:
        #   https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
        print("\n[ARUCO DETECTOR] Camera properties upon initialisation:")
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
        print("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
        print("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
        #print("CAP_PROP_HUE :             '{}'".format(self.cam.get(cv2.CAP_PROP_HUE)))
        #print("CAP_PROP_CONVERT_RGB :     '{}'".format(self.cam.get(cv2.CAP_PROP_CONVERT_RGB)))
        #print("CAP_PROP_POS_MSEC :        '{}'".format(self.cam.get(cv2.CAP_PROP_POS_MSEC)))
        #print("CAP_PROP_FRAME_COUNT :     '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_COUNT)))
        print("CAP_PROP_BUFFERSIZE :      '{}'".format(self.cam.get(cv2.CAP_PROP_BUFFERSIZE)))

        # Set the camera properties to the desired values
        # > Frame height and  width, in [pixels]
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_frame_height)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH,  self.camera_frame_width)
        # > Frame rate, in [fps]
        self.cam.set(cv2.CAP_PROP_FPS, self.camera_fps)
        # > Auto focus, [bool: 0=off, 1=on]
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # > Focus absolute, [int: min=0 max=250 step=5 default=0]
        #   0 corresponds to focus at infinity
        self.cam.set(cv2.CAP_PROP_FOCUS, 0)
        # > Buffer size, [int: min=1]
        #   Setting the buffer to zero ensures that we get that
        #   most recent frame even when the "timerCallbackForCameraRead"
        #   function takes longer than (1.self.camera_fps) seconds
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Display the properties of the camera after setting the desired values
        print("\n[ARUCO DETECTOR] Camera properties upon initialisation:")
        print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        print("CV_CAP_PROP_FRAME_WIDTH :  '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
        print("CAP_PROP_FPS :             '{}'".format(self.cam.get(cv2.CAP_PROP_FPS)))
        print("CAP_PROP_FOCUS :           '{}'".format(self.cam.get(cv2.CAP_PROP_FOCUS)))
        print("CAP_PROP_AUTOFOCUS :       '{}'".format(self.cam.get(cv2.CAP_PROP_AUTOFOCUS)))
        print("CAP_PROP_BRIGHTNESS :      '{}'".format(self.cam.get(cv2.CAP_PROP_BRIGHTNESS)))
        print("CAP_PROP_CONTRAST :        '{}'".format(self.cam.get(cv2.CAP_PROP_CONTRAST)))
        print("CAP_PROP_SATURATION :      '{}'".format(self.cam.get(cv2.CAP_PROP_SATURATION)))
        #print("CAP_PROP_HUE :             '{}'".format(self.cam.get(cv2.CAP_PROP_HUE)))
        #print("CAP_PROP_CONVERT_RGB :     '{}'".format(self.cam.get(cv2.CAP_PROP_CONVERT_RGB)))
        #print("CAP_PROP_POS_MSEC :        '{}'".format(self.cam.get(cv2.CAP_PROP_POS_MSEC)))
        #print("CAP_PROP_FRAME_COUNT  :    '{}'".format(self.cam.get(cv2.CAP_PROP_FRAME_COUNT)))
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

        # Note: For OpenCV versions <=4.6, the above functions were:
        #self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        #self.aruco_parameters = aruco.DetectorParameters_create()
        #results = aruco.detectMarkers(current_frame_gray, self.aruco_dict, parameters=self.aruco_parameters)

        # Define the marker corner point in the marker frame
        marker_size_half = 0.5 * MARKER_SIZE
        self.single_marker_object_points = np.array([  \
                [-marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half, marker_size_half, 0.0], \
                [ marker_size_half,-marker_size_half, 0.0], \
                [-marker_size_half,-marker_size_half, 0.0]  \
                ], dtype=np.float32 )

        # Specify the intrinsic parameters of the camera
        # > These parameters could either be hardcoded here;
        # > Or you can load then from a file that you may have
        #   saved during the calibration procedure.
        # > Note the that values hardcoded here may give
        #   meaningless results for your camera

        # Camera parameter using online script (off by 10 cm for every 50 cm)
        #self.intrinic_camera_matrix = np.array( [[1.72800406e+03,0,1.17683136e+03] , [0,1.7304367e+03,7.74611481e+02] , [0,0,1]], dtype=float)
        #self.intrinic_camera_distortion  = np.array( [[0.07947289,-0.24118671 , -0.00167789, 0.00130923, 0.07593051]], dtype=float)
        
        #self.intrinic_camera_matrix = np.array([[1.22873002e+03, 0.00000000e+00, 5.60492400e+02],
                #[0.00000000e+00, 1.22873002e+03 ,8.93162106e+02],
                #[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=float)
        #self.intrinic_camera_distortion  = np.array( [[-0.08933016, 0.04536915, 0.02764684, 0.00206304, -0.01612242, 0.11649558, -0.04873472, 0.00266629]]
                #,dtype=float)
        
        self.intrinic_camera_matrix = np.array( [[ 1419.74, 0, 963.032], [0, 1415, 534.979],[0, 0, 1 ]], dtype=float)
        self.intrinic_camera_distortion  = np.array( [[ 26.0444, 91.5814, -0.00380966, -0.00208639, 4.77382, 26.0752, 89.9129, 9.68585, 0, 0, 0, 0, 0, 0 ]], dtype=float)

        # Camera Parameter Using Paul Script (off by 10 cm)
        #self.intrinic_camera_matrix = np.array( [[1.72800396e+03,0,1.17683140e+03] , [0,1.730443664e+03,7.74611469e+02] , [0,0,1]], dtype=float)
        #self.intrinic_camera_distortion  = np.array( [[ 0.07947315, -0.24118757, -0.00167788, 0.00130928, 0.07594123]], dtype=float)
        
        #self.intrinic_camera_matrix = np.array( [[1.99678469e+03,0,1.07683136e+03] , [0,1.92957938e+03,7.9589064e+02] , [0,0,1]], dtype=float)
        #self.intrinic_camera_distortion  = np.array( [[ 0.19293941, -0.1818269, -0.00177126, 0.08391974, 0.1782458]], dtype=float)


        # Initial Camera Value
        #self.intrinic_camera_matrix = np.array( [[1726,0,1107] , [0,1726,788] , [0,0,1]], dtype=float)
        #self.intrinic_camera_distortion  = np.array( [[ 5.5252e-02, -2.3523e-01, -1.0507e-04, -8.9834e-04, 2.4028e-01]], dtype=float)

        # Read the a camera frame as a double check of the properties
        # > Read the frame
        return_flag , current_frame = self.cam.read()
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

    # Respond to timer callback
    def timerCallbackForCameraRead(self, event):
        # Read the camera frame
        #rospy.loginfo("[ARUCO DETECTOR] Now reading camera frame")
        return_flag , current_frame = self.cam.read()
        # Note: return_flag is false if no frame was grabbed

        # get dimensions of image
        #dimensions = current_frame.shape
        #height = current_frame.shape[0]
        #width = current_frame.shape[1]
        #channels = current_frame.shape[2]

        # Check if the camera frame was successfully read
        if (return_flag == True):
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
                    # > Optionally use flags to specify solve method:
                    #   > SOLVEPNP_ITERATIVE Iterative method is based on a Levenberg-Marquardt optimization. In this case the function finds such a pose that minimizes reprojection error, that is the sum of squared distances between the observed projections "imagePoints" and the projected (using cv::projectPoints ) "objectPoints". Initial solution for non-planar "objectPoints" needs at least 6 points and uses the DLT algorithm. Initial solution for planar "objectPoints" needs at least 4 points and uses pose from homography decomposition.
                    #   > SOLVEPNP_P3P Method is based on the paper of X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang "Complete Solution Classification for the Perspective-Three-Point Problem". In this case the function requires exactly four object and image points.
                    #   > SOLVEPNP_AP3P Method is based on the paper of T. Ke, S. Roumeliotis "An Efficient Algebraic Solution to the Perspective-Three-Point Problem". In this case the function requires exactly four object and image points.
                    #   > SOLVEPNP_EPNP Method has been introduced by F. Moreno-Noguer, V. Lepetit and P. Fua in the paper "EPnP: Efficient Perspective-n-Point Camera Pose Estimation".
                    #   > SOLVEPNP_IPPE Method is based on the paper of T. Collins and A. Bartoli. "Infinitesimal Plane-Based Pose Estimation". This method requires coplanar object points.
                    #   > SOLVEPNP_IPPE_SQUARE Method is based on the paper of Toby Collins and Adrien Bartoli. "Infinitesimal Plane-Based Pose Estimation". This method is suitable for marker pose estimation. It requires 4 coplanar object points defined in the following order:
                    #         point 0: [-squareLength / 2, squareLength / 2, 0]
                    #         point 1: [ squareLength / 2, squareLength / 2, 0]
                    #         point 2: [ squareLength / 2, -squareLength / 2, 0]
                    #         point 3: [-squareLength / 2, -squareLength / 2, 0]
                    #   > SOLVEPNP_SQPNP Method is based on the paper "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem" by G. Terzakis and M.Lourakis. It requires 3 or more points.
                    solvepnp_method = cv2.SOLVEPNP_IPPE_SQUARE
                    success_flag, rvec, tvec = cv2.solvePnP(self.single_marker_object_points, corners_of_this_marker, self.intrinic_camera_matrix, self.intrinic_camera_distortion, flags=solvepnp_method)

                    # Note: the aruco.estimatePoseSingleMarkers" function was deprecated in OpenCV version 4.7
                    # > The recommended alternative "cv2.solvePnP" is used above.
                    #this_rvec_estimate, this_tvec_estimate, _objPoints = aruco.estimatePoseSingleMarkers(corners_of_this_marker, MARKER_SIZE, self.intrinic_camera_matrix, self.intrinic_camera_distortion)
                    #rvec = this_rvec_estimate[0]
                    #tvec = this_tvec_estimate[0]

                    # Draw the marker's axes onto the image
                    current_frame_with_marker_outlines = cv2.drawFrameAxes(current_frame_with_marker_outlines, self.intrinic_camera_matrix, self.intrinic_camera_distortion, rvec, tvec, 0.5*MARKER_SIZE)
                    
                    # At this stage, the variable "rvec" and "tvec" respectively
                    # describe the rotation and translation of the marker frame
                    # relative to the camera frame, i.e.:
                    # tvec - is a vector of length 3 expressing the (x,y,z) coordinates
                    #        of the marker's center in the coordinate frame of the camera.
                    # rvec - is a vector of length 3 expressing the rotation of the marker's
                    #        frame relative to the frame of the camera.
                    #        This vector is an "axis angle" representation of the rotation.

                    # Compute the rotation matrix from the rvec using the Rodrigues
                    Rmat = cv2.Rodrigues(rvec)

                    # A vector expressed in the maker frame coordinates can now be
                    # rotated to the camera frame coordinates as:
                    #[x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}

                    # Note: the camera frame convention is:
                    # > z-axis points along the optical axis, i.e., straight out of the lens
                    # > x-axis points to the right when looking out of the lens along the z-axis
                    # > y-axis points to the down  when looking out of the lens along the z-axis
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
                # Display that no aruco markers were found
                #rospy.loginfo("[ARUCO DETECTOR] No markers found in this image")
                # Set the frame variable that is used for save/display/publish
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
