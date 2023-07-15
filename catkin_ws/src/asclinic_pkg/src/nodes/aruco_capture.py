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


# This is a customized library by https://www.pyimagesearch.com/
import argparse
# The argparse module makes it easy to write user-friendly command-line interfaces. The program
# defines what arguments it requires, and argparse will figure out how to parse those out of sys.argv. 
# In this code, I don't use this, but you are welcome to implement it

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

from scipy.signal import convolve2d as conv2

from skimage import color, restoration
# Import aruco
import cv2.aruco as aruco

import math as MATH
# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge

# > Properties of the camera images captured
DESIRED_CAMERA_FRAME_HEIGHT = 1080
DESIRED_CAMERA_FRAME_WIDTH = 1920
DESIRED_CAMERA_FPS = 5

#DESIRED_CAMERA_FRAME_HEIGHT = 480
#DESIRED_CAMERA_FRAME_WIDTH = 640
#DESIRED_CAMERA_FPS = 30

# > For the size of the aruco marker, in meters
MARKER_SIZE = 0.250

# define threshold
low_threshold = 30
high_threshold = 1000

class ArucoDetector:

    def __init__(self):
        
        # Initialise a publisher for the images
        rospy.Subscriber("/asc"+"/camera_image", Image, self.timerCallbackForCameraRead)
        self.img_publisher = rospy.Publisher("camera_feed",Image,queue_size=1)
        self.pose_publisher = rospy.Publisher("/asc" + "/pose_update",TwistStamped,queue_size=10)
        self.counter = 0
        self.cv_bridge = CvBridge()

        # Get the ArUco dictionary to use
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        # Create an parameter structure needed for the ArUco detection
        self.aruco_parameters = aruco.DetectorParameters()
        # > Specify the parameter for: corner refinement
        #self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_parameters.cornerRefinementMethod = aruco.CORNER_REFINE_APRILTAG
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

        # Display the status
        rospy.loginfo("[ARUCO DETECTOR] Initialisation complete")

    # Respond to timer callback
    def timerCallbackForCameraRead(self, img):
        # Read the camera frame
        current_frame = self.cv_bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
        self.counter += 1
        if (self.counter == 5):
            self.counter = 0
            
        # h1, w1 = current_frame.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.intrinic_camera_matrix, self.intrinic_camera_distortion, (h1, w1), 0, (h1, w1))
        # dst1 = cv2.undistort(current_frame, self.intrinic_camera_matrix, self.intrinic_camera_distortion, None, newcameramtx)
        # x, y, w1, h1 = roi
        # dst1 = dst1[y:y + h1, x:x + w1]
        # current_frame=dst1


        current_frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        #current_frame_gray = cv2.undistort(current_frame_gray,self.intrinic_camera_matrix,self.intrinic_camera_distortion,None,newcameramtx)
        #fm = self.variance_of_laplacian(current_frame_gray)
        self.flag = UInt32()
        self.flag = 0
        #rospy.loginfo(fm)
        #if fm < low_threshold:
            #self.flag = 1
            #psf = np.ones((5, 5)) / 25
            #astro = conv2(current_frame_gray, psf, 'same')
            #astro_noisy = astro.copy()
            #astro_noisy += (np.random.poisson(lam=25, size=astro.shape) - 10) / 255.
            #current_frame_gray = restoration.richardson_lucy(astro_noisy, psf, 30)
            #self.flag = 0
        #elif fm > high_threshold:
            #self.flag = 1
        #else:
            #self.flag = 0
        

        if (self.flag == 0):
            # Detect ArUco markers from the frame
            aruco_corners_of_all_markers, aruco_ids, _ = self.aruco_detector.detectMarkers(current_frame_gray)
            # Process any ArUco markers that were detected
            if aruco_ids is not None:
                # Display the number of markers found
                #if (len(aruco_ids)==1):
                    #rospy.loginfo("[ARUCO DETECTOR] Found " + "{:3}".format(len(aruco_ids)) + " marker  with id  = " + str(aruco_ids[:,0]))
                #else:
                    #rospy.loginfo("[ARUCO DETECTOR] Found " + "{:3}".format(len(aruco_ids)) + " markers with ids = " + str(aruco_ids[:,0]))
                # Outline all of the markers detected found in the image
                current_frame_with_marker_outlines = aruco.drawDetectedMarkers(current_frame_gray.copy(), aruco_corners_of_all_markers, aruco_ids, borderColor=(0, 220, 0))
                prev_marker = TwistStamped()
                MARKER_DICT = []
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
                    _, rvec, tvec = cv2.solvePnP(self.single_marker_object_points, corners_of_this_marker, self.intrinic_camera_matrix, self.intrinic_camera_distortion, flags=solvepnp_method)

                    # Note: the aruco.estimatePoseSingleMarkers" function was deprecated in OpenCV version 4.7
                    # > The recommended alternative "cv2.solvePnP" is used above.
                    #this_rvec_estimate, this_tvec_estimate, _objPoints = aruco.estimatePoseSingleMarkers(corners_of_this_marker, MARKER_SIZE, self.intrinic_camera_matrix, self.intrinic_camera_distortion)
                    #rvec = this_rvec_estimate[0]
                    #tvec = this_tvec_estimate[0]

                    # (If desired) Draw the marker's axes onto the image
                    current_frame_with_marker_outlines = cv2.drawFrameAxes(current_frame_with_marker_outlines, self.intrinic_camera_matrix, self.intrinic_camera_distortion, rvec, tvec, 0.5*MARKER_SIZE)
                    
                    # At this stage, the variable "rvec" and "tvec" respectively
                    # describe the rotation and translation of the marker frame
                    # relative to the camera frame, i.e.:
                    # tvec - is a vector of length 3 expressing the (x,y,z) coordinates
                    #        of the marker's center in the coordinate frame of the camera.
                    # rvec - is a vector of length 3 expressing the rotation of the marker's
                    #        frame relative to the frame of the camera.
                    #        This vector is an "axis angle" representation of the rotation.

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
                    rospy.loginfo(aruco_ids)
                    if (len(aruco_ids)) > 1 and ([17] not in aruco_ids):
                        MARKER_DICT.append(new_msg)
                    else:
                        self.pose_publisher.publish(new_msg)

                    # Display the rvec and tvec
                    #rospy.loginfo("[ARUCO DETECTOR] for id = " + str(this_id) + ", tvec = [ " + str(tvec[0]) + " , " + str(tvec[1]) + " , " + str(tvec[2]) + " ]" )
                    #rospy.loginfo("[ARUCO DETECTOR] for id = " + str(this_id) + ", rvec = [ " + str(rvec[0]) + " , " + str(rvec[1]) + " , " + str(rvec[2]) + " ]" )
                min_dist = 1000
                pub_msg = TwistStamped()
                actual_msg = TwistStamped()
                if (len(aruco_ids)) > 1:
                    for i_marker_id in range(len(MARKER_DICT)):
                        pub_msg = MARKER_DICT[i_marker_id]
                        dist1 = np.sqrt(MATH.pow(pub_msg.twist.linear.x,2) + MATH.pow(pub_msg.twist.linear.y,2) + MATH.pow(pub_msg.twist.linear.z,2))
                        if (min_dist > dist1):
                            min_dist = dist1
                            actual_msg = pub_msg
                    self.pose_publisher.publish(actual_msg)
        
            else:
                # Display that no aruco markers were found
                #rospy.loginfo("[ARUCO DETECTOR] No markers found in this image")
                # Set the frame variable that is used for save/display/publish
                current_frame_with_marker_outlines = current_frame_gray
            self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame_with_marker_outlines))
    def variance_of_laplacian(self,image):
    # compute the Laplacian of the image and then return the focus
    # measure, which is simply the variance of the Laplacian
        return cv2.Laplacian(image, cv2.CV_64F).var()



if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "aruco_detector"
    rospy.init_node(node_name)
    aruco_detector_object = ArucoDetector()
    # Spin as a single-threaded node
    rospy.spin()
