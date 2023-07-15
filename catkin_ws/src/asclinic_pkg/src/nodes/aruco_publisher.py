#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseArray,Pose 

class arucoPublisher:

    def __init__(self):
        # Initialise a publisher
        self.pose_publisher = rospy.Publisher("/asc"+"/all_aruco_id", PoseArray, queue_size=10)
        
        self.ArucoStored = PoseArray()
        
        newID1 = [1,12.435,0.0,270,1]
        #newID2 = [2,4.46,9.1,90,0]
        newID2 = [2,20.005,8.01,180,1]
        newID3 = [3,14.235,1.55,180,1]
        newID4 = [4,20.035,1.56,180,1]
        newID5 = [5,0.0,0.0,180,0]
        newID6 = [6,1.0,1.0,-90,0]
        newID7 = [7,18.805,9.62,90,1]
        newID8 = [8,3.0,3.0,180,0]
        newID9 = [9,0.0,0.0,180,0]
        newID10 = [10,19.605,6.31,180,1]
        #newID11 = [11,5.96,8.6,210,0]
        newID11 = [11,12.945,9.67,90,1]
        newID12 = [12,15.562,1.56,0,1]
        newID13 = [13,15.965,9.72,90,1]
        newID14 = [14,14.365,9.7,90,1]
        newID15 = [15,14.375,5.06,210,0]
        newID16 = [18,14.555,5.06,330,1]
        newID17 = [17,9.4,16.54,180,0]   # Do not use marker 17
        newID18 = [18,14.555,5.06,330,0]
        newID19 = [19,15.387,2.02,270, 1]
        newID20 = [20,20.005,8.01,180,0]
        newID21 = [21,0.0,0.0,180,0]
        newID22 = [22,10.165,9.08,0,1]
        newID23 = [23,13.965,1,180,0]
        newID24 = [24,19.605,3.81,180,1]
        newID25 = [25,10.205,1.08,0,0]
        #newID26 = [26,14.465,4.92,90,0]
        newID26 = [26,20.035,1.56,180,1]
        newID27 = [27,10.005,4.13,0,1]
        newID28 = [28,10.205,1.08,0,0]
        newID29 = [29,18.735,0.48,270,1]
        newID30 = [30,10.205,1.08,0,1]

        for i in range(1,31):
            # Do not use marker 17
            #if i != 17:
                self.marker_storage(locals()["newID"+str(i)])
        
        self.counter = 0
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing)
       


    def marker_storage(self,ID):
        ArucoPosition = Pose()
        ArucoPosition.position.x = ID[1]
        ArucoPosition.position.y = ID[2]
        ArucoPosition.position.z = ID[3]
        ArucoPosition.orientation.w = ID[0]
        ArucoPosition.orientation.y = ID[4]

        self.ArucoStored.poses.append(ArucoPosition)

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        self.counter += 1
        self.pose_publisher.publish(self.ArucoStored)

    

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "aruco_publisher"
    rospy.init_node(node_name)
    template_py_node = arucoPublisher()
    # Spin as a single-threaded node
    rospy.spin()
