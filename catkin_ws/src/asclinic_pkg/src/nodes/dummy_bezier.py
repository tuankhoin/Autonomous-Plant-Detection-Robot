#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt32
import numpy as np
#import matplotlib.pyplot as plt 
import math as MATH
from asclinic_pkg.msg import Path2, TemplateMessage, RrtNode
from geometry_msgs.msg import PoseArray,Pose

class TrajGenNode:

    def __init__(self):

        # Initialise a timer
        self.counter = 1
        #self.subscriber_msg = PoseArray()
        self.new_msg = PoseArray()
        self.current_msg = PoseArray()
        self.pub_msg = PoseArray()
        self.updated_counter =TemplateMessage()
        #rospy.loginfo("Hi")
        #new_twistrospy.Subscriber("/Current_Waypoint",TemplateMessage,self.CounterCallback)
      
        rospy.Subscriber("/actualPath", Path2, self.templateSubscriberCallback)
        self.reference_target = rospy.Publisher("/dummy/reference_target", PoseArray, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing) 
        
        
        # Initialise a publisher

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        #self.counter += 1
        #rospy.loginfo(self.current_msg)
        self.reference_target.publish(self.new_msg)
   
    def CounterCallback(self,msg):
        self.updated_counter = msg.temp_uint32

        rospy.loginfo(self.updated_counter)
    # Respond to subscriber receiving a message
    def templateSubscriberCallback(self, msg):
        #rospy.loginfo((msg.path))
        #self.new_msg.poses = np.empty(len(msg.path),dtype=object)
        #msg.path.poses = reversed(msg.path.poses)
        #rospy.loginfo((self.updated_counter))
        msg.path = (list(reversed(msg.path)))
        #rospy.loginfo(msg.path)
        for i in range(len(msg.path)):
            t = msg.path[i]
            #rospy.loginfo(t)
            if (i+1) == len(msg.path):
                h = RrtNode()
                h.x1 = 0
                h.y1 = 0
            else:
                h = msg.path[i + 1]
            #rospy.loginfo(t)
            trial_msg = Pose()
            trial_msg.position.x = t.x1
            trial_msg.position.y = t.y1
            trial_msg.orientation.w = t.id
            trial_msg.orientation.x = t.parent
            trial_msg.orientation.z = self.angle_check(np.arctan2(h.y1-trial_msg.position.y,h.x1-trial_msg.position.x)) 
            self.new_msg.poses.append(trial_msg)
            #rospy.loginfo(self.new_msg.poses)
            #if (len(self.new_msg.poses) > 1):
                #self.new_msg.poses = np.fliplr(self.new_msg.poses)
        rospy.loginfo(self.new_msg.poses)
        self.pub_msg = self.new_msg
        #rospy.loginfo((self.updated_counter))
        #rospy.loginfo(((self.counter)))
    
    def nextWaypoint(self):
        if (self.counter > len(self.new_msg.poses)):
            #rospy.loginfo(len(self.new_msg.poses))
            return
        else:
            #if (self.updated_counter > len(self.new_msg.poses)):
                #return self.current_msg.poses.append(self.new_msg.poses[0])
            
            self.current_msg.poses.clear()
           
            self.current_msg.poses.append(self.new_msg.poses[self.updated_counter-1])
            self.current_msg.poses.append(self.new_msg.poses[self.updated_counter])

            #rospy.loginfo(self.new_msg.poses[self.updated_counter])

    def angle_check(self,angle):
        
        if angle > MATH.pi:
            angle -= 2*MATH.pi
        elif angle < -MATH.pi:
            angle += -2*MATH.pi
        
        return angle

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "bezier_curve"
    rospy.init_node(node_name)
    template_py_node = TrajGenNode()
    # Spin as a single-threaded node
    rospy.spin()
