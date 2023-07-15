#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32, Bool

from asclinic_pkg.msg import PlantInfo

from enum import Enum
 
class State(Enum):
    WAYPOINT_FINDING = 1
    PLANT_FINDING = 2

class StateController:

    def __init__(self):
        # State of the robot (WAYPOINT_FINDING or PLANT_FINDING)
        self.robot_state = UInt32()
        # Initialise goal pose twist message
        self.goal_pose = Twist()
        # Initialise a publisher
        self.robot_state_publisher = rospy.Publisher("/asc/robot_state", UInt32, queue_size=1, latch=True)
        # Initialise a subscriber
        rospy.Subscriber("/asc/no_plant", Bool, self.fusedPoseSubscriberCallback)
        rospy.Subscriber("/asc/plant_check", PlantInfo, self.nextPlantSubscriberCallback)

        data = UInt32()
        data.data = 0
        self.robot_state_publisher.publish(data)

    def updatingSubscriberCallback(self,msg):
        self.updating = msg.data

    # Subscriber to fused_pose topic
    def fusedPoseSubscriberCallback(self, msg):
            self.robot_state.data = State.WAYPOINT_FINDING.value
            # Publish the robot state
            self.updating = True
            self.robot_state_publisher.publish(self.robot_state)

    # Subscriber to next_plant topic
    def nextPlantSubscriberCallback(self, msg):
        self.robot_state.data = State.PLANT_FINDING.value
        # Publish the robot state
        self.robot_state_publisher.publish(self.robot_state)


if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "state_controller"
    rospy.init_node(node_name)
    # Display the namespace of the node handle
    rospy.loginfo("[STATE_CONTROLLER NODE] namespace of node = " + rospy.get_namespace())
    template_py_node = StateController()
    # Spin as a single-threaded node
    rospy.spin()