#!/usr/bin/env python
import rospy

from asclinic_pkg.msg import LeftRightFloat32
from sensor_msgs.msg import Joy



class joy2MotorNode:
    def __init__(self):
        self.node_name = "joy_motor_input"

        self.joy_topic = "joy"
        self.motor_topic = "/set_motor_duty_cycle"

        self.pub = rospy.Publisher(self.motor_topic, LeftRightFloat32, queue_size=1)
        self.sub = rospy.Subscriber(self.joy_topic, Joy, self.talk2Motor)

        self.listener()


    def talk2Motor(self, joy_msg):
        l_trig = self.joy2LeftTrigger(joy_msg)
        r_trig = self.joy2RightTrigger(joy_msg)
        l_stick = self.joy2LeftJoyHoriz(joy_msg)
        [scaled_left, scaled_right] = self.joyInputs2MotorSpeeds(l_trig, r_trig, l_stick)
        motor_msg = LeftRightFloat32()
        motor_msg.left = scaled_left
        motor_msg.right = scaled_right
        rospy.loginfo("left out: "+str(motor_msg.left))
        rospy.loginfo("right out: "+str(motor_msg.right))
        
        self.pub.publish(motor_msg)

    def listener(self):
        rospy.init_node(self.node_name, anonymous=True)
        
        rospy.spin()
            
    def joyInputs2MotorSpeeds(self, trigger_right: float, trigger_left: float, left_joy_horiz: float):
        forward_vel = trigger_right - trigger_left
        left_vel = forward_vel + left_joy_horiz
        right_vel = forward_vel - left_joy_horiz

        norm_left_vel = left_vel
        norm_right_vel = right_vel

        if(abs(left_vel)>1 or abs(right_vel)>1):
            scaling_factor = max(abs(left_vel), abs(right_vel))
            norm_left_vel = left_vel/scaling_factor
            norm_right_vel = right_vel/scaling_factor

        scaled_left = norm_left_vel * 30
        scaled_right = norm_right_vel * 30

        return [scaled_left, scaled_right]

    def joy2LeftTrigger(self, joy_msg):
        return joy_msg.axes[2]

    def joy2RightTrigger(self, joy_msg):
        return joy_msg.axes[5]

    def joy2LeftJoyHoriz(self, joy_msg):
        return joy_msg.axes[0]



if __name__ == '__main__':
    nd = joy2MotorNode()





    
