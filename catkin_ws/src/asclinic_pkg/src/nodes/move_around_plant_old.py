#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32, UInt32, UInt8
from geometry_msgs.msg import Twist
from asclinic_pkg.msg import Bbox, PlantInfo, ServoPulseWidth

IMG_SIZE_X = 960#640#1920
IMG_SIZE_Y = 540#360#1080
PIXEL_THRESH = 100
# SIZE_X_THRESH = 300
# SIZE_Y_THRESH = 500

FW_SPEED = 0.12# m/s
ROT_SPEED = 0.09
SERVO_SPEED = 2
INIT_SERVO_POS = 1500

MAX_HEIGHT = 0.9
MAX_POT_HEIGHT = 0.7
GOOD_SIZE = (IMG_SIZE_Y*i for i in (MAX_HEIGHT,0.8,MAX_POT_HEIGHT,0.6,0.55,0.3,0.2))

# function to check object proximity
edgy = lambda r: r.top < PIXEL_THRESH or IMG_SIZE_Y - r.bottom < PIXEL_THRESH
# proximity = lambda r: (r.size_x > SIZE_X_THRESH or r.size_y > SIZE_Y_THRESH) and edgy(r)

in_size = lambda i: any([abs(i-n) < PIXEL_THRESH for n in GOOD_SIZE])

class PlantMover:

    def __init__(self):
        self.servo = INIT_SERVO_POS
        self.run = False
        # Initialise a publisher
        self.cam_publisher = rospy.Publisher("/asc/request_save_image", UInt32, queue_size = 1)
        self.finish_publisher = rospy.Publisher("photos_taken", Bool, queue_size = 1)
        self.occuplant_publisher = rospy.Publisher("update_occuplant", UInt8, queue_size = 1)
        self.move_publisher = rospy.Publisher("cmd_vel", Twist, queue_size = 2)
        self.servo_publisher = rospy.Publisher("set_servo_pulse_width", ServoPulseWidth, queue_size = 5)
        self.state_publisher = rospy.Publisher("no_plant", Bool, queue_size = 1)
        self.delay_publisher = rospy.Publisher("updating_plant", Bool, queue_size = 1)
        # Initialise a subscriber
        rospy.Subscriber("/asc/robot_state", UInt32, self.stateCallback)
        rospy.Subscriber("/asc/plant_check", PlantInfo, self.plantCallback)

        self.move_publisher.publish(Twist()) # Reset
        rospy.loginfo("[PLANT CHECKER] Initialisation complete")

    def stateCallback(self,state):
        self.run = state.data == 2

    # Respond to subscriber receiving a message
    def plantCallback(self, info):
        # self.cam_publisher.publish(69)
        if self.run:
            rospy.loginfo(f"[PLANT CHECKER] Pot detected. Status: {'Plant missing/not found' if info.type=='pot' else 'Plant detected'}")
            limit = MAX_POT_HEIGHT if info.type=='pot' else MAX_HEIGHT
            # If bbox is aligned in middle frame
            if abs(info.offset_x) < PIXEL_THRESH:
                # If within a specific size, take a pic
                if in_size(info.size_y): self.cam_publisher.publish(69)
                # If plant is reached close enough
                if (info.size_y > limit*IMG_SIZE_Y):
                    self.delay_publisher.publish(True)
                    rospy.loginfo("[PLANT CHECKER] Plant reached")
                    # Take a pic
                    self.cam_publisher.publish(69)
                    # Update zone status. Assign a bigger number if missing plant to indicate status
                    extra = 70 if info.type=='pot' else 0
                    self.occuplant_publisher.publish(info.zone+extra)
                    # Servo to initial position
                    self.servo = INIT_SERVO_POS
                    sv = ServoPulseWidth(4,self.servo)
                    self.servo_publisher.publish(sv)
                    # Tell state controller that we are out of plant mode
                    self.finish_publisher.publish(True)
                    # Aaaand... Stop!
                    speed = Twist()
                    self.move_publisher.publish(speed)
                    self.state_publisher.publish(True)
                else:
                    rospy.loginfo("[PLANT CHECKER] Approaching plant")
                    # Move forward
                    speed = Twist()
                    speed.linear.x = FW_SPEED
                    self.move_publisher.publish(speed)
            else:
                # Rotate robot, do alignment
                if info.offset_x < 0:
                    rospy.loginfo(f'[PLANT CHECKER] Offset: {info.offset_x:.2f} pixels. Rotating left')
                    speed = Twist()
                    speed.angular.z = ROT_SPEED
                    self.move_publisher.publish(speed)
                else:
                    rospy.loginfo(f'[PLANT CHECKER] Offset: {info.offset_x:.2f} pixels. Rotating right')
                    speed = Twist()
                    speed.angular.z = -ROT_SPEED
                    self.move_publisher.publish(speed)
            if abs(info.offset_y) > PIXEL_THRESH:
                # TODO: Rotate servo
                if info.offset_y < 0:
                    rospy.loginfo(f'[PLANT CHECKER] Offset: {info.offset_y:.2f} pixels. Servo up')
                    self.servo += SERVO_SPEED
                    sv = ServoPulseWidth(4,self.servo)
                    self.servo_publisher.publish(sv)
                else:
                    rospy.loginfo(f'[PLANT CHECKER] Offset: {info.offset_y:.2f} pixels. Servo down')
                    self.servo -= SERVO_SPEED
                    sv = ServoPulseWidth(4,self.servo)
                    self.servo_publisher.publish(sv)

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "move_around_plant"
    rospy.init_node(node_name)
    node = PlantMover()
    # Spin as a single-threaded node
    rospy.spin()