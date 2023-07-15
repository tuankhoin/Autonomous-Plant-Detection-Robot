#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import torch
import time
import math

from mmdet.apis import inference_detector, init_detector
from std_msgs.msg import UInt8, Bool, UInt32
from asclinic_pkg.msg import Bbox, PlantInfo, Occuplant
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from collections import defaultdict

"""TODO
- occupancy graph for detection (6)
- take pictures upon approach (10)
- adjustment for plants up high (10)
- look on segmentation (0)
"""

CONFIG = '~/Testing/configs/yolo/yolov3_d53_320_273e_coco.py'
WEIGHTS = '~/Testing/yolov3_d53_320_273e_coco-421362b6.pth'

VASE = 75
PLANT = 58
PERSON = 0
CHAIR = 56
CONFIDENCE = 4
CONFIDENT_THERSHOLD = 0.7
CLOSE_ENOUGH = 0.6

FPS = 10
ROT_SPEED = 0.14

IMG_SIZE_X = 960#640#1920
IMG_SIZE_Y = 540#360#1080
PIXEL_THRESH = 100
STATUS = {1:'Plant found', 0:'Not found yet', -1:'Pot only (Missing plant)'}

# function to detect center alignment
alignment_offset_x = lambda r: (r[0]+r[2])/2 - IMG_SIZE_X/2
alignment_offset_y = lambda r: (r[1]+r[3])/2 - IMG_SIZE_Y/2
size_x = lambda r: r[2]-r[0]
size_y = lambda r: r[3]-r[1]
area = lambda r: size_x(r)*size_y(r)

ROOM_X = 10
ROOM_Y = -10.1
move = lambda z,d: 1 if z==2 and d=='left' or z==3 and d=='up' else\
                   2 if z==1 and d=='right' or z==4 and d=='up' else\
                   3 if z==1 and d=='down' or z==3 and d=='left' else\
                   4 if z==2 and d=='down' or z==3 and d=='right' else\
                   z
heading = lambda a: 'right' if a>-math.pi/4 and a<math.pi/4 else \
                    'up' if a>math.pi/4 and a<math.pi*3/4 else \
                    'down' if a>-math.pi*3/4 and a<-math.pi/4 else \
                    'left'
zoner = lambda x,y: 1 if x-10<ROOM_X/2  and y>ROOM_Y/2  else \
                    2 if x-10>=ROOM_X/2 and y>ROOM_Y/2  else \
                    3 if x-10<ROOM_X/2  and y<=ROOM_Y/2 else 4
plant_zoner = lambda z, close, d: z if close else move(z,heading(d))

class ObjectDetector:
    def __init__(self):     
        self.device = torch.device('cuda:0')
        self.model = init_detector(CONFIG, WEIGHTS, device=self.device)
        self.cv_bridge = CvBridge()
        self.occuplant = defaultdict(int)
        self.loc = (0,0,0)
        self.negative_count = 0
        self.state = 0

        # TODO: Publisher/subscribers
        #self.image_publisher = rospy.Publisher("object_detection_feed", Image, queue_size=10)
        self.human_publisher = rospy.Publisher("human_check", Bbox, queue_size=5)
        self.chair_publisher = rospy.Publisher("chair_check", Bbox, queue_size=5)
        self.plant_publisher = rospy.Publisher("plant_check", PlantInfo, queue_size=5)
        self.occuplant_publisher = rospy.Publisher("plant_status", Occuplant, queue_size=1, latch=True)
        self.state_publisher = rospy.Publisher("no_plant", Bool, queue_size = 1)
        self.move_publisher = rospy.Publisher("cmd_vel", Twist, queue_size = 2)
        self.delay_publisher = rospy.Publisher("updating_plant", Bool, queue_size = 1)

        rospy.Subscriber("/asc"+"/camera_image", Image, self.detectCallback)
        rospy.Subscriber("/asc/fused_pose", Twist, self.PoseCallback)
        rospy.Subscriber("update_occuplant", UInt8, self.occuplantCallback)
        rospy.Subscriber("/asc/robot_state", UInt32, self.stateCallback)
        
        occ = Occuplant()
        occ.plant_1 = STATUS[self.occuplant[1]]
        occ.plant_2 = STATUS[self.occuplant[2]]
        occ.plant_3 = STATUS[self.occuplant[3]]
        occ.plant_4 = STATUS[self.occuplant[4]]
        occ.plant_5 = STATUS[self.occuplant[5]]
        occ.plant_6 = STATUS[self.occuplant[6]]
        self.occuplant_publisher.publish(occ)

        # Log
        rospy.loginfo("[OBJECT DETECTOR] Initialisation complete")


    def stateCallback(self,state):
        self.state = state.data

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
        self.delay_publisher.publish(False)

    # def get_zone(self,p):
    #     # TODO: Detect the zone based on alignment_offset_x(p), self.loc
    #     robot_zone = zoner(self.loc[0], self.loc[1])
    #     zone = plant_zoner(robot_zone, size_y(p) > IMG_SIZE_Y*0.3, self.loc[2])
    #     return zone, self.occuplant[zone]
    def get_zone(self,p):
        zone = zoner(self.loc[0], self.loc[1])
        return zone, self.occuplant[zone]

    def detectCallback(self,data):
        '''TODO: docstring
        '''
        last_time = time.time()

        img = cv2.resize(self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='passthrough'),(IMG_SIZE_X,IMG_SIZE_Y))[:,:,::-1]
        result = inference_detector(self.model, img)
        # left_top = (bbox[0], bbox[1])
        # right_bottom = (bbox[2], bbox[3])
        people = [r for r in result[PERSON] if r[CONFIDENCE]>CONFIDENT_THERSHOLD and size_y(r)>IMG_SIZE_Y*CLOSE_ENOUGH]
        chair = [r for r in result[CHAIR] if r[CONFIDENCE]>CONFIDENT_THERSHOLD]
        plant = [r for r in result[PLANT] if r[CONFIDENCE]>CONFIDENT_THERSHOLD and size_y(r)>IMG_SIZE_Y*CLOSE_ENOUGH]
        pots = [r for r in result[VASE] if r[CONFIDENCE]>CONFIDENT_THERSHOLD]

        # current_frame = self.model.show_result(img, result, score_thr=CONFIDENT_THERSHOLD)
        # self.image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(current_frame))

        rospy.loginfo(f'[OBJECT DETECTOR] Result detected (FPS={1/(time.time()-last_time):.2f}): {len(people)} people up close, {len(plant)} potted plants, {len(pots)} empty pots, {len(chair)} chairs')

        potted_plants = [(p,'plant',self.get_zone(p)) for p in plant] + [(p,'pot',self.get_zone(p)) for p in pots]

        # TODO: Subscribe to any information you needed (if needed)
        # TODO: Publish whatever you needed with `result`
        if len(people): 
            bbox = Bbox()
            try:
                largest = max(people, key=area)
                bbox.left, bbox.top, bbox.right, bbox.bottom, bbox.confidence = largest
                self.human_publisher.publish(bbox)
            except:
                pass

        if len(chair): 
            bbox = Bbox()
            try:
                largest = max(people, key=area)
                bbox.left, bbox.top, bbox.right, bbox.bottom, bbox.confidence = largest
                self.chair_publisher.publish(bbox)
            except:
                pass

        if len(potted_plants): 
            try:
                chosen_plant, category, zone = max(potted_plants, key=lambda p: abs(p[2][1])*area(p[0]))
                if zone[1]:
                    rospy.loginfo('[OBJECT DETECTOR] Found an already detected plant!')
                    # In case of delay: If plant detected, revert to trajectory mode
                    # self.negative_count += 1
                    # if self.negative_count > 5:
                    #     self.state_publisher.publish(True)
                    #     self.negative_count = 0
                else:
                    info = PlantInfo()
                    info.left, info.top, info.right, info.bottom, info.confidence = chosen_plant
                    info.offset_x = alignment_offset_x(chosen_plant)
                    info.offset_y = alignment_offset_y(chosen_plant)
                    info.size_x = size_x(chosen_plant)
                    info.size_y = size_y(chosen_plant)
                    info.type = category
                    info.zone = zone[0]
                    if self.state: self.plant_publisher.publish(info)
                    self.negative_count = 0
            except:
                rospy.loginfo('[OBJECT DETECTOR] SUM TING WONG!')
        elif self.state == 2:
            # Rotate if in state but lost track of plant
            speed = Twist()
            speed.angular.z = ROT_SPEED
            self.move_publisher.publish(speed)

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "object_detector"
    rospy.init_node(node_name)
    od = ObjectDetector()
    rospy.spin()
    pass
