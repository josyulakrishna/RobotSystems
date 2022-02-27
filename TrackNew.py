#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import threading
import math
import rospy
import numpy as np
from threading import Timer

from std_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import Image

from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.yaml_handle as yaml_handle

from sensor.msg import Led
from warehouse.msg import Grasp
from object_sorting.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import apriltag
from armpi_fpv import bus_servo_control

# getAreaMaxContour- NO CHANGE ACROSS ALL 3 FILES 
#skip move and init() functions a

#Contains Tracking, Sorting and Paletizing 
class Perception: 
    def __init__(self):
        self.AK = ArmIK()
        self.target_color = None
        self.range_rgb = {
    'red': (25, 25, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),}
        self.lab_data = yaml_handle.get_yaml_data("/home/ubuntu/Sensor/HiwonderSDK/lab_config.yaml")
        self.servo_data = []

        self.x_dis = 500
        self.y_dis = 10
        self.Z_DIS = 18
        self.z_dis = self.Z_DIS
        self.x_pid = PID.PID(P=0.1, I=0.00, D=0.008)  # pid初始化 #pid initialization
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.005, I=0, D=0)

        self.get_roi = False
        self.detect_color = 'red'
        self.start_pick_up = False
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.roi = ()
        self.st = True
        self.x_dis = 500
        self.Y_DIS = 0
        self.y_dis = Y_DIS
        self.last_x_dis = x_dis
        self.last_x_dis = y_dis
        self.x_pid = PID.PID(P=0.01, I=0.001, D=0)#pid初始化
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.tag_x_dis = 500
        self.tag_y_dis = 0
        self.tag_x_pid = PID.PID(P=0.01, I=0.001, D=0)#pid初始化
        self.tag_y_pid = PID.PID(P=0.02, I=0, D=0)
        self.stop_state = 0
        self.move_state = 1
        self.adjust = False
        self.approach = False
        self.rotation_angle = 0
        self.start_move = False
        self.adjust_error = False
        self.last_X, last_Y = 0, 0
        self.box_rotation_angle = 0
        self.last_box_rotation_angle = 0
        self.tag1 = ['tag1', -1, -1, -1, 0]
        self.tag2 = ['tag2', -1, -1, -1, 0]
        self.tag3 = ['tag3', -1, -1, -1, 0]
        self.current_tag = ['tag1', 'tag2', 'tag3']
        self.detect_color = ('red', 'green', 'blue')
        self.count = 0
        self.count2 = 0
        self.count3 = 0
        self.count_d = 0
        self.count_timeout = 0
        self.count_tag_timeout = 0
        self.count_adjust_timeout = 0

    def initMove(self):
        #servo_id, pulse = 1500, use_time = 1000
        Board.setPWMServoPulse(1, 500, 800)
        Board.setPWMServoPulse(2, 500, 800)
        self.AK.setPitchRangeMoving((0, self.y_dis, self.z_dis), 0,-90, 0, 1500)
        time.sleep(1.5)
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        areaMaxContour = None

        for c in contours:  # 历遍所有轮廓 #Iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积 #Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰 # Only when the area is greater than 300, the contour with the largest area is valid to filter out interference
                    areaMaxContour = c

        return areaMaxContour, contour_area_max  # 返回最大的轮廓 # Return the largest contour
    def set_rgb(color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()
    ###############TEST RUN FROM TRACKING.PY###########################################
    def run_track(self, img):
        st = self.st 
        roi = self.roi
        rect = self.rect
        get_roi = self.get_roi
        detect_color = self.detect_color
        rotation_angle = self.rotation_angle
        start_pick_up = self.start_pick_up
        # img_h = self.img_h
        # img_w = self.img_w
        x_dis = self.x_dis
        y_dis = self.y_dis
        z_dis = self.z_dis
        size = (640, 480)
        x_pid = PID.PID(P=0.1, I=0.00, D=0.008)  # pid初始化 #pid initialization
        y_pid = PID.PID(P=0.00001, I=0, D=0)
        z_pid = PID.PID(P=0.005, I=0, D=0)
        
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        
        
        
        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间 # Convert the image to LAB space
        
        area_max = 0
        areaMaxContour = 0
        if not start_pick_up:
            for i in self.lab_data:
                #print(i, "\n")
                if i in self.target_color:
                    detect_color = i
                    frame_mask = cv2.inRange(frame_lab,
                                                (self.lab_data[detect_color]['min'][0],
                                                self.lab_data[detect_color]['min'][1],
                                                self.lab_data[detect_color]['min'][2]),
                                                (self.lab_data[detect_color]['max'][0],
                                                self.lab_data[detect_color]['max'][1],
                                                self.lab_data[detect_color]['max'][2]))  #对原图像和掩模进行位运算 #Bit operation on the original image and mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 #open operation
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 #closed operation
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓 #find the outline
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓 #find the largest contour
            if area_max > 1000:  # 有找到最大面积 #have found the largest area
                (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆 Get the smallest circumcircle
                center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
                center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, size[0], 0, img_w))
                if radius > 100:
                    return img
                
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(img, [box], -1, self.range_rgb[self.target_color], 2)
                
                x_pid.SetPoint = img_w / 2.0  # 设定 #set up
                x_pid.update(center_x)  # 当前 # current
                dx = x_pid.output
                x_dis += int(dx)  # 输出 #output

                x_dis = 0 if x_dis < 0 else x_dis
                x_dis = 1000 if x_dis > 1000 else x_dis

                y_pid.SetPoint = 9000  # 设定 #set up
                if abs(area_max - 9000) < 50:
                    area_max = 9000
                y_pid.update(area_max)  # 当前 #current
                dy = y_pid.output
                y_dis += dy  # 输出 #output
                y_dis = 5.00 if y_dis < 5.00 else y_dis
                y_dis = 10.00 if y_dis > 10.00 else y_dis
                
                if abs(center_y - img_h/2.0) < 20:
                    z_pid.SetPoint = center_y
                else:
                    z_pid.SetPoint = img_h / 2.0
                    
                z_pid.update(center_y)
                dy = z_pid.output
                z_dis += dy

                z_dis = 32.00 if z_dis > 32.00 else z_dis
                z_dis = 10.00 if z_dis < 10.00 else z_dis
                
                target = self.AK.setPitchRange((0, round(y_dis, 2), round(z_dis, 2)), -90, 0)
                
                if target:
                    servo_data = target[0]
                    if st:
                        Board.setBusServoPulse(3, servo_data['servo3'], 1000)
                        Board.setBusServoPulse(4, servo_data['servo4'], 1000)
                        Board.setBusServoPulse(5, servo_data['servo5'], 1000)
                        Board.setBusServoPulse(6, int(x_dis), 1000)
                        time.sleep(1)
                        st = False
                    else:
                        Board.setBusServoPulse(3, servo_data['servo3'], 20)
                        Board.setBusServoPulse(4, servo_data['servo4'], 20)
                        Board.setBusServoPulse(5, servo_data['servo5'], 20)
                        Board.setBusServoPulse(6, int(x_dis), 20)
                        time.sleep(0.03)
                    
                    
                        
        return img
 #################################Perception Code from SORTING#####################################   
    def apriltagDetect(self, img):
        tag1 = self.tag1
        tag2 = self.tag2
        tag3 = self.tag3
        detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())    
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray, return_image=False)
        
        tag1 = ['tag1', -1, -1, -1, 0]
        tag2 = ['tag2', -1, -1, -1, 0]
        tag3 = ['tag3', -1, -1, -1, 0]
        if len(detections) != 0:
            for i, detection in enumerate(detections):              
                corners = np.rint(detection.corners) # Get four corners
                cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

                tag_family = str(detection.tag_family, encoding='utf-8') # Get tag_family
                tag_id = int(detection.tag_id) # Get tag_id
                
                object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点   # center point          
                object_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角 
    # Calculate the rotation angle
                
                cv2.putText(img, str(tag_id), (object_center_x - 10, object_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 255, 255], 2)
                
                if tag_id == 1:
                    tag1 = ['tag1', object_center_x, object_center_y, object_angle]
                elif tag_id == 2:
                    tag2 = ['tag2', object_center_x, object_center_y, object_angle]
                elif tag_id == 3:
                    tag3 = ['tag3', object_center_x, object_center_y, object_angle]

    # 获取roi，防止干扰
    # Get roi to prevent interference
    def getROI(self, rotation_angle):
        rotate1 = cv2.getRotationMatrix2D((rows*0.5, cols*0.5), int(rotation_angle), 1)
        rotate_rotate1 = cv2.warpAffine(mask2, rotate1, (cols, rows))
        mask_and = cv2.bitwise_and(rotate_rotate1, mask1)
        rotate2 = cv2.getRotationMatrix2D((rows*0.5, cols*0.5), int(-rotation_angle), 1)
        rotate_rotate2 = cv2.warpAffine(mask_and, rotate2, (cols, rows))
        frame_resize = cv2.resize(rotate_rotate2, (710, 710), interpolation=cv2.INTER_NEAREST)
        roi = frame_resize[40:280, 184:504]
        
        return roi
    
    def color_sort(self, img, target):
        size = (320, 240)
        last_x = 0
        last_y = 0
        state = None
        x_adjust = 0
        pick_color = ''
        # 颜色夹取策略
        # Color clipping strategy
        X= self.X 
        Y=self.Y
        count=self.count
        state=self.state
        adjust=self.adjust
        approach=self.approach
        x_adjust=self.x_adjust 
        pick_color=self.pick_color
        current_tag=self.current_tag
        adjust_error=self.adjust_error
        x_dis=self.x_dis 
        y_dis=self.y_dis
        detect_color=self.detect_color
        count_timeout=self.count_timeout
        rotation_angle=self.rotation_angle
        box_rotation_angle=self.box_rotation_angle
        last_x_dis=self.last_x_dis 
        last_y_dis=self.last_y_dis
        last_box_rotation_angle =self.last_box_rotation_angle 
        last_x =self.last_x 
        last_y =self.last_y 
        count_d=self.count_d 
        start_move=self.start_move


        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        
        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gray = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间 # Convert image to LAB space
        
        max_area = 0
        color_area_max = None
        areaMaxContour_max = 0
        roi = getROI(rotation_angle)
        for i in self.color_range:
            if i in target:
                if i in detect_color:
                    target_color_range = self.color_range[i]                
                    frame_mask1 = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 对原图像和掩模进行位运算# Do bitwise operations on the original image and mask
                    #mask = cv2.bitwise_and(roi, frame_gray)
                    frame_mask2 = cv2.bitwise_and(roi, frame_mask1)
                    eroded = cv2.erode(frame_mask2, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀 #corrosion
                    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀 #expand
                    #cv2.imshow('mask', dilated)
                    #cv2.waitKey(1)
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓 # find the contour
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓 # find the largest contour
                    if areaMaxContour is not None:
                        if area_max > max_area and area_max > 100:#找最大面积 #find the maximum area
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
        if max_area > 100:  # 有找到最大面积 # have found the largest area
            rect = cv2.minAreaRect(areaMaxContour_max)
            box_rotation_angle = rect[2]
            if box_rotation_angle > 45:
                box_rotation_angle =  box_rotation_angle - 90        
            
            box = np.int0(cv2.boxPoints(rect))   
            for j in range(4): # 映射到原图大小 # map to the original image size
                box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
                box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))
            
            cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
            
            centerX = int(Misc.map(((areaMaxContour_max[areaMaxContour_max[:,:,0].argmin()][0])[0] + (areaMaxContour_max[areaMaxContour_max[:,:,0].argmax()][0])[0])/2, 0, size[0], 0, img_w))
            centerY = int(Misc.map((areaMaxContour_max[areaMaxContour_max[:,:,1].argmin()][0])[1], 0, size[1], 0, img_h))
            
            #cv2.line(img, (0, 430), (640, 430), (0, 255, 255), 2)
            #cv2.circle(img, (int(centerX), int(centerY)), 5, range_rgb[color_area_max], -1)
            if abs(centerX - last_x) <= 5 and abs(centerY - last_y) <= 5 and not start_move:
                count_d += 1
                if count_d > 5:
                    count_d = 0
                    start_move = True
                    
                    led = Led()
                    led.index = 0
                    led.rgb.r = self.range_rgb[color_area_max][2]
                    led.rgb.g = self.range_rgb[color_area_max][1]
                    led.rgb.b = self.range_rgb[color_area_max][0]
                    self.rgb_pub.publish(led)
                    led.index = 1
                    self.rgb_pub.publish(led)
                    rospy.sleep(0.1)
                    
                    # 位置映射
    # location map
                    if 298 + self.d_color_map < centerY <= 424 + self.d_color_map:
                        Y = Misc.map(centerY, 298 + self.d_color_map, 424 + self.d_color_map, 0.12, 0.12 - 0.04)
                    elif 198 + self.d_color_map  < centerY <= 298 + self.d_color_map :
                        Y = Misc.map(centerY, 198 + self.d_color_map , 298 + self.d_color_map , 0.12 + 0.04, 0.12)
                    elif 114 + self.d_color_map  < centerY <= 198 + self.d_color_map :
                        Y = Misc.map(centerY, 114 + self.d_color_map , 198 + self.d_color_map , 0.12 + 0.08, 0.12 + 0.04)
                    elif 50 + self.d_color_map  < centerY <= 114 + self.d_color_map :
                        Y = Misc.map(centerY, 50 + self.d_color_map , 114 + self.d_color_map , 0.12 + 0.12, 0.12 + 0.08)
                    elif 0 + self.d_color_map  < centerY <= 50 + self.d_color_map :
                        Y = Misc.map(centerY, 0 + self.d_color_map , 50 + self.d_color_map , 0.12 + 0.16, 0.12 + 0.12)                    else:
                        Y = 1
            else:
                count_d = 0
            
            last_x = centerX
            last_y = centerY
            if (not approach or adjust) and start_move: # pid调节           
                detect_color = (color_area_max, )            
                self.x_pid.SetPoint = self.center_x #设定           
                self.x_pid.update(centerX) #当前
                dx = self.x_pid.output
                x_dis += dx #输出  
                
                x_dis = 0 if x_dis < 0 else x_dis          
                x_dis = 1000 if x_dis > 1000 else x_dis
                
                if adjust:
                    self.y_pid.SetPoint = self.color_y_adjust
                    start_move = True
                    centerY += abs(Misc.map(70*math.sin(math.pi/4)/2, 0, size[0], 0, img_w)*math.sin(math.radians(abs(gripper_rotation) + 45))) + 65*math.sin(math.radians(abs(roll_angle)))
                    if Y < 0.12 + 0.04:
                        centerY += d_color_y 
                    if 0 < centerY - self.color_y_adjust <= 5:
                        centerY = self.color_y_adjust
                    self.y_pid.update(centerY)

                    dy = self.y_pid.output
                    y_dis += dy
                    y_dis = 0.1 if y_dis > 0.1 else y_dis
                    y_dis = -0.1 if y_dis < -0.1 else y_dis
                else:
                    dy = 0
                if abs(dx) < 0.1 and abs(dy) < 0.0001 and (abs(last_box_rotation_angle - rect[2]) <= 10 or abs(last_box_rotation_angle - rect[2] >= 80)):
                    count += 1
                    rospy.sleep(0.01)
                    if (adjust and count > 10) or (not adjust and count >= 10):
                        count = 0
                        if adjust:
                            adjust = False
                        else:
                            rotation_angle = 240 * (x_dis - 500)/1000.0
                            X = round(-Y * math.tan(math.radians(rotation_angle)), 4)
                            state = 'color'
                            pick_color = detect_color[0]
                            adjust_error = False
                            approach = True
                else:
                    count = 0
                    
                if adjust and (abs(last_x_dis - x_dis) >= 2 or abs(last_y_dis - y_dis) > 0.002):
                    position = self.grasps.grasp_pos.position
                    rotation = self.grasps.grasp_pos.rotation
                    target = ik.setPitchRanges((position.x, position.y + y_dis, position.z), rotation.r, -180, 0)
                    if target: 
                        servo_data = target[1]
                        bus_servo_control.set_servos(self.joints_pub, 100, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, int(x_dis))))
                        rospy.sleep(0.1)
                        last_x_dis = x_dis
                        last_y_dis = y_dis
                    else:
                        bus_servo_control.set_servos(self.joints_pub, 20, ((6, int(x_dis)), ))
                else:                    
                    bus_servo_control.set_servos(self.joints_pub, 20, ((6, int(x_dis)), ))
                
                last_box_rotation_angle = rect[2]
        else:
            count_timeout += 1
            if count_timeout > 20:
                adjust_error = True
                count_timeout = 0
                current_tag = ['tag1', 'tag2', 'tag3']
                detect_color = self.__target_data[0]
        return img

    def tag_sort(self, img, target):
        self.d_map = 0.015
        self.tag_map = [425, 384, 346, 310, 272, 239, 208, 177, 153, 129, 106, 86, 68, 51]
        global X, Y
        global state
        global count2
        global count3
        global adjust    
        global approach
        global start_move
        global current_tag   
        global adjust_error
        global last_X, last_Y
        global box_rotation_angle
        global tag_x_dis, tag_y_dis
    
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        centerX = target[1]
        centerY = target[2]

        box_rotation_angle = abs(target[3])
        if box_rotation_angle > 90:
            box_rotation_angle -= 90
        if box_rotation_angle > 45:
            box_rotation_angle =  box_rotation_angle - 90
        if target[3] < 0:
            box_rotation_angle = -box_rotation_angle
        
        distance = math.sqrt(pow(centerX - last_X, 2) + pow(centerY - last_Y, 2)) #对比上次坐标来判断是否移动        
        if distance < 5 and not start_move:
            count2 += 1
            if count2 > 20:
                count2 = 0
                start_move = True          
        else:
            count2 = 0
        if (not approach or adjust) and start_move:  
            self.tag_x_pid.SetPoint = self.center_x #设定
            self.tag_x_pid.update(centerX) #当前
            dx = self.tag_x_pid.output
            tag_x_dis += dx #输出  
            tag_x_dis = 0 if tag_x_dis < 0 else tag_x_dis          
            tag_x_dis = 1000 if tag_x_dis > 1000 else tag_x_dis
            
            if abs(centerX - last_X) <= 1 and X != -1:
                count3 += 1
                rospy.sleep(0.01)
                if count3 > 30:
                    count3 = 0
                    if adjust:
                        adjust = False
                    else:
                        current_tag = target[0]
                        # 位置映射
                        if self.tag_map[1] + self.d_tag_map < centerY <= self.tag_map[0] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[1] + self.d_tag_map, self.tag_map[0] + self.d_tag_map, 0.12 + d_map, 0.12) - 0.005
                        elif self.tag_map[2] + self.d_tag_map < centerY <= self.tag_map[1] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[2] + self.d_tag_map, self.tag_map[1] + self.d_tag_map, 0.12 + 2*d_map, 0.12 + d_map)
                        elif self.tag_map[3] + self.d_tag_map < centerY <= self.tag_map[2] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[3] + self.d_tag_map, self.tag_map[2] + self.d_tag_map, 0.12 + 3*d_map, 0.12 + 2*d_map)
                        elif self.tag_map[4] + self.d_tag_map < centerY <= self.tag_map[3] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[4] + self.d_tag_map, self.tag_map[3] + self.d_tag_map, 0.12 + 4*d_map, 0.12 + 3*d_map)
                        elif self.tag_map[5] + self.d_tag_map < centerY <= self.tag_map[4] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[5] + self.d_tag_map, self.tag_map[4] + self.d_tag_map, 0.12 + 5*d_map, 0.12 + 4*d_map)
                        elif self.tag_map[6] + self.d_tag_map < centerY <= self.tag_map[5] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[6] + self.d_tag_map, self.tag_map[5] + self.d_tag_map, 0.12 + 6*d_map, 0.12 + 5*d_map)
                        elif self.tag_map[7] + self.d_tag_map < centerY <= self.tag_map[6] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[7] + self.d_tag_map, self.tag_map[6] + self.d_tag_map, 0.12 + 7*d_map, 0.12 + 6*d_map)
                        elif self.tag_map[8] + self.d_tag_map < centerY <= self.tag_map[7] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[8] + self.d_tag_map, self.tag_map[7] + self.d_tag_map, 0.12 + 8*d_map, 0.12 + 7*d_map)
                        elif self.tag_map[9] + self.d_tag_map < centerY <= self.tag_map[8] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[9] + self.d_tag_map, self.tag_map[8] + self.d_tag_map, 0.12 + 9*d_map, 0.12 + 8*d_map)
                        elif self.tag_map[10] + self.d_tag_map < centerY <= self.tag_map[9] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[10] + self.d_tag_map, self.tag_map[9] + self.d_tag_map, 0.12 + 10*d_map, 0.12 + 9*d_map)
                        elif self.tag_map[11] + self.d_tag_map < centerY <= self.tag_map[10] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[11] + self.d_tag_map, self.tag_map[10] + self.d_tag_map, 0.12 + 11*d_map, 0.12 + 10*d_map)
                        elif self.tag_map[12] + self.d_tag_map < centerY <= self.tag_map[11] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[12] + self.d_tag_map, self.tag_map[11] + self.d_tag_map, 0.12 + 12*d_map, 0.12 + 11*d_map)
                        elif self.tag_map[13] + self.d_tag_map < centerY <= self.tag_map[12] + self.d_tag_map:
                            Y = Misc.map(centerY, self.tag_map[13] + self.d_tag_map, self.tag_map[12] + self.d_tag_map, 0.12 + 13*d_map, 0.12 + 12*d_map)
                        else:
                            Y = 1
                        
                        X = round(-Y * math.tan(math.radians(self.rotation_angle)), 4)
                        state = 'tag'
                        approach = True
                        adjust_error = False
                        adjust = False
            else:
                count3 = 0
            
            bus_servo_control.set_servos(self.joints_pub, 20, ((6, int(tag_x_dis)), ))
        last_X, last_Y = centerX, centerY
            
        return img

if __name__ == '__main__':
    #test tracking## 
    tr = Perception()
    tr.initMove()    
    tr.target_color = ('red')
    cap = cv2.VideoCapture(0)
    time.sleep(3)
    
    while True:
        ret,img = cap.read()
        if ret:
            frame = img.copy()
            Frame = tr.run_track(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    #show detections
    my_camera.camera_close()
    cv2.destroyAllWindows()