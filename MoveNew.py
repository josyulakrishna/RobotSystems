#!/usr/bin/python3
# coding=utf8
import sys
from charset_normalizer import detect
import cv2
import time
import threading

from numpy import place
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
import HiwonderSDK.yaml_handle as yaml_handle
from armpi_fpv import bus_servo_control
import rospy
from warehouse.msg import Grasp
import HiwonderSDK.yaml_handle as yaml_handle


joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)


class MoveArm: 
    def __init__(self):
        self.x_dis = 500
        self.y_dis = 10
        self.Z_DIS = 18
        self.z_dis = 18
        self.x_pid = PID.PID(P=0.1, I=0.00, D=0.008)  # pid初始化 #pid initialization
        self.y_pid = PID.PID(P=0.00001, I=0, D=0)
        self.z_pid = PID.PID(P=0.005, I=0, D=0)
        self.__isRunning = False

    #move the arm to track a target
    def track(self,areaMaxContour):
        size = (640,320)
        (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour)  # 获取最小外接圆 Get the smallest circumcircle
        center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
        center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        rect = cv2.minAreaRect()
        box = np.int0(cv2.boxPoints(rect))
        self.x_pid.SetPoint = self.img_w / 2.0  # 设定 #set up
        self.x_pid.update(center_x)  # 当前 # current
        dx = self.x_pid.output
        x_dis += int(dx)  # 输出 #output

        x_dis = 0 if x_dis < 0 else x_dis
        x_dis = 1000 if x_dis > 1000 else x_dis

        self.y_pid.SetPoint = 9000  # 设定 #set up
        if abs(area_max - 9000) < 50:
            area_max = 9000
        self.y_pid.update(area_max)  # 当前 #current
        dy = self.y_pid.output
        y_dis += dy  # 输出 #output
        y_dis = 5.00 if y_dis < 5.00 else y_dis
        y_dis = 10.00 if y_dis > 10.00 else y_dis
        
        if abs(center_y - self.img_h/2.0) < 20:
            self.z_pid.SetPoint = center_y
        else:
            self.z_pid.SetPoint = self.img_h / 2.0
            
        self.z_pid.update(center_y)
        dy = self.z_pid.output
        z_dis += dy

        z_dis = 32.00 if z_dis > 32.00 else z_dis
        z_dis = 10.00 if z_dis < 10.00 else z_dis
        
        target = AK.setPitchRange((0, round(y_dis, 2), round(z_dis, 2)), -90, 0)
        
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
    
    #place targets palletizing
    def place(self, places):
        position = places.grasp_pos.position
        rotation = places.grasp_pos.rotation
        approach = places.grasp_approach
        retreat = places.grasp_retreat
        
        # 计算是否能够到达目标位置，如果不能够到达，返回False
        target1 = ik.setPitchRanges((position.x + approach.x, position.y + approach.y, position.z + approach.z), rotation.r, -180, 0)
        target2 = ik.setPitchRanges((position.x, position.y, position.z), rotation.r, -180, 0)
        target3 = ik.setPitchRanges((position.x, position.y, position.z + places.up), rotation.r, -180, 0)
        target4 = ik.setPitchRanges((position.x + retreat.x, position.y + retreat.y, position.z + retreat.z), rotation.r, -180, 0)
        
        if not self.__isRunning:
            return False
        if target1 and target2 and target3 and target4:
            # 第一步：云台转到朝向目标方向
            servo_data = target1[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, places.pre_grasp_posture), (2, int(F*rotation.y)), (3, 150), (4, 825), (5, 625), (6, servo_data['servo6'])))
            rospy.sleep(1)
            if not self.__isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
                rospy.sleep(0.5)            
                return False
            
            # 第二步：移到接近点
            bus_servo_control.set_servos(joints_pub, 1500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))      
            rospy.sleep(1.6)
            if not self.__isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
                rospy.sleep(0.5)             
                return False
            
            # 第三步：移到目标点
            servo_data = target2[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6']))) 
            rospy.sleep(1.5)
            if not self.__isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
                rospy.sleep(0.5)             
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
                rospy.sleep(1)              
                return False
            
            # 第四步：抬升
            if places.up != 0:
                servo_data = target3[1]
                bus_servo_control.set_servos(joints_pub, 800, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
                rospy.sleep(0.8)
            if not self.__isRunning:
                bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))               
                rospy.sleep(0.5)             
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
                rospy.sleep(1)              
                return False

            # 第五步：放置
            bus_servo_control.set_servos(joints_pub, 400, ((1, places.pre_grasp_posture - 40), ))         
            rospy.sleep(0.8)        
            bus_servo_control.set_servos(joints_pub, 500, ((1, places.grasp_posture), ))         
            rospy.sleep(1)
            if not self.__isRunning:
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))       
                rospy.sleep(1)              
                return False
            
            # 第六步：移到撤离点
            servo_data = target4[1]
            if servo_data != target3[1]:
                bus_servo_control.set_servos(joints_pub, 600, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
                rospy.sleep(0.6)
                if not self.__isRunning:
                    return False
                
            # 第七步：移到稳定点
            servo_data = target1[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
            rospy.sleep(1)
            if not self.__isRunning:
                return False
            
            return True
        else:
            rospy.loginfo('place failed')
            return False


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    areaMaxContour = None

    for c in contours:  # 历遍所有轮廓 #Iterate over all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积 #Calculate the contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰 # Only when the area is greater than 300, the contour with the largest area is valid to filter out interference
                areaMaxContour = c
    return areaMaxContour

if __name__=="main":
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    ma=MoveArm()
    detect_color="red"
    size = [640,320]
    for i in range(100):
        img = cv2.VideoCapture(0)
        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)     
        frame_mask = cv2.inRange(frame_lab,(lab_data[detect_color]['min'][0],
                                    lab_data[detect_color]['min'][1],
                                    lab_data[detect_color]['min'][2]),
                                    (lab_data[detect_color]['max'][0],
                                    lab_data[detect_color]['max'][1],
                                    lab_data[detect_color]['max'][2]))  

        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] 
        areaMaxContour = getAreaMaxContour(contours)
        #test Tracking 
        ma.track(areaMaxContour)

    #Test grasp and place
    place_position = {'red':  [-0.13, -0, 0.01],
                  'green':[-0.13, -0, 0.01],
                  'blue': [-0.13, -0, 0.01],
                  'tag1': [ 0.13, -0, 0.01],
                  'tag2': [ 0.13, -0, 0.01],
                  'tag3': [ 0.13, -0, 0.01]}
    position = place_position["red"]
    places = Grasp()                    
    places.grasp_pos.position.x = position[0]
    places.grasp_pos.position.y = position[1]
    places.grasp_pos.position.z = position[2]
    places.grasp_pos.position.z -= 0
    places.grasp_pos.rotation.r = 0
    places.grasp_pos.rotation.y = 0
    
    places.up = 0.0
    places.grasp_approach.z = 0.02
    places.grasp_retreat.z = 0.04
    
    places.grasp_posture = 75
    places.pre_grasp_posture = 450                        
    ma.place(places)