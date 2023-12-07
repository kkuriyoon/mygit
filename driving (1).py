#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수

CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)


def draw_line(img, slope, intercept, color, thickness):
    height = img.shape[0]
    width = img.shape[1]
    
    # 시작점의 x 좌표 계산
    x1 = 0
    # 시작점의 y 좌표 계산
    y1 = int(slope * x1 + intercept)
    # 끝점의 x 좌표 계산
    x2 = width - 1
    # 끝점의 y 좌표 계산
    y2 = int(slope * x2 + intercept)
    
    # 직선 그리기
    cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def start():

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    
    src = np.float32([(180, 280), (460, 280), (640, 480), (0, 480)])
    dst = np.float32([(0, 0), (640, 0), (640, 480), (0,480)])

    while not rospy.is_shutdown():

        # 원본이미지를 복사
        img = image.copy()  
        cv2.waitKey(1)  
        # top view
        M = cv2.getPerspectiveTransform(src, dst)
        img2 = cv2.warpPerspective(img, M, (640, 480))
        # 흑백 변
        img_hsv = cv2.cvtColor(img2,cv2.COLOR_BGR2HSV)
        #cv2.imshow("g", img_hsv)
        # 이미지 이진화
        #_, img_bin = cv2.threshold(img_gaussian, 150, 255, cv2.THRESH_BINARY)
        # canny
        #edges = cv2.Canny(img_bin, 100, 200)
        # only 흰색 픽셀 추출
        lower = (0,0,180)
        upper = (0,0,255)
        white_mask = cv2.inRange(img_hsv, lower, upper)
        img_white = cv2.bitwise_and(img2, img2, mask=white_mask)

        #cv2.imshow("w", img_white)
        #white_mask = cv2.inRange(img_bin, lower, upper)
        #img_white = cv2.bitwise_and(edges, edges, mask=white_mask)

        # 필터 적용 (gaussian)
        img_gaussian = cv2.GaussianBlur(img_white, (5, 5), 0)
        edges = cv2.Canny(img_gaussian, 100, 200)
        cv2.imshow("canny", edges)

        # 직선 검출(허프변환)
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=80, minLineLength=70, maxLineGap=5)
        # 가로 직선
        center_y = img2.shape[0] // 2
        cv2.line(img2, (0, center_y), (img2.shape[1], center_y), (255, 0 ,0), 2)
        # 왼쪽 교점, 오른쪽 교점
        left_intersection = None
        right_intersection = None

        # 원
        d=[]
        p=[]
        s=[]
        u=[]
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            # 허프변환을 통해 얻은 하나의 직선에 대한 기울기와 y 절편
            a = (y2 - y1) / (x2 - x1)
            b = y1 - a * x1
            
           
            
            
            
            # 왼쪽 차선과 오른쪽 차선 구분
            if a>=0 :
                if (y1 >= center_y)&(y2>=center_y) :
                    cv2.line(img2, (x1, y1), (x2, y2), (0,0,255),2)
                    if x1>=x2 :
                        cv2.circle(img2, (x1,y1), radius = 5, color = (0,0,255), thickness=-1)
                    
                    else :
                        cv2.circle(img2, (x2,y2), radius = 5, color = (0,0,255), thickness=-1)
                        
                
                
                
    
            else :
                if (y1 >= center_y)&(y2 >= center_y) :
                    cv2.line(img2, (x1, y1), (x2, y2), (0,255,0),2)
                    if x1 >= x2 :
                        cv2.circle(img2, (x2,y2), radius = 5, color = (0,0,255), thickness=-1)
                    else :
                        cv2.circle(img2, (x1,y1), radius = 5, color = (0,0,255), thickness=-1)
                
              
                
            
            
            
            intersection_x = (center_y - b) / a
            intersection_y = center_y
            
            
            min = 0
            max = 1000
            if intersection_x >= min :
                min = intersection_x
                cv2.circle(img2, (int(min), int(center_y)), radius = 5, color = (0, 255, 0), thickness = -1)
                
            if intersection_x <= max :
                max = intersection_x
                cv2.circle(img2, (int(max), int(center_y)), radius = 5, color = (0, 255, 0), thickness = -1)
          
        # 교점의 중심 좌표
            
            
            
        
            
      
        # 이미지 표
        cv2.imshow("Top View", img2)
        cv2.waitKey(1)       
       
        angle = 0
        speed = 10
		
        drive(angle, speed)

if __name__ == '__main__':
    start()

