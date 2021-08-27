#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math, copy, rospy, time
from cv_bridge import CvBridge
from calibration import Calibration
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor

class StopLineCountNoneZero(Calibration):
    def __init__(self):
        # self.image = np.empty(shape=[0])
        self.image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        # self.width = 640
        # self.height = 480
        self.stop = False

        # self.mtx = np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])
        # self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        # self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width, self.height), 1, (self.width, self.height))

    def img_callback(self, img):
        self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

    # def calibrate_image(self, frame, mtx, dist, cal_mtx, cal_roi):
    #     tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    #     x, y, w, h = cal_roi
    #     tf_image = tf_image[y:y+h, x:x+w]
    #     return cv2.resize(tf_image, (frame.shape[1], frame.shape[0]))

    def detect_stopline(self, high_threshold_value):
        img = self.image
        #cv2.imshow('orginal', img)
        image = self.calibrate_image(img, self.mtx, self.dist, self.cal_mtx, self.cal_roi)
        stopline_roi = self.set_roi(image)
        #cv2.imshow('roi', stopline_roi)
        image = self.image_processing(stopline_roi, high_threshold_value)
        image = cv2.bitwise_not(image)
        cv2.imshow('image_processing', image)
        if cv2.countNonZero(image) > 1000:
            print("stopline")
            self.stop = True
        else:
            print("go")
            self.stop = False

    def set_roi(self, img):
        return img[370:395, 160:480]

    def image_processing(self, image, high_threshold_value):
        blur = cv2.GaussianBlur(image, (5, 5), 0)
        # img = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
        # cv2.imshow("2", img)
        L, A, B = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2LAB))
        #cv2.imshow('L', L)
        _, lane = cv2.threshold(L, 80, 255, cv2.THRESH_BINARY)
        #cv2.imshow('img6', lane)
        return lane

motor_msg = xycar_motor()
rospy.init_node('stop_line')     
stopline = StopLineCountNoneZero()
pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size = 1)

while not rospy.is_shutdown():
    # while stopline.image.shape != 640*480*3:
    #     continue

    while stopline.image is None:
        time.sleep(0.03)

    stopline.detect_stopline(5)
    # cv2.imshow("image", image)
    
    if stopline.stop == True:
        motor_msg.speed = 0
    else:
        motor_msg.speed = 10
    motor_msg.angle = 0
    
    pub.publish(motor_msg)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# while cap.isOpened():
#     #_, frame = cap.read()
#     cal_image = calibrate_image(frame, mtx, dist, cal_mtx, cal_roi)
#     detect_stopline(cal_image, 70)
#     cv2.imshow("simple detect", cal_image)
#     cv2.waitKey(1)
