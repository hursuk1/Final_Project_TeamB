#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math, copy, rospy
from calibration import calibration.calibrate_image
class StopLineCountNoneZero():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.width = 640
        self.height = 480
        self.image = None

        self.mtx = np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])
        self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

    def img_callback(self, img):
        self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")

    def detect_stopline(self, cal_image, high_threshold_value):
        stopline_roi, _, _ = self.set_roi(cal_image, 250, 350, 10)
        image = self.image_processing(stopline_roi, high_threshold_value)
        if cv2.countNonZero(image) > 1000:
            print("stopline")

    def set_roi(self, frame, x_len, start_y, offset_y):
        _, width, _ = frame.shape
        start_x = int(width/2 - (x_len/2))
        end_x = int(width - start_x)
        return frame[start_y:start_y+offset_y, start_x:end_x], start_x, start_y

    def image_processing(self, image, high_threshold_value):
        blur = cv2.GaussianBlur(image, (5, 5), 0)
        L, A, B = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2LAB))
        _, lane = cv2.threshold(L, 0, 255, cv2.THRESH_BINARY)
        return lane

    # def calibrate_image(frame, mtx, dist, cal_mtx, cal_roi):
    #     height, width, _ = frame.shape
    #     tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    #     x, y, w, h = cal_roi
    #     tf_image = tf_image[y:y+h, x:x+w]

    #     return cv2.resize(tf_image, (width, height))

     
stopline = StopLineCountNoneZero()

while not rospy.is_shutdown():
    if StopLineCountNoneZero.image.shape != 640*480*3:
        continue
    
    stopline.detect_stopline()
# while cap.isOpened():
#     #_, frame = cap.read()
#     cal_image = calibrate_image(frame, mtx, dist, cal_mtx, cal_roi)
#     detect_stopline(cal_image, 70)
#     cv2.imshow("simple detect", cal_image)
#     cv2.waitKey(1)
