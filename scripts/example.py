#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import time


class ImageProcess:
    def __init__(self, mtx, dist ,size, H) :
        self.mtx = mtx
        self.dist = dist
        self.size = size
        self.H = H
        self.num = 1
        self.bridge = CvBridge()
        self.cameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, size, 1, size)

    def image_callback(self, msg):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            time = msg.header.stamp
            dst = cv2.undistort(cv2_img, self.mtx, self.dist, None, self.cameramtx)
            warp_img = cv2.warpPerspective(dst, self.H, self.size)
            cv2.imwrite('/home/mjlee/tmp/jpeg/warp/warp_image'+str(self.num)+'.jpeg', warp_img)
            self.num = self.num + 1
    

if __name__ == '__main__':
    try:
        rospy.init_node('image_scriber', anonymous=True)
        cv_file = cv2.FileStorage("../Intrinsic.xml", cv2.FILE_STORAGE_READ)
        cv_file2 = cv2.FileStorage("../Distortion.xml", cv2.FILE_STORAGE_READ)

        # Load Camera intrinsic matrix
        mtx = cv_file.getNode("Intrinsic").mat()
        dist = cv_file2.getNode("Distortion").mat()
        cv_file.release()
        cv_file2.release()
        size = (640, 480)

        # Find Homography
        imgPts = np.array([[304.2, 56.2], [308.7, 273.3], [474.0, 270.5], [474.7, 53.8]], dtype = np.float32)
        objPts = np.array([[170, 45], [170, 435], [470, 435], [470, 45]], dtype = np.float32)
        H = cv2.getPerspectiveTransform(imgPts, objPts)

        img = ImageProcess(mtx, dist, size, H)
        sub = rospy.Subscriber('/usb_cam/image_raw', Image, img.image_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
