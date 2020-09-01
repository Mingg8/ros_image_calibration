#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time


class ImageProcess:
    def __init__(self, mtx, dist ,size) :
        self.mtx = mtx
        self.dist = dist
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
            # cv2.imwrite('./jpeg/camera_image'+str(self.num)+'.jpeg', cv2_img)
            self.num = self.num + 1
    

if __name__ == '__main__':
    try:
        rospy.init_node('image_scriber', anonymous=True)
        cv_file = cv2.FileStorage("../Intrinsic.xml", cv2.FILE_STORAGE_READ)
        cv_file2 = cv2.FileStorage("../Distortion.xml", cv2.FILE_STORAGE_READ)

        mtx = cv_file.getNode("Intrinsic").mat()
        dist = cv_file2.getNode("Distortion").mat()
        cv_file.release()
        cv_file2.release()
        size = (640, 480)

        img = ImageProcess(mtx, dist, size)
        sub = rospy.Subscriber('/usb_cam/image_raw', Image, img.image_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
