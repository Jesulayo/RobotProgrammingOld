# from asyncio.windows_events import NULL
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from random import randint
import sys


class Tracker(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.front_camera = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.callback)
        # self.tracker = cv2.TrackerCSRT_create()

    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # success, frame,_ = cv_image.shape
        print(cv_image)
        # cv2.imshow("cv_image", cv_image)
        cv2.waitKey(30)

def main():
    rospy.init_node("tracker", anonymous=True)
    Tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()