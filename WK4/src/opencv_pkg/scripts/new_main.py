# from asyncio.windows_events import NULL
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from time import sleep
# from tracker import EuclideanDistTracker

# tracker = EuclideanDistTracker()

class Tracker():
    def __init__(self):
        self.bridge = CvBridge()
        # self.number = 0
    def subs(self):
        self.number = 0
        self.front_camera = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.callback)
    
    def callback(self, data):
        global id
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Resize image
        # test_image = cv_image
        # cv2.imshow("GCM", test_image)
        test_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        cv2.imshow("GCM", test_image)
        # Extract region of interest (roi)
        # height, width, _ = cv_image.shape
        # roi = cv_image[0: 540, 600: 960]
        roi = cv_image[0: 1080, 1000: 1920]
        # roi = cv_image[0: 1080, 1100: 1920]
        # Convert the extracted image from RGB to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Set range for grape colour
        lower_grape_colour = np.array([100, 18, 46])
        higher_grape_colour = np.array([107, 255, 255])

        # Threshold HSV image to get only grape colours
        grape_colour_mask = cv2.inRange(hsv, lower_grape_colour, higher_grape_colour)
        _, grape_colour_mask = cv2.threshold(grape_colour_mask, 254, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(grape_colour_mask, kernel, iterations = 1)




        # Retrieve contour for analysis and shape detection
        cnts, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
        detections = []
        # id = 0
        print(self.number)
        
        for c in cnts:
            area = cv2.contourArea(c)
            # print(area)
            if area > 350:
                x,y,w,h = cv2.boundingRect(c)
                detections.append([x,y,w,h])
                # cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
                # cv2.putText(roi, str(id), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
        # print(len(detections))
        # print(detections)
        for detection in detections:
            x, y, w, h = detection
            self.number += 1
            cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
            cv2.putText(roi, str(self.number), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)


        # boxes_ids = tracker.update(detections)
        # for box_id in boxes_ids:
        #     x, y, w, h, id = box_id
        #     cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
        #     cv2.putText(roi, str(id), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)



        cv2.imshow("ROI", roi)
        # cv2.imshow("GCM", test_image)
        #print(height, width)

        cv2.waitKey(30)
        # sleep(10)
        # self.front_camera.unregister()

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