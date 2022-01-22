# from asyncio.windows_events import NULL
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# from tracker import EuclideanDistTracker

# tracker = EuclideanDistTracker()

# count = 0
# centroids = []
state = False
# id = []

class Tracker(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.front_camera = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.callback)
    
    def callback(self, data):
        # global count
        global state
        # global centroids
        # global id
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        # Resize image
        test_image = cv_image
        cv2.imshow("GCM", test_image)
        #cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)

        # Extract region of interest (roi)
        # height, width, _ = cv_image.shape
        # roi = cv_image[0: 540, 600: 960]
        roi = cv_image[0: 1080, 1000: 1920]

        
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
        id = []
        count = 0
        centroids = []
        # id = 0
        
        for c in cnts:
            area = cv2.contourArea(c)
            # print(area)
            if area > 350:
                x,y,w,h = cv2.boundingRect(c)
                detections.append([x,y,w,h])
                # cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
                # cv2.putText(roi, str(id), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)


        # print(detections)
        # if len(detections) >= 1:
        for detection in detections:
            x, y, w, h = detection
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            centroid = [cx, cy]
            # centroids.append(centroid)
            if state is False:
                centroids.append(centroid)
                count += 1
                id.append([centroid + [count]])
                print("here")
                # print(id)
                # print(centroid)
                # print(centroids)
                # print(centroid)
                # cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
                # cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                # cv2.putText(roi, str(count), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
                state = True
            if len(centroids) >= 1:
                if centroid in centroids:
                    ind = centroids.index(centroid)
                    # print(centroid)
                    # print(centroids)
                    # print(ind)
                    print("repete")
                    # get_count = id[ind][3]
                    # cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
                    # cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                    # cv2.putText(roi, str("nan"), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
                else:
                    count += 1
                    centroids.append(centroid)
                    id.append([centroid + [count]])
                    print(centroid)
                    print(centroids)
                    # print(ind)
                    print(id)
                    # cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
                    # cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                    # cv2.putText(roi, str(count), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)
            else:
                pass

            cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)
            cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(roi, str(count), (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)


        cv2.imshow("ROI", roi)
        # cv2.imshow("GCM", test_image)
        #print(height, width)

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