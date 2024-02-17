#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import numpy as np


bridge = CvBridge()

class ball_detection : #Definition of the ball detection class
    def __init__(self): #Definition of the instructor
        self.ball_position = None
#
    def image_callback(self, image):
        cv2_img = bridge.imgmsg_to_cv2(image, "bgr16")
        cv2.imshow("Prometheus Bot Camera", cv2_img)
        frame = np.array(cv2_img, dtype=np.uint8)
        color_image = self.color_detection(frame)
        edge_image = self.edges_detection(frame)
        if self.ball_position is not None:
            x, y = self.ball_position
            cv2.rectangle(frame, (int(x) - 10, int(y) - 10), (int(x) + 10, int(y) + 10), (0, 0, 255), 2)
            cv2.putText(frame, f"Ball Position: {self.ball_position}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Ball Tracking", frame)
        cv2.waitKey(3)

    def edges_detection(self, frame):
        grey_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_img = cv2.blur(grey_img, (7,7))
        edges = cv2.Canny(blurred_img, 15.0, 30.0)
        cv2.imshow ("Edges", edges)
        return edges
    
    def color_detection (self,frame):
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # lower boundary RED color range values; Hue (0 - 10)
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([10, 255, 255])
        
        # upper boundary RED color range values; Hue (160 - 180)
        lower2 = np.array([160,100,20])
        upper2 = np.array([179,255,255])

        lower_mask = cv2.inRange(hsv_img, lower1, upper1)
        upper_mask = cv2.inRange(hsv_img, lower2, upper2)

        full_mask = lower_mask + upper_mask

        result = cv2.bitwise_and(frame, frame, mask=full_mask)
        cv2.imshow ("Binary Masked Image", result)
        # Find contours in the binary mask
        contours, _ = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour (assuming it corresponds to the ball)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

        # If the radius is above a certain threshold, consider it as the ball
            if radius > 5:
            # Draw a circle around the ball
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

            # Update the ball position
                self.ball_position = (int(x), int(y))
        return result
    
    def camera_view_node (self):
        rospy.init_node("image_subscriber", anonymous= False)
        image_topic = "/prometheus_final/camera_bot/image_raw"
        sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.spin()

if __name__ == '__main__':
    prometheus_bot = ball_detection()
    prometheus_bot.camera_view_node()