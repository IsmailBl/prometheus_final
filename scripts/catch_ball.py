#!/usr/bin/env python3

import rospy    #Ros liberary
from geometry_msgs.msg import Twist #Twist to make robot move
import cv2, cv_bridge  #Open cv library for image processing
import numpy as np #numpy for the arrays
from std_msgs.msg import Float64 
from sensor_msgs.msg import Image #image msg for camera


#Initialising the variables
x_loc = y_loc = area = 0
twist = Twist()
ball_detected = False

#image callback function gets the image from camera and calculates the coordinates of the ball
def goal_detection(image):
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #converting to hsv
    lower_blue = np.array([90, 100, 20]) #lower limit of blue bgr
    upper_blue = np.array([110, 255, 255]) #upper limit of blue bgr
    mask = cv2.inRange(hsv, lower_blue, upper_blue) #masking the image frame
    M= cv2.moments(mask)

    if M['m00'] > 0: #finding the center of the locataion of ball, if ball is in the camera frame
        x,y,w,h= cv2.boundingRect(mask)
        x_loc= int(M['m10']/M['m00']) #x coordinate of ball
        y_loc= int(M['m01']/M['m00']) #y coordinate of ball
        area = w*h #calculating the area in order to analys the distance from camera to ball
        #print(x_loc, y_loc, area)
    else: #if ball is not in the camera
        x_loc = y_loc = area = 0 #then all parameters are zero
    cv2.imshow("window", image) #showing rgb image
    cv2.imshow("mask", mask) #showing mask image
    cv2.waitKey(3) #delay to display frames
    if M['m00'] > 0: #finding the center of the location of the goal
        x,y,w,h= cv2.boundingRect(mask)
        x_loc= int(M['m10']/M['m00']) #x coordinate of the goal
        y_loc= int(M['m01']/M['m00']) #y coordinate of the goal
        area = w*h #calculating the area in order to analys the distance from camera to ball
        #print(x_loc, y_loc, area)
    else: #if ball is not in the camera
        x_loc = y_loc = area = 0 #then all parameters are zero
    #cv2.imshow("window", image) #showing rgb image
    cv2.imshow("Goal mask",mask) #showing mask image
    cv2.waitKey(3) #delay to display frames


def image_callback(msg):
    global x_loc, y_loc, area
    bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)    #rgb frame of camera
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    goal_detection(image)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #converting to hsv
    
    lower1 = np.array([0, 100, 20])
    upper1 = np.array([10, 255, 255])
        
        # upper boundary RED color range values; Hue (160 - 180)
    lower2 = np.array([160,100,20])
    upper2 = np.array([179,255,255])

    lower_mask = cv2.inRange(hsv, lower1, upper1)
    upper_mask = cv2.inRange(hsv, lower2, upper2)

    full_mask = lower_mask + upper_mask
    #mask = cv2.inRange(hsv, lower_blue, upper_blue) #masking the image frame
    M= cv2.moments(full_mask)
    
    if M['m00'] > 0: #finding the center of the location of ball, if ball is in the camera frame
        x,y,w,h= cv2.boundingRect(full_mask)
        x_loc= int(M['m10']/M['m00']) #x coordinate of ball
        y_loc= int(M['m01']/M['m00']) #y coordinate of ball
        area = w*h #calculating the area in order to analys the distance from camera to ball
        #print(x_loc, y_loc, area)
    else: #if ball is not in the camera
        x_loc = y_loc = area = 0 #then all parameters are zero
    cv2.imshow("window", image) #showing rgb image
    cv2.imshow("Ball mask",full_mask) #showing mask image
    cv2.waitKey(3) #delay to display frames

    
rospy.init_node('ball_catch_node') #starting the node
twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #twist publisher
dribbler_pub = rospy.Publisher('/prometheus_final/dribbler_velocity_controller/command', Float64, queue_size=10) #joint1 angle publisher
image_sub = rospy.Subscriber('/prometheus_final/camera_bot/image_raw', Image, image_callback) #subcribing the image from camera
kicker_pub = rospy.Publisher('/prometheus_final/kicker_position_controller/command', Float64, queue_size=10)
rate = rospy.Rate(10) #frequency

def reach_ball():
    #print("-----")
    if x_loc < 250 and area < 150000: #if the ball is on the left side of camera and far away
        #print("rotate left")
        twist.linear.x = 0
        twist.angular.z = 2.5 #rotate left
    elif x_loc > 450 and area < 150000: #if the ball is on the right side of camera and far away
        #print("rotate right")
        twist.linear.x = 0
        twist.angular.z = -2.5#rotate right
    elif x_loc < 451 and x_loc > 249 and area < 150000: #if the ball is in the middle of camera and far away
        #print("move forward")
        twist.linear.x = 0.8 #move forward
        twist.angular.z = 0
    elif area > 150000: #if ball is very close in front of camera
        twist.linear.x = 0 #stop
        twist.angular.z = 0
        twist_pub.publish(twist)
        rospy.sleep(2)
        dribbler_pub.publish(92.0) #turn the dribbler
        print(" Ball reached")
        kicker_pub.publish(100) # =Kick the ball
        rospy.sleep(2)
        kicker_pub.publish(100) # =Kick the ball
        kicker_pub.publish(0) # =Kick the ball
        
    twist_pub.publish(twist) #publishing the twist msg
    
        
#running the loop
while not rospy.is_shutdown():
    reach_ball()
    #reach_goal()
    rate.sleep()