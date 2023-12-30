#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Krishi Bot (KB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script is to implement Task 2.2 of Krishi Bot (KB) Theme (eYRC 2022-23).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ 2546 ]
# Author List:		[ Joel Devasia ]
# Filename:			percepStack.pys
# Functions:		
# 					[ img_clbck, depth_clbck, image_processing, tf_broadcaster, get_realWorld_Cordinates, twoD_to_threeD,  main ]

####################### IMPORT MODULES #######################
import cv2 
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import numpy as np
import tf
import math
# You can add more if required
##############################################################

# Initialize Global variables

bridge = CvBridge()
pose = []
fruit_red = []
fruit_yellow = []
depth_val_red = []
depth_val_yellow = []
bot_Status = ""
fx = 554.387
fy = 554.387
cx = 320.5
cy = 240.5

################# ADD UTILITY FUNCTIONS HERE #################

def bot_status_updater(status):
    global bot_Status
    bot_Status = status.data

bot_pub = rospy.Publisher("bot_status", String, queue_size=1)
rospy.Subscriber("bot_status", String, bot_status_updater)

def tf_broadcaster(X, Y, Z, fruit, parent):
    rate = rospy.Rate(5)
    br = tf.TransformBroadcaster()
    global bot_Status
    # print("X: ", X, "Y: ", Y, "Z: ", Z, "fruit: ", fruit, "parent: ", parent)
    if(not math.isnan(X) and not math.isnan(Y) and not math.isnan(Z)):
        if(Z> 0.4 and Z < 1.5):
            br.sendTransform((Z, -X, -Y),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     fruit,
                     parent)
            bot_pub.publish("stop")
            rate.sleep()
        # else:
        #     a = 1 +3

    else:
        a = 1+2
        # print("X: ", X, "Y: ", Y, "Z: ", Z, "fruit: ", fruit, "parent: ", parent)
    print("Depth: ", Z)


def get_realWorld_Cordinates():
    global pose
    global fruit_red
    global fruit_yellow
    global pub_rgb
    global depth_val_red
    global depth_val_yellow

    # get yellow fruit cordinates

    try:
        if(len(fruit_yellow) != 0):
            # TODO print("Yellow fruit detected")
            # print("fruit_yellow: ", fruit_yellow)
            # print("depth_val_yellow: ", depth_val_yellow)
            Xy, Yy, Zy = twoD_to_threeD(fruit_yellow[0][0], fruit_yellow[0][1], depth_val_yellow[0])
            tf_broadcaster(Xy, Yy, Zy, "yellow_bell_pepper", "camera_link")
    
    except Exception as e:

        print("Error in [YELLOW] get_realWorld_Cordinates: ", e)

    # get red fruit cordinates
    try:
        if(len(fruit_red) != 0):
            # TODO print("Red fruit detected")
            # print("fruit_red: ", fruit_red)
            # print("depth_val_red: ", depth_val_red)
            Xr, Yr, Zr = twoD_to_threeD(fruit_red[0][0], fruit_red[0][1], depth_val_red[0])
            tf_broadcaster(Xr, Yr, Zr, "red_bell_pepper", "camera_link")

    except Exception as e:
        print("Error in [RED] get_realWorld_Cordinates", e)

def twoD_to_threeD(x, y, depth):
    X = depth * ((x - cx) / fx)
    Y = depth * ((y - cy) / fy)
    Z = depth
    return X, Y, Z
##############################################################


def img_clbck(img_msg):

    global pub_rgb #, add global variable if any
    global pose
    global fruit_red
    global fruit_yellow

    ############################### Add your code here #######################################
    # image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    np_arr = np.frombuffer(img_msg.data, np.uint8)    # img_msg is the callback data
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # cv_image is the cv2 format image
    ##########################################################################################
    # cv2.imshow("new mask", cv_image)
    # cv2.waitKey(20)

    fruit_red, fruit_yellow  = image_processing(cv_image)
    # pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):

    global depth_val_red
    depth_val_red = []
    global depth_val_yellow
    depth_val_yellow = []

    ############################### Add your code here #######################################
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    for i in range(len(fruit_red)):
        depth_val_red.append(depth[fruit_red[i][1], fruit_red[i][0]]/1000)

    for i in range(len(fruit_yellow)):
        depth_val_yellow.append(depth[fruit_yellow[i][1], fruit_yellow[i][0]]/1000)

    # print("depth_val_red:", depth_val_red)
    # print("depth_val_yellow:", depth_val_yellow)

    ##########################################################################################
    try:
        get_realWorld_Cordinates()
    except:
        print("Error in get_realWorld_Cordinates")
        pass
    # cv2.imshow("new mask", depth)
    # cv2.waitKey(20)

def image_processing(image):

    fruit_red = []
    fruit_yellow = []

    ############### Write Your code to find centroid of the bell peppers #####################
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    hsvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    output = image.copy()

    # yellow_lower = np.array([8, 86, 49], np.uint8)
    # yellow_upper = np.array([16, 228, 255], np.uint8)
    # yellow_lower = np.array([6, 133, 98], np.uint8)
    # yellow_lower = np.array([9, 110, 86], np.uint8)
    # yellow_upper = np.array([16, 219, 255], np.uint8)
    
    # yellow_lower = np.array([9, 153, 156], np.uint8)
    # yellow_upper = np.array([23, 255, 255], np.uint8)
    yellow_lower = np.array([5, 129, 123], np.uint8)
    yellow_upper = np.array([23, 255, 255], np.uint8)

    # red_lower = np.array([0, 76, 39], np.uint8)
    # red_upper = np.array([14, 219, 225], np.uint8)
    # red_lower = np.array([0, 59, 0], np.uint8)
    # red_upper = np.array([0, 255, 225], np.uint8)
    
    # red_lower = np.array([165, 148, 124], np.uint8)
    # red_upper = np.array([179, 255, 225], np.uint8)
    red_lower = np.array([154, 99, 85], np.uint8)
    red_upper = np.array([179, 255, 225], np.uint8)

    # Threshold the HSV image to get only yellow colors
    yellow_mask = cv2.inRange(hsvImage, yellow_lower, yellow_upper)
    red_mask = cv2.inRange(hsvImage, red_lower, red_upper)
    newMask = yellow_mask | red_mask

    #define kernel size  
    kernel = np.ones((7,7),np.uint8)

    # Remove unnecessary noise from mask
    newMask = cv2.morphologyEx(newMask, cv2.MORPH_CLOSE, kernel)
    newMask = cv2.morphologyEx(newMask, cv2.MORPH_OPEN, kernel)

    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    contours_yellow, hierarchy_yellow = cv2.findContours(yellow_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy_red = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print("len(contours_red):", len(contours_red))
    # print("len(contours_yellow):", len(contours_yellow))

    # loop over the contours
    for c in contours_yellow:
        if cv2.contourArea(c) > 500:
            # compute the center of the contour
            # print("Yellow_Area:", cv2.contourArea(c))
            M = cv2.moments(c)
            centroidX = int(M["m10"] / M["m00"])
            centroidY = int(M["m01"] / M["m00"])

            fruit_yellow.append([centroidX, centroidY])
            # draw the contour and center of the shape on the image
            output = cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            output = cv2.circle(image, (centroidX, centroidY), 7, (255, 255, 255), -1)
            output = cv2.putText(image, "Yellow", (centroidX - 20, centroidY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    for c in contours_red:
        if cv2.contourArea(c) > 500:
            # compute the center of the contour
            # print("Red_Area:", cv2.contourArea(c))
            M = cv2.moments(c)
            centroidX = int(M["m10"] / M["m00"])
            centroidY = int(M["m01"] / M["m00"])

            fruit_red.append([centroidX, centroidY])
            # draw the contour and center of the shape on the image
            output = cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            output = cv2.circle(image, (centroidX, centroidY), 7, (255, 255, 255), -1)
            output = cv2.putText(image, "Red", (centroidX - 20, centroidY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # print("detected Yellow: ", fruit_yellow)
    # print("detected Red: ", fruit_red)

    # rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck)
    
    # print(sub_image_depth_1)

    # while cv2.waitKey(1):
    #     cv2.imshow("new mask", output)

    ##########################################################################################
    cv2.imshow("new mask", output)
    cv2.waitKey(20)
    return fruit_red, fruit_yellow


def main():
    '''
    MAIN FUNCTION

    Purpose:
    -----
    Initialize ROS node and do the publish and subscription of data.

    NOTE: We have done the subscription only for one image, you have to iterate over 
    three images in the same script and publish the centroid and depth in the 
    same script for three images, calling the same callback function.

    '''

    #### EDIT YOUR CODE HERE FOR SUBSCRIBING TO OTHER TOPICS AND TO APPLY YOUR ALGORITHM TO PUBLISH #####
    # rospy.sleep(15)
    rospy.init_node("percepStack", anonymous=True)
    print("Initialized Perception Stack Script")
    rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, img_clbck)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_clbck)
    rospy.spin()
    ####################################################################################################


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")