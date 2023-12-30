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
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import tf
import math
# You can add more if required
##############################################################


# Initialize Global variables
# pub_rgb = rospy.Publisher('/center_rgb', String, queue_size = 1)
# pub_depth = rospy.Publisher('/center_depth', String, queue_size = 1)
bridge = CvBridge()
pose = []
fruit_red = []
fruit_yellow = []
depth_val_red = []
depth_val_yellow = []
bot_Status = ""
fx = 554.3827128226441
fy = 554.3827128226441
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
        if(Z> 0.4 and Z < 0.9):
            br.sendTransform((X, Y, Z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     fruit,
                     parent)
            bot_pub.publish("stop")
            rate.sleep()
        else:
            a = 1 +3
            # print("Depth: ", Z)

            # TODO print("Z is greater than 0.7")
    else:
        
        # if(Z > 0.7):
        #     br.sendTransform((X, Y, Z),
        #              tf.transformations.quaternion_from_euler(0, 0, 0),
        #              rospy.Time.now(),
        #              "LoL",
        #              parent)
        a = 1+2
    # print("Depth: ", Z)
        # TODO print("nan Detected in tf_broadcaster:")
        # print("X: ", X, "Y: ", Y, "Z: ", Z)
        # if(listener.frameExists(fruit)):
        #     if(listener.getLatestCommonTime(fruit, parent) != rospy.Time(0)):
        #         br.sendTransform((X, Y, Z),
        #              tf.transformations.quaternion_from_euler(0, 0, 0),
        #              rospy.Time.now(),
        #              fruit,
        #              parent)
        # else:
        #     br.sendTransform((X, Y, Z),
        #              tf.transformations.quaternion_from_euler(0, 0, 0),
        #              rospy.Time.now(),
        #              fruit,
        #              parent)
    # print("tf for ", fruit, " broadcasted")

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
            tf_broadcaster(Xy, Yy, Zy, "yellow_bell_pepper", "camera_depth_frame2")
    
    except Exception as e:

        print("Error in [YELLOW] get_realWorld_Cordinates: ", e)

    # get red fruit cordinates
    try:
        if(len(fruit_red) != 0):
            # TODO print("Red fruit detected")
            # print("fruit_red: ", fruit_red)
            # print("depth_val_red: ", depth_val_red)
            Xr, Yr, Zr = twoD_to_threeD(fruit_red[0][0], fruit_red[0][1], depth_val_red[0])
            tf_broadcaster(Xr, Yr, Zr, "red_bell_pepper", "camera_depth_frame2")

    except Exception as e:
        print("Error in [RED] get_realWorld_Cordinates", e)

def twoD_to_threeD(x, y, depth):
    X = depth * ((x - cx) / fx)
    Y = depth * ((y - cy) / fy)
    Z = depth
    return X, Y, Z
##############################################################


def img_clbck(img_msg):
    '''
    Callback Function for RGB image topic

    Purpose:
    -----
    Convert the image in a cv2 format and then pass it 
    to image_processing function by saving to the 
    'image' variable.

    Input Args:
    -----
    img_msg: Callback message.
    '''

    global pub_rgb #, add global variable if any
    global pose
    global fruit_red
    global fruit_yellow

    ############################### Add your code here #######################################
    image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    ##########################################################################################

    fruit_red, fruit_yellow  = image_processing(image)
    # pub_rgb.publish(str(pose))

def depth_clbck(depth_msg):
    '''
    Callback Function for Depth image topic

    Purpose:
	--- 
    1. Find the depth value of the centroid pixel returned by the
    image_processing() function.
    2. Publish the depth value to the topic '/center_depth'


    NOTE: the shape of depth and rgb image is different. 
    
    Input Args:
    -----
    depth_msg: Callback message.
    '''
    global depth_val_red
    depth_val_red = []
    global depth_val_yellow
    depth_val_yellow = []

    ############################### Add your code here #######################################
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    for i in range(len(fruit_red)):
        depth_val_red.append(depth[fruit_red[i][1], fruit_red[i][0]])

    for i in range(len(fruit_yellow)):
        depth_val_yellow.append(depth[fruit_yellow[i][1], fruit_yellow[i][0]])


    
    # print("depth_val_red:", depth_val_red)
    # print("depth_val_yellow:", depth_val_yellow)

    global pub_depth
    ##########################################################################################
    try:
        get_realWorld_Cordinates()
    except:
        print("Error in get_realWorld_Cordinates")
        pass
    # cv2.imshow("new mask", depth)
    # cv2.waitKey(20)

def image_processing(image):
    '''
    NOTE: Do not modify the function name and return value.
          Only do the changes in the specified portion for this
          function.
          Use cv2.imshow() for debugging but make sure to REMOVE it before submitting.
    
    1. Find the centroid of the bell pepper(s).
    2. Add the x and y values of the centroid to a list.  
    3. Then append this list to the pose variable.
    3. If multiple fruits are found then append to pose variable multiple times.

    Input Args:
    ------
    image: Converted image in cv2 format.

    Example:
    ----
    pose = [[x1, y1] , [x2, y2] ...... ]
    '''
    fruit_red = []
    fruit_yellow = []

    ############### Write Your code to find centroid of the bell peppers #####################
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    hsvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    output = image.copy()

    # yellow_lower = np.array([8, 86, 49], np.uint8)
    # yellow_upper = np.array([16, 228, 255], np.uint8)
    # yellow_lower = np.array([6, 133, 98], np.uint8)
    yellow_lower = np.array([9, 110, 86], np.uint8)
    yellow_upper = np.array([16, 219, 255], np.uint8)

    # red_lower = np.array([0, 76, 39], np.uint8)
    # red_upper = np.array([14, 219, 225], np.uint8)
    red_lower = np.array([0, 59, 0], np.uint8)
    red_upper = np.array([0, 255, 225], np.uint8)

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

    # loop over the contours
    for c in contours_yellow:
        if cv2.contourArea(c) > 1000:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            fruit_yellow.append([cX, cY])
            # draw the contour and center of the shape on the image
            output = cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            output = cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            output = cv2.putText(image, "Yellow", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    for c in contours_red:
        if cv2.contourArea(c) > 1000:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # fruit_red.append([cX, cY])
            # draw the contour and center of the shape on the image
            output = cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            output = cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            output = cv2.putText(image, "Red", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # print("detected Yellow: ", fruit_yellow)
    # print("detected Red: ", fruit_red)



    rospy.Subscriber("/camera/depth/image_raw2", Image, depth_clbck)
    
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
    # sub_image_color_1 = rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
    rospy.Subscriber("/camera/color/image_raw2", Image, img_clbck)
    rospy.spin()
    ####################################################################################################


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print("Error:", str(e))
    finally:
        print("Executed Perception Stack Script")