#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from cmvision.msg import Blobs, Blob

import math
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


isDepthReady = False
isWaiting = False
depthData = Image()
dist = 0.0
prevDistError = 0.0
prevCommandX = 0.0
prevAngError = 0.0
colorImage = Image()
isColorImageReady = False
isTaking = False
working = 0
prevAng = 0


def depthCallback(data):
    global depthData, isDepthReady, dist, pub
    command = Twist()
    depthData = data
    isDepthReady = True

def updateColorImage(data):
    global colorImage, isColorImageReady
    colorImage = data
    isColorImageReady = True


def bumperCallback(data):
	global pub, prevCommandX,prevAngError, isWaiting
        
        if data.state == BumperEvent.PRESSED and isWaiting == False:
                isWaiting = True                
                os.system('spd-say \"oh shit\"')
	        command = Twist()
	        command.linear.x = 0
	        command.angular.z = 0
                prevCommandX = 0
                prevAngError = 0.0
	        pub.publish(command)

	        rospy.sleep(3)
                isWaiting = False
        else:
                os.system('spd-say \"I\'m okay. I\'m okay\"')

def blobsCallback(data):
        global prevCommandX,pub, blobpub, depthData, isDepthReady,dist,prevDistError,prevAngError, isTaking, prevAng, isWaiting

        if isTaking == False and isWaiting == False:
                isDepthReady = False                
                theX = 0
                theY = 0
                getOutState = 0
                command = Twist()
                for i in range(data.blob_count):
                        if data.blobs[i].name == 'Pink':
                                for j in range(data.blob_count):
                                        if data.blobs[j].name == "Green":                                   
                                                if (data.blobs[i].left < data.blobs[j].left) and (data.blobs[i].right > data.blobs[j].right)and (data.blobs[i].top < data.blobs[j].top) and (data.blobs[i].bottom > data.blobs[j].bottom):
                                                        blobpub.publish(data.blobs[i])                                               
                                                        theX = data.blobs[j].x
                                                        theY = data.blobs[j].y
                                                        getOutState = 1
                                                        break
                                        if data.blobs[j].name == "Orange":                                   
                                                if (data.blobs[i].left < data.blobs[j].left) and (data.blobs[i].right > data.blobs[j].right)and (data.blobs[i].top < data.blobs[j].top) and (data.blobs[i].bottom > data.blobs[j].bottom):
                                                        blobpub.publish(data.blobs[i])                                               
                                                        theX = data.blobs[j].x
                                                        theY = data.blobs[j].y
                                                        getOutState = 2
                                                        isTaking = True
                                                        break
                        if getOutState>0:
                                break
                
                if getOutState == 1:
                        while not isDepthReady:
                                pass
                
                        step = depthData.step
                        offset = (theY * step) + (theX * 4)
                        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
                

                        if(math.isnan(dist)):
                                if prevDistError <= -2.8 or prevDistError >= 2.8:
                                        print "wait for me!"
                                        command.linear.x = prevCommandX
                                else:
                                        if prevCommandX == 0:
                                                command.linear.x = 0
                                        else: 
                                                command.linear.x = prevCommandX - (prevCommandX/abs(prevCommandX))* .01
                        else:                
                                error =  (dist - 1.2 + prevDistError) / 2          #math.min(math.abs(2.05 - depth), math.abs(1.95 - depth)
                                derivative = error - prevDistError
                                KP = .75
                                KD = 1.25
                                if -.15 < error < .15:
                                        #if -0.1 < prevCommandX < 0.1:
                                        #        command.linear.x = 0
                                        #else:
                                        #        if prevCommandX == 0:
                                        #                command.linear.x = 0
                                        #        else: 
                                        #                command.linear.x = prevCommandX - (prevCommandX/abs(prevCommandX))* .01
                                        command.linear.x = 0
                                else:
                                        command.linear.x = error * KP + derivative * KD
                           
                                prevDistError = error

                                #stuff for turning 
                                error = ((data.image_width/2.0 - theX) / (data.image_width/2.0) + prevAngError) / 2
                                KP = 1
                                derivative = 0.0
                                KD = .5
                        
                                derivative = error- prevAngError
                                if (  (data.image_width / 2.15) < theX < (data.image_width / 1.85) ):
                                        command.angular.z = 0
                                else:
                                        command.angular.z= error*KP + KD*derivative
                                prevAngError = error
                elif getOutState == 2:
                        #stuff for taking photos
                        
                        bridge = CvBridge()
                        global colorImage
                        try:             
                                color_image = bridge.imgmsg_to_cv2(colorImage, "bgr8")
                                
                        except CvBridgeError, e:
                                print e

                        os.system('spd-say \"Three\"')
                        rospy.sleep(1)                

                        os.system('spd-say \"Two\"')
                        rospy.sleep(1)                

                        os.system('spd-say \"One\"')

                        rospy.sleep(1)                

                        os.system('spd-say \"Say Tree\"')

                        rospy.sleep(1)                


                        cv2.imwrite("/home/student/selfieBot.jpg",color_image,[cv2.IMWRITE_JPEG_QUALITY,100])         
                        
                        os.system('spd-say \"photo saved"')
                        isTaking = False
                else:
                        if prevCommandX > -.01 and prevCommandX < .01:
                                command.linear.x = 0
                        elif prevCommandX >= .01:
                                if prevCommandX == 0:
                                        command.linear.x = 0
                                else: 
                                        command.linear.x = prevCommandX - (prevCommandX/abs(prevCommandX))* .01
                        else:
                                if prevCommandX == 0:
                                        command.linear.x = 0
                                else: 
                                        command.linear.x = prevCommandX - (prevCommandX/abs(prevCommandX))* .01
                        command.angular.z = prevAng
                        if prevAng != 0 and command.angular.z < prevAng / abs(prevAng) * 1.5:
                                command.angular.z = command.angular.z + prevAng / abs(prevAng) * .05

                prevCommandX = command.linear.x
                prevAng = command.angular.z
                print getOutState, " | ", command.linear.x
                pub.publish(command)
                working = 0

                #print command.linear.x
'''
                if prevCommandX == 0 and abs(command.linear.x) > .05:
                        command.linear.x = command.linear.x / abs(command.linear.x) * .05
                elif command.linear.x == 0 and abs(prevCommandX) > .05:
                        command.linear.x = prevCommandX - prevCommandX/abs(prevCommandX) * .05 
                elif prevCommandX != 0 and command.linear.x != prevCommandX and abs(command.linear.x) > abs(prevCommandX + (command.linear.x - prevCommandX)/abs(command.linear.x - prevCommandX) * .05):
                        command.linear.x = prevCommandX + (command.linear.x - prevCommandX)/abs(command.linear.x - prevCommandX) * .05


                #if command.linear.x > 0.4:
                #        command.linear.x = 0.4
                #elif command.linear.x <-0.4:
                #        command.linear.x = -0.4
                
                if prevCommandX != 0 and command.linear.x != prevCommandX and abs(command.linear.x) > abs(prevCommandX + (command.linear.x - prevCommandX)/abs(command.linear.x - prevCommandX) * .001):
                        print getOutState
             
                prevCommandX = command.linear.x
                pub.publish(command)
                working = 0
'''
def detect_blob():
        global pub
        global blobpub
        global updateColorImage
        rospy.init_node('paparazzi', anonymous = True)
        rospy.Subscriber('/blobs', Blobs, blobsCallback,queue_size=10)
        rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)
        rospy.Subscriber('mobile_base/events/bumper',BumperEvent,bumperCallback)
        defaultImageTopic = "/camera/rgb/image_color"
        rospy.Subscriber(defaultImageTopic, Image, updateColorImage, queue_size=10)
        #rospy.Subscriber('/odom', Odometry, odomCallback)
        #odomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)  
        pub = rospy.Publisher('kobuki_command', Twist, queue_size = 10)
        #pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        blobpub = rospy.Publisher('/our_blobs', Blob, queue_size = 10)
        rospy.spin()


if __name__ == '__main__':
        detect_blob()
