#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from struct import unpack

import math
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

depthData = Image();
isDepthReady = False;
dist = 0.0
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def depthCallback(data):
    global depthData, isDepthReady, dist, pub
    command = Twist()
    depthData = data
    isDepthReady = True

    

    step = depthData.step
    midX = 320
    midY = 240
    
    places = [40, 100, 160, 220, 260, 320, 380, 440, 500,560, 620]
    print "Distance: ",
    for midX in places:
        offset = (240 * step) + (midX * 4)
        (dist,) = unpack('f', depthData.data[offset] + depthData.data[offset+1] + depthData.data[offset+2] + depthData.data[offset+3])
        if (dist < 1.0) and (not math.isnan(dist)):
                command.linear.x = 0
                command.angular.z = 0.5
                break
        elif (dist >=1.0) or (math.isnan(dist)):
                command.linear.x  = 0.2
                command.angular.z = 0
        #print '%.3f' %dist,
    #print

        
    pub.publish(command)
    

def main():
    global depthData, isDepthReady
    rospy.init_node('depth_example', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, depthCallback, queue_size=10)

    while not isDepthReady and not rospy.is_shutdown():
        pass

    while not rospy.is_shutdown():
        pass    
        

if __name__ == '__main__':
    main()
