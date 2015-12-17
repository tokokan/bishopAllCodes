#!/usr/bin/env python

import rospy
import math
from cmvision.msg import Blobs, Blob
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

lastError = 0.0
state = 0
turn = 1
odomDist = 0.0
odomDegree = 0
odomPub = 0
theta1 = 0.0
theta2 = 0.0
theta3 = 0.0
theta4 = 0.0
theta5 = 0.0
yB = 0.0
xB = 0.0
xS = 0.0
flag = True

def blobsCallback(data):
        global pub, state, turn, lastError, odomPub, odomDegree, odomDist, theta1, theta2, theta3, theta4, theta5, xB, yB, xS
        
        #print("State: %i", state)
        command = Twist()
        x= 0
        y= 0
        area = 0
        
        command.angular.z = 0
        command.linear.x = 0
        if state == 0 and data.blob_count > 0:
                for box in data.blobs:
                        if box.name == 'Green':
                                area = area + box.area
                                x = x + (box.x * box.area)
                                y = y + (box.y * box.area)
                if area != 0:                
                        x = x / area
                        y = y / area

                if area == 0:
                        command.linear.x = 0
                        state = 1
                        pub.publish(command)
                        raw_input("Please pull the lid up")
                        odomPub.publish(Empty())
                        state = 2
                else:
                        command.linear.x = 0.2
                #if (area > 7000):
                #        command.linear.x = 0.2
                #elif (area < 4000):
                #        command.linear.x = -0.2
                #
                        error = ( data.image_width/2.0 - x) / (data.image_width/2.0)
                        KP = 2
                        derivative = 0.0
                        KD = .5
                
                        derivative = error-lastError
                        #if error < 0 :
                        #        derivative = -derivative

                        if (  (data.image_width / 2.3) < x < (data.image_width / 1.7) ):
                                command.angular.z = 0
                        else:
                                command.angular.z= error*KP + KD*derivative
                #elif(x < (data.image_width / 2.3)):
                #        command.angular.z = 1                       
                #elif(x > data.image_width / 1.7):
                 #       command.angular.z = -1
                        lastError = error
        elif state == 2:
                for box in data.blobs:
                        if box.name == 'Orange':
                                area = area + box.area
                                x = x + (box.x * box.area)
                                y = y + (box.y * box.area)
                if area != 0:                
                        x = x / area
                        y = y / area
                if area != 0 and (data.image_width / 2.1) < x < (data.image_width / 1.9):
                        theta1 = odomDegree
                        print(theta1)
                        state = 3
                else:
                        if (odomDegree >= 90 and turn == 1) or (odomDegree <= -90 and turn == -1):
                                        turn = -1 * turn
                        if (odomDegree >= 80 or odomDegree <= -80):
                                        command.angular.z = turn * .15
                        else: 
                                command.angular.z = turn * .3
        elif state == 3:
             for box in data.blobs:
                        if box.name == 'Red':
                                area = area + box.area
                                x = x + (box.x * box.area)
                                y = y + (box.y * box.area)
             if area != 0:                
                     x = x / area
                     y = y / area 
             if area != 0 and (data.image_width / 2.1) < x < (data.image_width / 1.9):
                        theta2 = odomDegree
                        print(theta2)
                        state = 4
             else:
                     if (odomDegree >= 90 and turn == 1) or (odomDegree <= -90 and turn == -1):
                                turn = -1 * turn
                     if (odomDegree >= 80 or odomDegree <= -80):
                                command.angular.z = turn * .15
                     else:
                                command.angular.z = turn * .3            
        elif state == 4:
                #if theta2 > 0:
                 #       error = odomDegree - 90
                  #      command.angular.z = .3
                   #     if math.fabs(error) < 2:
                   #             state = 5
                   #             command.angular.z = 0
                   #             odomPub.publish(Empty())
               # elif theta2 <= 0:
                error = odomDegree + 90
                command.angular.z = -.3
                if math.fabs(error) < 1:
                        state = 5
                        command.angular.z = 0
                        odomPub.publish(Empty())
        elif state == 5:
                if theta2 > 0:
                        dist = -.3
                else:
                        dist = .3
                error = dist - odomDist
                command.linear.x = dist/3
                if math.fabs(error) < .05:
                        state = 6
                        command.linear.x = 0
                        odomPub.publish(Empty())
                        turn = 1
        elif state == 6:
                for box in data.blobs:
                        if box.name == 'Orange':
                                area = area + box.area
                                x = x + (box.x * box.area)
                                y = y + (box.y * box.area)
                if area != 0:                
                        x = x / area
                        y = y / area
                if area != 0 and (data.image_width / 2.1) < x < (data.image_width / 1.9):
                        theta3 = odomDegree
                        print(theta3)
                        state = 7
                else:
                        if (odomDegree <= 2 and turn < 0) or (odomDegree >= 178 and turn > 0):
                                turn = -1 * turn
                        if (odomDegree <= 10 or odomDegree >= 170):
                                command.angular.z = turn * .15
                        else:
                                command.angular.z = turn * .3
        elif state == 7:        
                for box in data.blobs:
                        if box.name == 'Red':
                                area = area + box.area
                                x = x + (box.x * box.area)
                                y = y + (box.y * box.area)
                if area != 0:                
                        x = x / area
                        y = y / area
                if area != 0 and (data.image_width / 2.1) < x < (data.image_width / 1.9):
                        theta4 = odomDegree
                        print(theta4)
                        state = 12
                else:
                        if (odomDegree <= 2 and turn < 0) or (odomDegree >= 178 and turn > 0):
                                turn = -1 * turn
                        if (odomDegree <= 10 or odomDegree >= 170):
                                command.angular.z = turn * .15
                        else:
                                command.angular.z = turn * .3
        elif state == 8:
                temptheta3 = math.pi / 180 * theta3
                temptheta4 = math.pi / 180 * theta4
                if theta2 > 0:
                        dist = -.3
                else:
                        dist = .3
                temptheta1 = math.pi / 180 * (90 + theta1)
                temptheta2 = math.pi / 180 * (90 + theta2)
 
                xG = dist * math.tan(temptheta3)/(math.tan(temptheta3) - math.tan(temptheta1))
                yG = xG * math.tan(temptheta1)
                xB = dist * math.tan(temptheta4)/(math.tan(temptheta4) - math.tan(temptheta2))
                yB = xB * math.tan(temptheta2)
                
                m = (yG - yB)/(xG - xB)
                b = yG - m * xG

                xS = -b / m

                theta5 = - (180 / math.pi * math.atan(yG/(xS - xG)) )
                xS = xS - dist
                
                print(xG, yG, xB, yB, xS, theta5)
                #return
                state = 9;

        elif state == 9:
                error = xS - odomDist
                command.linear.x = error
                if math.fabs(error) < .05:
                        state = 10
                        odomPub.publish(Empty())
        elif state == 10:
                #error = theta5 - odomDegree
                
                #if math.fabs(error) < 1:
                #        command.angular.z = 0
                #        state = 11
                #        odomPub.publish(Empty())
                #else:
                #        command.angular.z = .3

                for box in data.blobs:
                        if box.name == 'Red':
                                area = area + box.area
                                x = x + (box.x * box.area)
                                y = y + (box.y * box.area)
                if area != 0:                
                        x = x / area
                        y = y / area
                if area != 0 and (data.image_width / 2.1) < x < (data.image_width / 1.9):
                        command.angular.z = 0
                        odomPub.publish(Empty())
                        state = 11
                else:
                        if (odomDegree <= 2 and turn < 0) or (odomDegree >= 178 and turn > 0):
                                turn = -1 * turn
                        if (odomDegree <= 10 or odomDegree >= 170):
                                command.angular.z = turn * .15
                        else:
                                command.angular.z = turn * .3
        elif state == 11:
                global flag
                command.linear.x = .5
                if theta2 > 0:
                        dist = -.3
                else:
                        dist = .3
                if flag:
                        xS = xS + dist
                        flag = False
                print(math.sqrt((xS - xB) * (xS - xB) + yB * yB))             
                if odomDist >= math.sqrt((xS - xB) * (xS - xB) + yB * yB) + .05:
                        command.linear.x = 0
                        pub.publish(command)
                        state = 1
        elif state == 12:
                error = odomDegree
                if math.fabs(error) < 1:
                        command.angular.z = 0
                        state = 8
                        odomPub.publish(Empty())
                elif math.fabs(error) < 10:
                        command.angular.z = .15
                else:
                        command.angular.z = .3
                
        pub.publish(command)
                
        
        #print "(%i, %i, %i)" % (x, y, area)        
def odomCallback(data):
        global odomDegree
        global odomDist
        global odomPub
        q = [data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w]
	roll, pitch, yaw = euler_from_quaternion(q)
    	odomDegree = yaw * 180 / math.pi
        odomDist = data.pose.pose.position.x      

def detect_blob():
        global pub
        global odomPub
        rospy.init_node('blob_tracker', anonymous = True)
        rospy.Subscriber('/blobs', Blobs, blobsCallback)
        rospy.Subscriber('/odom', Odometry, odomCallback)
        odomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)  
        pub = rospy.Publisher('kobuki_command', Twist, queue_size = 10)
        rospy.spin()


if __name__ == '__main__':
        detect_blob()
