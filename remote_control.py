#!/usr/bin/env python

import rospy
import math
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

isWaiting = 0
command = Twist()
command2 = Twist()
mode = 0
state = 0
deltatime = 0
curtime = 0

def send_commands():
    global pub
    global isWaiting
    global command #the value of this change as we accelerate/decelerate
    global command2 #This is what the user type in, value don't change
    global mode
    commandMode = 0
    rawinput = 0

    START_SPEED = 0.05

    rospy.init_node('remote_control', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odomCallback)
    odomPub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    #I changed the while loop condition from 'and' into 'or', it might be the reason why
    #when constant_command isn't running, resetting odom works fine
    while pub.get_num_connections() == 0 or odomPub.get_num_connections() == 0:
        pass

    #Tweaking a bit, to make code cleaner
    #I think its better to reset the odom after user input direction
    while not rospy.is_shutdown():

        #commandMode = raw_input("Select mode (1 for single, 2 for multiple): ")
        '''        
        if int(commandMode) == 1:
                mode, vel, distance = raw_input("please enter a command: ").split()
        else:
        '''
        rawinput = raw_input("please enter one or more commands: ").split(",")
        
        #rawinput = rawinput.split(",")
        
        
        for i in rawinput:
                odomPub.publish(Empty())
                rospy.sleep(1)
       
	        print "sending command"
                mode, vel, distance = i.split()
	        if mode == 'F':
		        command.linear.x = START_SPEED
		        command.angular.z = 0
                        command2.linear.x = float(vel)
		        command2.angular.z = 0
        	elif mode == 'B':
                        command.linear.x = -START_SPEED
        		command.angular.z = 0
        		command2.linear.x = -float(vel)
        		command2.angular.z = 0
        	elif mode == 'R':
        		command.linear.x = 0
        		command.angular.z = -START_SPEED
                        command2.linear.x = 0
        		command2.angular.z = -float(vel)
        	elif mode == 'L':
        		command.linear.x = 0
        		command.angular.z = START_SPEED
                        command2.linear.x = 0
        		command2.angular.z = float(vel)
                #add S mode to reset the odom mannually, we still has to but in 2 other random numbers tho
                elif mode == 'S':
                        #odomPub.publish(Empty())
                        #rospy.sleep(1)
                        continue

                command.linear.z = float(distance)

	        pub.publish(command)

	        isWaiting = 1
	
    	        while isWaiting:
		        pass

                pub.publish(command)

	
def bumperCallback(data):
	global pub
        global isWaiting
        global command
	if data.state == 1:
		print 'collision detected, sending stop command!\n'
		command = Twist()
		command.linear.x = 0
		command.angular.z = 0

		pub.publish(command)

                isWaiting = 0 #if it hit stuff, back to receving more commands
		rospy.sleep(1)
'''
I've been messing up with accelerating& decelerating using 2 commands variables,
one contains what the user type in, what is the actual speed during speeding up/slowing down
'''
def odomCallback(data):
	global isWaiting
	global command
	global mode
        global command2
        global state
        global deltatime
        global curtime

        SPEED_DELTA = 0.005
        SPEED_DELTA2 = 0.0075
        DECELERATE_AT = 0.5
        MINIMUM_SPEED = 0.1
        MAX_ACCEL = .5
        
        #do we need diferent SPEED_DELTA, DECELERATE_AT, MINIMUM_SPEED for turning?
        #turning use different speeds than movin forward/backward

	q = [data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w]
	roll, pitch, yaw = euler_from_quaternion(q)
    	degree = yaw * 180 / math.pi

        if isWaiting == 1:
		print degree, command.linear.z, mode
		if mode == 'F':
                        if data.pose.pose.position.x >= command.linear.z:                              	        
                                print "sending stop"		
			        command.linear.x = 0.0
			        command.angular.z = 0.0
			        isWaiting = 0
                        elif data.pose.pose.position.x >= command.linear.z*DECELERATE_AT:
                                if command.linear.x - SPEED_DELTA2 >= MINIMUM_SPEED:
                                        print "decreasing speed!"
                                        command.linear.x -= SPEED_DELTA2
                                        pub.publish(command)                                   
                        else:
                                if command.linear.x < command2.linear.x:
                                        print "increasing speed"
                                        command.linear.x += SPEED_DELTA 
                                        pub.publish(command)
                                      
                                 
		elif mode == 'B':
                        if data.pose.pose.position.x <= -command.linear.z: 
			        print "sending stop"		
			        command.linear.x = 0.0
			        command.angular.z = 0.0
			        isWaiting = 0
                        elif data.pose.pose.position.x <= -(command.linear.z*DECELERATE_AT):
                                if command.linear.x + SPEED_DELTA2 <= -MINIMUM_SPEED:
                                        print "decreasing speed!"
                                        command.linear.x += SPEED_DELTA2
                                        pub.publish(command)
                        else:
                                if command.linear.x > command2.linear.x:
                                        print "increasing speed!"
                                        command.linear.x -= SPEED_DELTA
                                        pub.publish(command)

		elif mode == 'R':
                        if degree <= -command.linear.z:
			        print "sending stop"		
			        command.linear.x = 0.0
			        command.angular.z = 0.0
			        isWaiting = 0
                        elif degree <= -(command.linear.z*DECELERATE_AT):
                                if command.angular.z + SPEED_DELTA2 <= - MINIMUM_SPEED:
                                        print "decreasing speed!"
                                        command.angular.z +=SPEED_DELTA2
                                        pub.publish(command)
                        else:
                                if command.angular.z > command2.angular.z:
                                        print "increasing speed!"
                                        command.angular.z -=SPEED_DELTA
                                        pub.publish(command)
		elif mode == 'L':
                        if degree >= command.linear.z:
			        print "sending stop"		
			        command.linear.x = 0.0
			        command.angular.z = 0.0
			        isWaiting = 0
                        elif degree >= (command.linear.z*DECELERATE_AT):
                                if command.angular.z - SPEED_DELTA2 >= MINIMUM_SPEED:
                                        print "decreasing speed!"
                                        command.angular.z -= SPEED_DELTA2
                                        pub.publish(command)
                        else:
                                if command.angular.z < command2.angular.z:
                                        print "increasing speed!"
                                        command.angular.z += SPEED_DELTA
                                        pub.publish(command)

def initBumperDetection():
	rospy.Subscriber('mobile_base/events/bumper',BumperEvent,bumperCallback)

pub = rospy.Publisher('kobuki_command', Twist, queue_size=10)

if __name__ == '__main__':
    try:
	initBumperDetection()
        send_commands()
    except rospy.ROSInterruptException:
        pass
