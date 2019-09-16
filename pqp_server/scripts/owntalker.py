#!/usr/bin/env python

from __future__ import division
import math
import rospy
from pqp_server.srv import *
import random
import copy
import math
import numpy as np
import scipy.spatial
import matplotlib as mpl
from math import atan2
mpl.use('TkAgg')
import time
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from tf.transformations import euler_from_quaternion

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10) # 10hz
   
    x = ModelState()
    x.model_name = "piano2"
    x.pose = Pose()
	

    x.pose.orientation.y = 0
    x.pose.orientation.w = 1.0
    x.pose.orientation.x = 0
    x.pose.orientation.z = 0

    




    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        x.twist = Twist()
	x.twist.linear.y = 0
	x.twist.angular.y=0
    	
        
	
       	x.pose.position.x = 10
        x.pose.position.y = 9
        x.pose.position.z = 0.7
        #angle_to_turn = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])*math.pi/180
        x.pose.orientation.z =  random.uniform(0, 1)
        rot_q = x.pose.orientation
        x.twist.linear.x = 0.0
        x.twist.angular.z = 0.0
        x.twist.linear.x = 0.4
        x.twist.angular.z = 0.0



	    #x.twist = Twist()
	    #x.twist.linear.x = randint(-60, 60)*math.pi/180
        rospy.loginfo(x)
        pub.publish(x)
        time.sleep(0.8)
	   # print "one loop done"
	    
	
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



