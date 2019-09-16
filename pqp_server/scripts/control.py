#!/usr/bin/env python

import rospy
from gazebo_msgs import ModelStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import point, twist
from math import atan2


x = 0.0
y = 0.0
theta = 0.0


def newState (msg):
	global x
	global y
	global theta

	x = msg.pose.position.x
	y = msg.pose.position.y
	
	rot_q = msg.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node ("speed_controller")

sub = rospy.Subscriber("/gazebo/model_states", ModelStates, newState)
pub = rospy.Publisher("/gazebo/model_states", ModelStates, queue_size = 1)


speed = Twist()

r = rospy.Rate(4)


goal = Point ()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
	inc_x = goal.x - x
	inc_y = goal.y - y

	angle_to_goal = atan2 (inc_y, inc_x)
	
	if abs(angle_to_goal - theta) > 0.1:
		speed.linear.x = 0.0
		speed.angular.z = 0.3
	else:
		speed.linear.x  = 0.5
		speed.angular.z = 0.0

	pub.publish(speed)
	r.sleep()

