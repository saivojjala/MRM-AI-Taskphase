#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher(linear_velocity, angular_velocity):
	pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
	rospy.init_node("publisher", anonymous=True)
	rate = rospy.Rate(10)
	velocity=Twist()

	while not rospy.is_shutdown():

		velocity.linear.x = linear_velocity
		velocity.linear.y = 0
		velocity.linear.z = 0

		velocity.angular.x = 0
		velocity.angular.y = 0
		velocity.angular.z = angular_velocity

		rospy.loginfo("Linear velocity = %f Angular velocity = %f", linear_velocity, angular_velocity)
		pub.publish(velocity)
		rate.sleep()

if __name__ == "__main__":
	publisher(3.0, 3.0)
	
