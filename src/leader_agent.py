#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from random import seed, random, randint

 
class LeaderAgent():


	def odometry_callback(self, data):
		self.positionX = data.pose.pose.position.x
		self.positionY = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w

	def getPosition(self):
		return self.positionX, self.positionY

	def calculateRandomVelocity(self):

		self.duration = randint(10, 50); #duration of one robot behavior in steps
		self.currentVelocity = self.angularVelocities[randint(0, 3)];

	def moveAgentRandomly(self):

		twistToPublish = Twist()
		twistToPublish.linear.x = 0.3
		twistToPublish.angular.z = self.currentVelocity
		if self.duration == 0:
			self.calculateRandomVelocity()
		else:
			self.duration -= 1
		self.pub.publish(twistToPublish)




	def __init__(self):

		self.pub = rospy.Publisher("/sphero2/cmd_vel", Twist, queue_size = 1 )
		rospy.sleep(0.2)

		rospy.Subscriber("/sphero2/odom", Odometry, self.odometry_callback, queue_size = 1)

		self.angularVelocities = [-1 ,-0.75, 0.75, 1]

		self.currentVelocity = 0

		self.duration = 0

		self.positionX = 0

		self.positionY = 0




if __name__ == "__main__":
	rospy.init_node("leader_agent")
	try:
		la = LeaderAgent()
	except rospy.ROSInterruptException:
		pass
