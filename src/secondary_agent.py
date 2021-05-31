#!/usr/bin/env python
import rospy
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import SetModelState

from random import seed, random, randint,choice

 
class SecondaryAgent():


	
	def odometry_callback(self, data):
		self.positionX = data.pose.pose.position.x
		self.positionY = data.pose.pose.position.y
		self.robot_qua_x = data.pose.pose.orientation.x
		self.robot_qua_y = data.pose.pose.orientation.y
		self.robot_qua_z = data.pose.pose.orientation.z
		self.robot_qua_w = data.pose.pose.orientation.w

	def respawn (self):
		model_state_msg = ModelState()
		model_state_msg.model_name = "sphero{}".format(self.agentNumber)

		if(self.agentRandomPositionNumber == 1):
			xy_list = [[0, 0], [1, 0], [2, 0]]
		if(self.agentRandomPositionNumber == 2):
			xy_list = [[0, 1], [1, 1], [2, 1]]
		if(self.agentRandomPositionNumber == 3):
			xy_list = [[0, 2], [1, 2], [2, 2]]


		pose = Pose();
		pose.position.x, pose.position.y = choice(xy_list)

		model_state_msg.pose = pose
		model_state_msg.twist = Twist()

		model_state_msg.reference_frame = "world"


		isTeleportSuccess = False;

		for i in range(5):
			if not isTeleportSuccess:
				try:
					rospy.wait_for_service('/gazebo/set_model_state')
					telep_model_prox = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
					telep_model_prox(model_state_msg)
					isTeleportSuccess = True
					break
				except Exception as e:
					rospy.logfatal("Error when teleporting agent " + str(e))
			else:
				rospy.logwarn("Trying to teleporting agent..." + str(i))
				time.sleep(2)

		if not isTeleportSuccess:
			rospy.logfatal("Error when teleporting agent")
			return "Err", "Err"

		self.positionX = pose.position.x
		self.positionY = pose.position.y


	def getPosition(self):
		return self.positionX, self.positionY

	def calculateRandomVelocity(self):

		self.durationRandom = randint(10, 50); #durationRandom of one robot behavior in steps
		self.currentVelocity = self.angularVelocities[randint(0, 3)];

	def moveAgentRandomly(self):

		twistToPublish = Twist()
		twistToPublish.linear.x = 0.3
		twistToPublish.angular.z = self.currentVelocity
		if self.durationRandom == 0:
			self.calculateRandomVelocity()
		else:
			self.durationRandom -= 1
		self.pub.publish(twistToPublish)



	def moveAgentCircle(self):


		twistToPublish = Twist()
		twistToPublish.linear.x = 0.3;
		twistToPublish.angular.z = 0.5;

		self.pub.publish(twistToPublish)


	def __init__(self, agentNumber):

		self.agentNumber = agentNumber;
		self.agentRandomPositionNumber = agentNumber - 1;

		self.pub = rospy.Publisher("/sphero{}/cmd_vel".format(self.agentNumber), Twist, queue_size = 1 )
		rospy.sleep(0.2)

		rospy.Subscriber("/sphero{}/odom".format(self.agentNumber), Odometry, self.odometry_callback, queue_size = 1)

		self.angularVelocities = [-1 ,-0.75, 0.75, 1]

		self.currentVelocity = 0

		self.durationRandom = 0

		self.durationCircular = 0

		self.positionX = 0

		self.positionY = 0




if __name__ == "__main__":
	rospy.init_node("secondary_agent")
	try:
		la = SecondaryAgent()
	except rospy.ROSInterruptException:
		pass
