import rospy
import roslaunch
import time
import numpy as np
import math


import itertools

from random import seed, random, randint, choice

from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SetModelState

from nav_msgs.msg import Odometry
import tf
from std_srvs.srv import Empty


class AgentPosController():
    '''
    This class control robot position
    We teleport our agent when environment reset
    So agent start from different position in every episode
    '''
    def __init__(self):
        self.agent_model_name = "sphero1"

    def teleportRandom(self):
        '''
        Teleport agent return new x and y point

        return agent posX, posY in list
        '''

        model_state_msg = ModelState()
        model_state_msg.model_name = self.agent_model_name
            # maze 1


        xy_list = [
            [0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]
        ]

        # Get random position for agent
        pose = Pose()
        pose.position.x, pose.position.y = choice(xy_list)

        model_state_msg.pose = pose
        model_state_msg.twist = Twist()

        model_state_msg.reference_frame = "world"

        # Start teleporting in Gazebo
        isTeleportSuccess = False
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
    
        return pose.position.x, pose.position.y



class TargetController:

    def __init__(self, targetPointX, targetPointY):

        self.positionX = 0
        self.positionY = 0

        self.model_path = "../models/gazebo/goal_sign/model.sdf"
        f = open(self.model_path, 'r')
        self.model = f.read()
        self.model_name = 'goal_sign'

        self.goal_position = Pose();
        self.goal_position.position.x = 0  # Initial positions
        self.goal_position.position.y = 0

        self.targetPointX = targetPointX
        self.targetPointY = targetPointY
        self.check_model = False



    def respawn (self):


        self.goal_position.position.x = self.targetPointX
        self.goal_position.position.y = self.targetPointY




        isTeleportSuccess = False;

        for i in range(5):
            if not self.check_model:  # This used to checking before spawn model if there is already a model
                try:
                    rospy.wait_for_service('gazebo/spawn_sdf_model')
                    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                    spawn_model_prox(self.model_name, self.model, 'robotos_name_space', self.goal_position, "world")
                    isSpawnSuccess = True
                    self.check_model = True
                    break
                except Exception as e:
                    rospy.logfatal("Error when spawning the goal sign " + str(e))
            else:
                rospy.logwarn("Trying to spawn goal sign ..." + str(i))
                time.sleep(2)
        
        if not isSpawnSuccess:
            rospy.logfatal("Error when spawning the goal sign")

    def deleteModel(self):
        '''
        Delete model from Gazebo
        '''
        while True:
            if self.check_model:
                try:
                    rospy.wait_for_service('gazebo/delete_model')
                    del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                    del_model_prox(self.model_name)
                    self.check_model = False
                    break
                except Exception as e:
                    rospy.logfatal("Error when deleting the goal sign " + str(e))
            else:
                break

class SpheroGymEnv():
    '''
    Main Gazebo environment class
    Contains reset and step function
    '''
    def __init__(self):
        # Initialize the node
        rospy.init_node('sphero_gym_env', anonymous=True)

        # Connect to gazebo
        self.velPub = rospy.Publisher('/sphero1/cmd_vel', Twist, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy(
            '/gazebo/reset_simulation', Empty)

        self.minCrashRange = 0.1  # Asume crash below this distance
        self.stateSize = 8  # argetAngle, distance
        self.actionSize = 9  # Size of the robot's actions

        self.targetDistance = 0  # Distance to target

        self.robotX = 0
        self.robotY = 0

        self.targetPointX = 0  # Target Pos X
        self.targetPointY = 0  # Target Pos Y

        self.obstaclePositions = np.zeros((3,2))

        # Means robot reached target point. True at beginning to calc random point in reset func
        self.isTargetReached = True
        self.isCrash = False
        self.agentController = AgentPosController()
        self.TargetController = TargetController(self.targetPointX, self.targetPointY)


    def pauseGazebo(self):
        '''
        Pause the simulation
        '''
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except Exception:
            print("/gazebo/pause_physics service call failed")

    def unpauseGazebo(self):
        '''
        Unpause the simulation
        '''
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except Exception:
            print("/gazebo/unpause_physics service call failed")

    def resetGazebo(self):
        '''
        Reset simualtion to initial phase
        '''
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except Exception:
            print("/gazebo/reset_simulation service call failed")


    def getOdomData(self):
        '''
        ROS callback function
        Modify odom data quaternion to euler

        return yaw, posX, posY of robot known as Pos2D
        '''
        try:
            odomData = rospy.wait_for_message('/sphero1/odom', Odometry, timeout=5)
            odomPoseData = odomData.pose.pose
            odomTwistData = odomData.twist.twist
            quat = odomPoseData.orientation
            quatTuple = (
                quat.x,
                quat.y,
                quat.z,
                quat.w,
            )
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                quatTuple)
            self.robotX = odomPoseData.position.x
            self.robotY = odomPoseData.position.y
            robotVelX = odomTwistData.linear.x
            robotVelY = odomTwistData.linear.y
            return yaw, self.robotX, self.robotY, robotVelX, robotVelY

        except Exception as e:
            rospy.logfatal("Error to get odom data " + str(e))



    def calcTargetAngle(self):
        '''
        Calculate angle from robot to target

        return angle in float [0, 2PI]


        '''
        targetAngle = math.atan2(self.targetPointY - self.robotY, self.targetPointX - self.robotX)

        if(targetAngle < 0):
            targetAngle = 2 * math.pi + targetAngle

        return round(targetAngle, 5)



    def calcDistance(self, x1, y1, x2, y2):
        '''
        Calculate euler distance of given two points

        return distance in float
        '''
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def calculateState(self, odomData):
        '''
        Calculate targetAngle angle
        Calculate distance to target

        returns state as np.array

        State contains:
        targetAngle, distance
        '''

        _, self.robotX, self.robotY, robotVelX, robotVelY = odomData
        targetAngle = self.calcTargetAngle()
        distance = self.calcDistance(
            self.robotX, self.robotY, self.targetPointX, self.targetPointY)

        # isCrash = False  # If robot hit to an obstacle



        obstaclePositionsToReturn = list(itertools.chain.from_iterable(self.obstaclePositions))


       # return [targetAngle, distance, obstacleMinRange, obstacleAngle], isCrash

        return [targetAngle, distance] +  obstaclePositionsToReturn


    def step(self, action):
        '''
        Act in envrionment
        After action return new state
        Calculate reward
        Calculate bot is crashed or not
        Calculate is episode done or not

        returns state as np.array

        State contains:
        targetAngle, distance, reward, done
        '''

        self.unpauseGazebo()



        # More basic actions
        
        if action == 0: #STAY
            velCmd = Twist()
            velCmd.linear.x = 0.0
            velCmd.linear.y = 0.0
            self.velPub.publish(velCmd)
        elif action == 1: #FORWARD
            velCmd = Twist()
            velCmd.linear.x = 1.0
            velCmd.linear.y = 0.0
            self.velPub.publish(velCmd)
        elif action == 2: #FORWARD-RIGHT
            velCmd = Twist()
            velCmd.linear.x = 0.7071
            velCmd.linear.y = 0.7071
            self.velPub.publish(velCmd)
        elif action == 3: #RIGHT
            velCmd = Twist()
            velCmd.linear.x = 0.0
            velCmd.linear.y = 1.0
            self.velPub.publish(velCmd)
        elif action == 4: #BACK-RIGHT
            velCmd = Twist()
            velCmd.linear.x = -0.7071
            velCmd.linear.y = 0.7071
            self.velPub.publish(velCmd)
        elif action == 5: #BACK
            velCmd = Twist()
            velCmd.linear.x = -1.0
            velCmd.linear.y = 0.0
            self.velPub.publish(velCmd)
        elif action == 6: #BACK-LEFT
            velCmd = Twist()
            velCmd.linear.x = -0.7071
            velCmd.linear.y = -0.7071
            self.velPub.publish(velCmd)   
        elif action == 7: #LEFT
            velCmd = Twist()
            velCmd.linear.x = 0.0
            velCmd.linear.y = -1.0
            self.velPub.publish(velCmd)     
        elif action == 8: #FOWARD-LEFT
            velCmd = Twist()
            velCmd.linear.x = 0.7071
            velCmd.linear.y = -0.7071
            self.velPub.publish(velCmd)      


        # Observe

        odomData = self.getOdomData()

        self.pauseGazebo()

        state = self.calculateState(odomData)

        done = False

        distanceToTarget = state[1]

        if distanceToTarget < 0.1:  # Reached to target
            self.isTargetReached = True


        for pos in self.obstaclePositions: #Crashed
            if (self.calcDistance(pos[0], pos[1], self.robotX, self.robotY) < self.minCrashRange):
                self.isCrash = True;



        if self.isTargetReached:
            # Reached to target
            rospy.logwarn("Reached to target!")
            reward = 50
            self.isTargetReached = False
        elif self.isCrash:
            #Crashed
             rospy.logwarn("Crash!")
             reward = -4000
             done = True
             self.isCrash = False
        else:
            # Neither reached to goal nor crashed calc reward for action
            yawReward = []
            currentDistance = state[1]
            targetAngle = state[0]

            yawReward.append(-1)  # small negative reward for staying in place

            for i in range(self.actionSize-1):
                 angle = math.fabs(targetAngle - float(i) * math.pi / 4.0)
                 actionsYawReward = 10 * ((1-(math.fabs((math.modf(angle / math.pi)[1] * (2.0 * math.pi)) - angle) / math.pi))-0.625)
                 actionsYawReward = round(actionsYawReward, 2)
                 yawReward.append(actionsYawReward)


            try:
                distanceRate = 2 ** (currentDistance / self.targetDistance)
            except Exception:
                print("Overflow err CurrentDistance = ", currentDistance, " TargetDistance = ", self.targetDistance)
                distanceRate = 2 ** (currentDistance // self.targetDistance)

            if(distanceRate < 2):
                distanceRate = 2
                
            reward = ((round(yawReward[action] * 5, 2)) * distanceRate)


            def actionSwitchToString(i):
                switcher = {
                    0: "STAY",
                    1: "FORWARD",
                    2: "FORWARD-RIGHT",
                    3: "RIGHT",
                    4: "BACK-RIGHT",
                    5: "BACK",
                    6: "BACK-LEFT",
                    7: "LEFT",
                    8: "FOWARD-LEFT"
                }

                return switcher.get(i, "INVALID")

            actionString = actionSwitchToString(action);

        #print("reward: ", reward, "   action: ", actionString, "distanceToTarget: ", distanceToTarget)
        #print("obstaclePositions: ", self.obstaclePositions);
        #print("state:", self.targetPointX, self.targetPointY);    




        return np.asarray(state), reward, done

    def reset(self):
        '''
        Reset the envrionment
        Reset bot position

        returns state as np.array
targetDistance
        State contains:
        targetAngle, distance
        '''
        self.resetGazebo()


        while True:
            # Teleport bot to a random point
            agentX, agentY = self.agentController.teleportRandom()
            if self.calcDistance(self.targetPointX, self.targetPointY, agentX, agentY) > self.minCrashRange:
                break
            else:
                rospy.logerr("Reteleporting the bot!")
                time.sleep(2)

        if self.isTargetReached:
            while True:
 #               self.targetPointX, self.targetPointY = self.goalCont.calcTargetPoint()
                #self.targetPointX, self.targetPointY = self.leaderAgent.getPosition()
                if self.calcDistance(self.targetPointX, self.targetPointY, agentX, agentY) > self.minCrashRange:
                    self.isTargetReached = False
                    break
                else:
                    rospy.logerr("Recalculating the target point!")
                    time.sleep(2)

        # Unpause simulation to make observation
        self.unpauseGazebo()
        odomData = self.getOdomData()
        self.pauseGazebo()

        state = self.calculateState(odomData)
        self.targetDistance = state[1]
        self.stateSize = len(state)

        return np.asarray(state)  # Return state
