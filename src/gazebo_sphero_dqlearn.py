import rospy
import roslaunch
import time
import numpy as np
import math

from random import seed, random, randint, choice

from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState

from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

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
            [0, 0], [0, 1], [0, 2], [0, 3],
            [1, 0], [1, 1], [1, 2], [1, 3],
            [2, 0], [2, 1], [2, 2], [2, 3],
            [3, 0], [3, 1], [3, 2], [3, 3],
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

class GoalController():
    """
    This class controls target model and position
    """
    def __init__(self):
        self.model_path = "../models/gazebo/goal_sign/model.sdf"
        f = open(self.model_path, 'r')
        self.model = f.read()

        self.goal_position = Pose()
        self.goal_position.position.x = None  # Initial positions
        self.goal_position.position.y = None
        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        self.model_name = 'goal_sign'
        self.check_model = False  # This used to checking before spawn model if there is already a model

    def respawnModel(self):
        '''
        Spawn model in Gazebo
        '''
        isSpawnSuccess = False
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

    def calcTargetPoint(self):
        """
        This function return a target point randomly for robot
        """
        self.deleteModel()
        # Wait for deleting
        time.sleep(0.5)

        goal_xy_list = [
                [0.5, 0.5], [0.5, 1.5], [0.5, 2.5],
                [1.5, 0.5], [1.5, 1.5], [1.5, 2.5],
                [2.5, 0.5], [2.5, 1.5], [2.5, 2.5]
        ]

        # Check last goal position not same with new goal
        while True:
            self.goal_position.position.x, self.goal_position.position.y = choice(goal_xy_list)

            if self.last_goal_x != self.goal_position.position.x:
                if self.last_goal_y != self.goal_position.position.y:
                    break

        # Spawn goal model
        self.respawnModel()

        self.last_goal_x = self.goal_position.position.x
        self.last_goal_y = self.goal_position.position.y

        # Inform user
        rospy.logwarn("New goal position : " + str(self.goal_position.position.x) + " , " + str(self.goal_position.position.y))

        return self.goal_position.position.x, self.goal_position.position.y

    def getTargetPoint(self):
        return self.goal_position.position.x, self.goal_position.position.y



class LeaderAgent():


    def odometry_callback(self, data):

        self.positionX = data.pose.pose.position.x
        self.positionY = data.pose.pose.position.y

    def getPosition(self):
        return self.positionX, self.positionY


    def calculateRandomVelocity(self):

        self.duration = randint(10, 50); #duration of one robot behavior in steps
        self.currentVelocity = self.angularVelocities[randint(0, 3)];

    def moveAgentRandomly(self):

        twistToPublish = Twist()
        twistToPublish.linear.x = 0.5
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

        self.minCrashRange = 0.2  # Asume crash below this distance
        self.stateSize = 2  # argetAngle, distance
        self.actionSize = 9  # Size of the robot's actions

        self.targetDistance = 0  # Distance to target

        self.targetPointX = 0  # Target Pos X
        self.targetPointY = 0  # Target Pos Y

        # Means robot reached target point. True at beginning to calc random point in reset func
        self.isTargetReached = True
        self.goalCont = GoalController()
        self.agentController = AgentPosController()
        self.leaderAgent = LeaderAgent()


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
            robotX = odomPoseData.position.x
            robotY = odomPoseData.position.y
            robotVelX = odomTwistData.linear.x
            robotVelY = odomTwistData.linear.y
            return yaw, robotX, robotY, robotVelX, robotVelY

        except Exception as e:
            rospy.logfatal("Error to get odom data " + str(e))



    def calcTargetAngle(self, targetPointX, targetPointY, robotX, robotY):
        '''
        Calculate angle from robot to target

        return angle in float [0, 2PI]


        '''
        targetAngle = math.atan2(targetPointY - robotY, targetPointX - robotX)

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

        _, robotX, robotY, robotVelX, robotVelY = odomData
        targetAngle = self.calcTargetAngle(self.targetPointX, self.targetPointY, robotX, robotY)
        distance = self.calcDistance(
            robotX, robotY, self.targetPointX, self.targetPointY)

        # isCrash = False  # If robot hit to an obstacle


       # return [targetAngle, distance, obstacleMinRange, obstacleAngle], isCrash

        return [targetAngle, distance]


    def step(self, action, targetPositionX, targetPositionY):
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

        self.targetPointX = targetPositionX;
        self.targetPointY = targetPositionY;



        # Move
        maxAngularVel = 1.5
        angVel = ((self.actionSize - 1)/2 - action) * maxAngularVel / 2

        # velCmd = Twist()
        # velCmd.linear.x = 0.1
        # velCmd.linear.y = 0.0

        # self.velPub.publish(velCmd)

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
        # if isCrash:
        #     done = True

        distanceToTarget = state[1]

        if distanceToTarget < 0.3:  # Reached to target
            self.isTargetReached = True

        # if isCrash:
        #     reward = -150

        if self.isTargetReached:
            # Reached to target
            rospy.logwarn("Reached to target!")
            reward = 50
            # Calc new target point
            #self.targetPointX, self.targetPointY = self.goalCont.calcTargetPoint()
            #self.targetPointX, self.targetPointY = self.leaderAgent.getPosition();

            self.isTargetReached = False

        else:
            # Neither reached to goal nor crashed calc reward for action
            yawReward = []
            currentDistance = state[1]
            targetAngle = state[0]

            # Calc reward 
            # reference https://emanual.robotis.com/docs/en/platform/turtlebot3/ros2_machine_learning/

            # for i in range(self.actionSize):
            #     angle = -math.pi / 4 + targetAngle + (math.pi / 8 * i) + math.pi / 2
            #     tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            #     yawReward.append(tr)


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

 #           print("reward: ", reward, "   action: ", actionString, "distanceToTarget: ", distanceToTarget)
    




        return np.asarray(state), reward, done

    def reset(self, targetPositionX, targetPositionY):
        '''
        Reset the envrionment
        Reset bot position

        returns state as np.array
targetDistance
        State contains:
        targetAngle, distance
        '''
        self.resetGazebo()

        self.targetPointX = targetPositionX
        self.targetPointY = targetPositionY

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
