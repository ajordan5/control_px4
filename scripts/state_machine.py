#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from ublox.msg import RelPos
from ublox.msg import PosVelEcef
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R


class StateMachine:
    def __init__(self):
        self.missionState = 0 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land
        self.waypoints = rospy.get_param('~waypoints', [[0.0,0.0,-2.0]])
        self.hlc = Odometry()
        self.antennaOffset = rospy.get_param('~antennaOffset', [0.0,0.0,0.0])

        self.missionThreshold = rospy.get_param('~missionThreshold', 0.3)
        self.rendevousThreshold = rospy.get_param('~rendevousThreshold', 0.3)
        self.rendevousHeight = rospy.get_param('~rendevousHeight', -2.0)
        self.landingThreshold = rospy.get_param('~landingThreshold', 0.1)
        self.baseXYAttitudeThreshold = rospy.get_param('~baseXYAttitudeThreshold',8.0)
        self.landingHeight = rospy.get_param('~landingHeight', -0.15)
        self.autoLand = rospy.get_param('~autoLand', False)
        self.cyclicalPath = rospy.get_param('~cyclicalPath', False)

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]
        self.rover2BaseRelPos = [0.0,0.0,0.0]
        self.RBase = R.from_rotvec(np.pi/180.0*np.array([0.0,0.0,0.0])) 
        self.currentWaypointIndex = 0
        self.feedForwardVelocity = [0.0,0.0,0.0]
        self.baseXYAttitude = [0.0,0.0]
        self.hlcMsg = PoseStamped()
        self.beginLandingRoutineMsg = Bool()
        self.hlc_pub_ = rospy.Publisher('hlc',Odometry,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)
        self.odom_sub_ = rospy.Subscriber('rover_odom',Odometry,self.odomCallback, queue_size=5)
        self.base_odom_sub_ = rospy.Subscriber('base_odom',Odometry,self.baseOdomCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def odomCallback(self,msg):
        self.odom = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        self.update_hlc()

    def baseOdomCallback(self,msg):
        self.rover2BaseRelPos[0] = msg.pose.pose.position.x
        self.rover2BaseRelPos[1] = msg.pose.pose.position.y
        self.rover2BaseRelPos[2] = msg.pose.pose.position.z
        self.feedForwardVelocity[0] = msg.twist.twist.linear.x
        self.feedForwardVelocity[1] = msg.twist.twist.linear.y
        self.feedForwardVelocity[2] = msg.twist.twist.linear.z

        orientBaseR = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        orientBaseEuler = orientBaseR.as_euler('xyz',degrees=True)
        self.baseXYAttitude = [orientBaseEuler[0],orientBaseEuler[1]]
        
        self.RBase = R.from_rotvec(np.array([0.0,0.0,orientBaseEuler[2]]))
    
    def update_hlc(self):
        if self.missionState == 1:
            commands = self.rendevous()
        elif self.missionState == 2:
            commands = self.descend()
        elif self.missionState == 3:
            commands = self.land()
        else:
            commands = self.rendevous()
            #commands = self.fly_mission()

        self.publish_hlc(commands)

    def fly_mission(self):
        currentWaypoint = self.waypoints[self.currentWaypointIndex]
        error = np.linalg.norm(np.array(currentWaypoint)-np.array(self.odom))
        if error < self.missionThreshold:
            print('reached waypoint ', self.currentWaypointIndex + 1)
            self.currentWaypointIndex += 1
            if self.currentWaypointIndex == len(self.waypoints) and self.autoLand == True:
                self.missionState = 1
                print('rendevous state')
                self.beginLandingRoutineMsg.data = True
                self.begin_landing_routine_pub_.publish(self.beginLandingRoutineMsg)
            if self.cyclicalPath:
                self.currentWaypointIndex %= len(self.waypoints)            
            elif self.currentWaypointIndex == len(self.waypoints):
                self.currentWaypointIndex -=1
        return [currentWaypoint,[0.0,0.0,0.0]]

    def rendevous(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.rendevousHeight]) + self.RBase.apply(np.array(self.antennaOffset))
        currentWaypoint = error + np.array(self.odom)
        if np.linalg.norm(error) < self.rendevousThreshold:
            self.missionState = 2
            print('descend state')
        return [currentWaypoint,self.feedForwardVelocity]

    def descend(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.landingHeight]) + self.RBase.apply(np.array(self.antennaOffset))
        currentWaypoint = error + np.array(self.odom)
        if np.linalg.norm(error) < self.landingThreshold and np.linalg.norm(self.baseXYAttitude) < self.baseXYAttitudeThreshold:
            self.missionState = 3
            print('land state')
        return [currentWaypoint,self.feedForwardVelocity]

    def land(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,5.0]) + self.RBase.apply(np.array(self.antennaOffset))
        currentWaypoint = error + np.array(self.odom) #multirotor attemptes to drive itself into the platform 5 meters deep.
        return [currentWaypoint,self.feedForwardVelocity]

    def publish_hlc(self,commands):
        self.hlc.pose.pose.position.x = commands[0][0]
        self.hlc.pose.pose.position.y = commands[0][1]
        self.hlc.pose.pose.position.z = commands[0][2]
        self.hlc.twist.twist.linear.x = commands[1][0]
        self.hlc.twist.twist.linear.y = commands[1][1]
        self.hlc.twist.twist.linear.z = commands[1][2]
        self.hlc.header.stamp = rospy.Time.now()
        self.hlc_pub_.publish(self.hlc)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        
