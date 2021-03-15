#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ublox.msg import RelPos
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R


class StateMachine:
    def __init__(self):
        self.missionState = 0 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land
        self.waypoints = rospy.get_param('~waypoints', [[0.0,0.0,-2.0]])
        self.hlc = self.waypoints
        self.antennaOffset = rospy.get_param('~antennaOffset', [0.0,0.0,0.0])

        self.missionThreshold = rospy.get_param('~missionThreshold', 0.3)
        self.rendevousThreshold = rospy.get_param('~rendevousThreshold', 0.3)
        self.rendevousHeight = rospy.get_param('~rendevousHeight', -2.0)
        self.landingThreshold = rospy.get_param('~landingThreshold', 0.1)
        self.landingHeight = rospy.get_param('~landingHeight', -0.15)
        self.autoLand = rospy.get_param('~autoLand', False)
        self.cyclicalPath = rospy.get_param('~cyclicalPath', False)

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]
        self.rover2BaseRelPos = [0.0,0.0,0.0]
        self.RBase = R.from_rotvec(np.pi/180.0*np.array([0.0,0.0,0.0])) 
        self.currentWaypointIndex = 0

        self.hlcMsg = PoseStamped()
        self.beginLandingRoutineMsg = Bool()
        self.hlc_pub_ = rospy.Publisher('hlc',PoseStamped,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)
        self.rover2BaseRelPos_sub_ = rospy.Subscriber('rover2BaseRelPos', Point, self.rover2BaseRelPosCallback, queue_size=5)
        self.odom_sub_ = rospy.Subscriber('odom',Odometry,self.odomCallback, queue_size=5)
        self.base_heading_sub = rospy.Subscriber('base_heading',Vector3,self.baseHeadingCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def rover2BaseRelPosCallback(self,msg):
        self.rover2BaseRelPos = [msg.x,msg.y,msg.z]

    def odomCallback(self,msg):
        self.odom = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        self.update_hlc()

    def baseHeadingCallback(self,msg):
        self.RBase = R.from_rotvec(np.array([0.0,0.0,msg.z])) #could add other orientations if needed.

    def update_hlc(self):
        if self.missionState == 1:
            self.rendevous()
        elif self.missionState == 2:
            self.descend()
        elif self.missionState == 3:
            self.land()
        else:
            self.rendevous()
            #self.fly_mission()

    def fly_mission(self):
        currentWaypoint = self.waypoints[self.currentWaypointIndex]
        self.hlc = currentWaypoint
        self.publish_hlc()
        error = np.linalg.norm(np.array(self.hlc)-np.array(self.odom))
        if error < self.missionThreshold:
            print('reached waypoint ', self.currentWaypointIndex + 1)
            self.currentWaypointIndex += 1
            if self.currentWaypointIndex == len(self.waypoints) and self.autoLand == True:
                self.missionState = 1
                print('rendevous state')
                self.beginLandingRoutineMsg.data = True
                self.begin_landing_routine_pub_.publish(self.beginLandingRoutineMsg)
                return
            if self.cyclicalPath:
                self.currentWaypointIndex %= len(self.waypoints)            
            elif self.currentWaypointIndex == len(self.waypoints):
                self.currentWaypointIndex -=1
                
    def rendevous(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.rendevousHeight]) + self.RBase.apply(np.array(self.antennaOffset))
        self.hlc = error + np.array(self.odom)
        self.publish_hlc()
        if np.linalg.norm(error) < self.rendevousThreshold:
            self.missionState = 2
            print('descend state')

    def descend(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.landingHeight]) + self.RBase.apply(np.array(self.antennaOffset))
        self.hlc = error + np.array(self.odom)
        self.publish_hlc()
        if np.linalg.norm(error) < self.landingThreshold:
            self.missionState = 3
            print('land state')

    def land(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,5.0]) + self.RBase.apply(np.array(self.antennaOffset))
        self.hlc = error + np.array(self.odom) #multirotor attemptes to drive itself into the platform 5 meters deep.
        self.publish_hlc()

    def publish_hlc(self):
        self.hlcMsg.pose.position.x = self.hlc[0]
        self.hlcMsg.pose.position.y = self.hlc[1]
        self.hlcMsg.pose.position.z = self.hlc[2]
        self.hlcMsg.header.stamp = rospy.Time.now()
        self.hlc_pub_.publish(self.hlcMsg)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        
