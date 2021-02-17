#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ublox.msg import RelPos
from std_msgs.msg import Bool


class StateMachine:
    def __init__(self):
        self.missionState = 0 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land
        self.firstWaypoint = rospy.get_param('~firstWaypoint', [0.0,0.0,-2.0])
        self.hlc = self.firstWaypoint
        self.antennaOffset = rospy.get_param('~antennaOffset', [0.0,0.0,-0.5])

        self.rendevousThreshold = rospy.get_param('~rendevousThreshold', 0.3)
        self.descendThreshold = rospy.get_param('~descendThreshold', 0.3)
        self.descendHeight = rospy.get_param('~descendHeight', -2.0)
        self.landingThreshold = rospy.get_param('~landingThreshold', 0.1)
        self.landingHeight = rospy.get_param('~landingHeight', -0.15)

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]
        self.rover2BaseRelPos = [0.0,0.0,0.0]

        self.hlcMsg = PoseStamped()
        self.beginLandingRoutineMsg = Bool()
        self.hlc_pub_ = rospy.Publisher('hlc',PoseStamped,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)
        self.relPos_sub_ = rospy.Subscriber('relPos', Point, self.relPosCallback, queue_size=5)
        self.odom_sub_ = rospy.Subscriber('odom',Odometry,self.odomCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def relPosCallback(self,msg):
        self.rover2BaseRelPos = [-msg.x,-msg.y,-msg.z]

    def odomCallback(self,msg):
        self.odom = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        self.update_hlc()

    def update_hlc(self):
        if self.missionState == 1:
            self.rendevous()
        elif self.missionState == 2:
            self.descend()
        elif self.missionState == 3:
            self.land()
        else:
            self.fly_mission()

    def fly_mission(self):
        self.hlc = self.firstWaypoint
        self.publish_hlc()
        error = np.array(self.hlc)-np.array(self.odom)
        if np.linalg.norm(error) < self.rendevousThreshold:
            self.missionState = 1
            print('rendevous state')
            self.beginLandingRoutineMsg.data = True
            self.begin_landing_routine_pub_.publish(self.beginLandingRoutineMsg)
        
    def rendevous(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.descendHeight]) + np.array(self.antennaOffset)
        self.hlc = error + np.array(self.odom)
        self.publish_hlc()
        if np.linalg.norm(error) < self.descendThreshold:
            self.missionState = 2
            print('descend state')

    def descend(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.landingHeight]) + np.array(self.antennaOffset)
        self.hlc = error + np.array(self.odom)
        self.publish_hlc()
        if np.linalg.norm(error) < self.landingThreshold:
            self.missionState = 3
            print('land state')

    def land(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,5.0]) + np.array(self.antennaOffset)
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
        