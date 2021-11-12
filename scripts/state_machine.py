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
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R


class StateMachine:
    def __init__(self):
        self.missionState = 1 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land
        self.waypoints = rospy.get_param('~waypoints', [[0.0,0.0,-2.0]])
        self.hlc = Odometry()
        self.antennaOffset = rospy.get_param('~antennaOffset', [0.0,0.0,0.0])

        self.missionThreshold = rospy.get_param('~missionThreshold', 0.3)
        self.rendevousThreshold = rospy.get_param('~rendevousThreshold', 0.3)
        self.rendevousTime = rospy.get_param('~rendevousTime', 0.1)
        self.rendevousHeight = rospy.get_param('~rendevousHeight', -2.0)
        self.landingThreshold = rospy.get_param('~landingThreshold', 0.1)
        self.landingTime = rospy.get_param('~landingTime', 0.02)
        self.baseXYAttitudeThreshold = rospy.get_param('~baseXYAttitudeThreshold',8.0)
        self.landingHeight = rospy.get_param('~landingHeight', -0.15)
        self.autoLand = rospy.get_param('~autoLand', False)
        self.cyclicalPath = rospy.get_param('~cyclicalPath', False)

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]
        self.rover2BaseRelPos = [0.0,0.0,0.0]
        self.Rb2i = R.from_quat([0.0,0.0,0.0,1.0]) 
        self.currentWaypointIndex = 0
        self.feedForwardVelocity = [0.0,0.0,0.0]
        self.hlcMsg = PoseStamped()
        self.beginLandingRoutineMsg = Bool()
        
        self.hlc_pub_ = rospy.Publisher('hlc',Odometry,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)
        self.mission_state_pub = rospy.Publisher('mission_state',PointStamped,queue_size=5,latch=True)

        # Velocity Control Parameters
        self.position_kp = np.array(rospy.get_param('~positionKp', [0.95, 0.95, 1.0]))
        self.landing_kp = np.array(rospy.get_param('~landingKp', [0.95, 0.95, 1.0]))

        # Mission threshold timing
        self.in_threshold = False
        self.threshold_time = 0.0

        self.publish_mission_state()

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

        self.Rb2i = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    
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
        velocityCommand = self.position_kp * (np.array(currentWaypoint)-np.array(self.odom))
        if error < self.missionThreshold:
            print('reached waypoint ', self.currentWaypointIndex + 1)
            self.currentWaypointIndex += 1
            if self.currentWaypointIndex == len(self.waypoints) and self.autoLand == True:
                self.missionState = 1
                self.publish_mission_state()
                print('rendevous state')
                self.beginLandingRoutineMsg.data = True
                self.begin_landing_routine_pub_.publish(self.beginLandingRoutineMsg)
            if self.cyclicalPath:
                self.currentWaypointIndex %= len(self.waypoints)            
            elif self.currentWaypointIndex == len(self.waypoints):
                self.currentWaypointIndex -=1
        return velocityCommand

    def rendevous(self):
        # Error from desired waypoint
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.rendevousHeight]) + self.Rb2i.apply(np.array(self.antennaOffset))
        velocityCommand = self.position_kp * error + self.feedForwardVelocity
        
        # Entered threshold
        if np.linalg.norm(error) < self.rendevousThreshold and self.in_threshold == False:
            self.start_threshold_timer()
        # Already in threshold
        elif np.linalg.norm(error) < self.rendevousThreshold and self.in_threshold == True:
            # Check how long inside threshold
            self.threshold_timer()
            if self.threshold_time > self.rendevousTime:
                self.missionState = 2
                self.publish_mission_state()
                print('descend state')
                self.in_threshold = False
        # Exited threshold
        elif np.linalg.norm(error) > self.rendevousThreshold and self.in_threshold == True:
            self.in_threshold = False
            print("EXITED THRESHOLD")
        
        return velocityCommand

    def descend(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.landingHeight]) + self.Rb2i.apply(np.array(self.antennaOffset))
        euler = self.Rb2i.as_euler('xyz')
        baseXYAttitude = euler[0:1]
        velocityCommand = self.position_kp * error + self.feedForwardVelocity
        # Entered threshold
        if np.linalg.norm(error) < self.landingThreshold and self.in_threshold == False:
            self.start_threshold_timer()
        # Already in threshold
        elif np.linalg.norm(error) < self.landingThreshold and self.in_threshold == True:
            # Check how long inside threshold
            self.threshold_timer()
            if self.threshold_time > self.landingTime:
                self.missionState = 3
                self.publish_mission_state()
                print('land state')
        # Exited threshold
        elif np.linalg.norm(error) > self.landingThreshold and self.in_threshold == True:
            self.in_threshold = False
            print("EXITED THRESHOLD")
        return velocityCommand

    def land(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,0.0]) + self.Rb2i.apply(np.array(self.antennaOffset))
        velocityCommand = self.landing_kp * error + self.feedForwardVelocity
        return velocityCommand

    def threshold_timer(self):
        """Method to manage the timing of how long the rover has been in the threshold"""
        computer_time = rospy.Time.now()
        time = computer_time.secs + computer_time.nsecs * 1E-9
        self.threshold_time = time - self.threshold_time_start
        print("TIME IN THRESHOLD:" ,self.threshold_time)
    
    def start_threshold_timer(self):
        """Start timing threshold"""
        print("ENTERED THRESHOLD")
        self.in_threshold = True
        computer_time = rospy.Time.now()
        self.threshold_time_start = computer_time.secs + computer_time.nsecs * 1E-9


    def publish_hlc(self,commands):
        self.hlc.twist.twist.linear.x = commands[0]
        self.hlc.twist.twist.linear.y = commands[1]
        self.hlc.twist.twist.linear.z = commands[2]
        self.hlc.header.stamp = rospy.Time.now()
        self.hlc_pub_.publish(self.hlc)

    def publish_mission_state(self):
        missionState = PointStamped()
        missionState.header.stamp = rospy.Time.now()
        missionState.point.x = self.missionState
        self.mission_state_pub.publish(missionState)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        
