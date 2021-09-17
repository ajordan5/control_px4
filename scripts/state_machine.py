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
    """A class to manage the state of the a multi-rotor as it attempts to land on moving target.

    Attributes:
        missionState (int): integer indicator for vehicle state
        waypoints (list[float]): 3d waypoints for the vehicle to follow enroute to the target, relative to the target (NED)
        hlc (message type: Odometry): published commands for a desired position and feedforward velocity
        antennaOffset (List[float]): position of rtk-gps antenna relative to the target
        missionThreshold (float): maximum norm of the error to a waypoint to consider it reached in meters
        rendevousThreshold (float): maximum norm of the error from vehicle to rendezvous waypoint to transition to descend
        rendevousHeight (float): target height (m) for the rendezvous state
        landingThreshold (float): maximum norm of the error from vehicle to descend waypoint to transition to land
        baseXYAttitudeThreshold (float): allowable tilt in the target base for a landing attempt (deg.)
        autoland (bool): Flag to determine if landing is autonomous
        cyclicalPath (bool): Flag to determine if the vehicle should travel mission waypoints in a continuous cycle
        roverNed (list[float]): rover NED location in the intertial frame 
        boatNed (list[float]): boat NED location in the intertial frame 
        rover2BaseRelPos (list[float]): NED position of the boat relative to the rover
        feedForwardVelocity (list[float]): NED base velocity for a feedforward term in the control
    """
    def __init__(self):
        self.missionState = 0 #0 - mission --> currently not used
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
        self.landingHeight = rospy.get_param('~landingHeight', -0.15)
        self.baseXYAttitudeThreshold = rospy.get_param('~baseXYAttitudeThreshold',8.0)
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
        # hlc: high level command
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
        self.Rb2i = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    
    def update_hlc(self):
        """Update high level command"""
        # Problem w flying mission outdoors. Basing control on body fixed frame for drone
        if self.missionState == 1:
            commands = self.rendevous()
        elif self.missionState == 2:
            commands = self.descend()
        elif self.missionState == 3:
            commands = self.land()
        else:
            #commands = self.rendevous()
            commands = self.fly_mission()

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
        # Rb2i takes the vector from the antenna to the center of the pad and converts body to inertial frame
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.rendevousHeight]) + self.Rb2i.apply(np.array(self.antennaOffset))
        # Waypoint position is the current vehicle position plus the error
        currentWaypoint = error + np.array(self.odom)
        #print('error=', error, np.linalg.norm(error))
        if np.linalg.norm(error) < self.rendevousThreshold:
            self.missionState = 2
            print('descend state')
            print('rover2Base = ', self.rover2BaseRelPos)
            print('odom = ', self.odom)
            print('error=', error, np.linalg.norm(error))
        return [currentWaypoint,self.feedForwardVelocity]

    def descend(self):
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.landingHeight]) + self.Rb2i.apply(np.array(self.antennaOffset))
        currentWaypoint = error + np.array(self.odom)
        euler = self.Rb2i.as_euler('xyz')
        baseXYAttitude = euler[0:1]
        if np.linalg.norm(error) < self.landingThreshold and np.linalg.norm(baseXYAttitude) < self.baseXYAttitudeThreshold:
            self.missionState = 3
            print('land state')
            print('rover2Base = ', self.rover2BaseRelPos)
            print('odom = ', self.odom)
            print('error=', error, np.linalg.norm(error))
        return [currentWaypoint,self.feedForwardVelocity]

    def land(self):
        # Set the waypoint to 5 meters below the landing pad
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,5.0]) + self.Rb2i.apply(np.array(self.antennaOffset))
        currentWaypoint = error + np.array(self.odom) #multirotor attemptes to drive itself into the platform 5 meters deep.
        print('error=', error, np.linalg.norm(error))
        return [currentWaypoint,self.feedForwardVelocity]

    def publish_hlc(self,commands):
        # Publish the current waypoint
        self.hlc.pose.pose.position.x = commands[0][0]
        self.hlc.pose.pose.position.y = commands[0][1]
        self.hlc.pose.pose.position.z = commands[0][2]
        # Publish the base velocity for a feedforward term in the control
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
        
