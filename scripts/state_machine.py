#!/usr/bin/env python3

import numpy as np
from numpy.core.defchararray import array
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from mavsdk.offboard import VelocityNedYaw
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
        self.hlc = Odometry() #VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
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
        self.position_kp = np.array([.95, .95, 1])
        self.landing_kp = np.array([.95, .95, 1])
        self.hlc_pub_ = rospy.Publisher('hlc',Odometry,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)
        self.odom_sub_ = rospy.Subscriber('rover_odom',Odometry,self.odomCallback, queue_size=5)
        self.base_odom_sub_ = rospy.Subscriber('base_odom',Odometry,self.baseOdomCallback, queue_size=5)

        # Subscribe to base pose from Gazebo to check waypoint
        self.base_sim_ned_sub_ = rospy.Subscriber('/base_pose', PoseStamped, self.baseNedCallback, queue_size=5)
        self.base_true = np.array([0, 0, 0])

        while not rospy.is_shutdown():
            rospy.spin()

    def baseNedCallback(self, msg):
        self.base_true = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

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
            commands = self.takeoff()
        elif self.missionState == 2:
            commands = self.descend()
        elif self.missionState == 3:
            commands = self.landtest()
        else:
            commands = self.takeoff()
        self.publish_hlc(commands)
    
    def takeoff(self):
        # Test method for takeoff to 2m with velocity control
        hgt = self.boatNed[2] + self.rendevousHeight
        error = np.zeros(3)
        error[2] = hgt - np.array(self.odom)[2] 
        velocityCommand = self.position_kp * error
        if np.linalg.norm(error) < self.rendevousThreshold:
            print("descend mode")
            self.missionState = 2
        #print(velocityCommand)
        return velocityCommand
    
    def descend(self):
        # Test method for landing from 2m with velocity control
        error = np.zeros(3)
        hgt = self.boatNed[2] + self.landingHeight
        error[2] = hgt -np.array(self.odom)[2]
        velocityCommand = self.landing_kp * error
        if np.linalg.norm(error) < 0.1 * self.rendevousThreshold:
            print("land mode")
            self.missionState = 3
        return velocityCommand

    def landtest(self):
        # Test method for landing from 2m with velocity control
        error = np.zeros(3)
        error[2] = -np.array(self.odom)[2]
        velocityCommand = self.landing_kp * error
        if np.linalg.norm(error) < 0.1 * self.rendevousThreshold:
            print("landed")
            velocityCommand = np.zeros(3)
        #velocityCommand[2] = 2.0
        return velocityCommand         

    def publish_hlc(self,commands):
       
        # Publish the base velocity for a feedforward term in the control
        self.hlc.twist.twist.linear.x = commands[0]
        self.hlc.twist.twist.linear.y = commands[1]
        self.hlc.twist.twist.linear.z = commands[2]
        self.hlc.header.stamp = rospy.Time.now()
        self.hlc_pub_.publish(self.hlc)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        
