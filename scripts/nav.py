#!/usr/bin/env python3

import rospy
import navpy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from ublox.msg import RelPos
from ublox.msg import PosVelEcef

class Nav:
    def __init__(self):
        self.baseVelocity = Point()
        self.relPos = Point()
        self.pose_update = PoseWithCovarianceStamped()

        self.refLlaSet = False

        #TODO we may need to speed up command inputs in order to have good performance.
        #this node can be used to extrapolate relpos messages given rover estimate updates.
        #or we could potentially use base estimation for the extrapolation as well.
        # self.rover_extrapolated_relPos_pub_ = rospy.Publisher('extrap_relPos', RelPos, queue_size=5, latch=True)
        self.rover_relPos_stripped_pub_ = rospy.Publisher('relPos_stripped', Point, queue_size=5, latch=True)
        self.base_velocity_pub_ = rospy.Publisher('base_velocity', Point, queue_size=5, latch=True)
        self.pose_update_pub_ = rospy.Publisher('pose_update', PoseWithCovarianceStamped, queue_size=5, latch=True)

        self.rover_relPos_sub_ = rospy.Subscriber('rover_relPos', RelPos, self.roverRelPosCallback, queue_size=5)
        self.posVelEcef_sub_ = rospy.Subscriber('posVelEcef', PosVelEcef, self.posVelEcefCallback, queue_size=5)
        self.base_posVelEcef_sub_ = rospy.Subscriber('base_posVelEcef', PosVelEcef, self.basePosVelEcefCallback, queue_size=5)
        # self.px4_estimate_sub_ = rospy.Subscriber('/px4_estimate', PoseStamped, self.px4EstimateCallback, queue_size=5)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def roverRelPosCallback(self, msg):
        self.relPos.x = np.array(msg.relPosNED[0])+np.array(msg.relPosHPNED[0])
        self.relPos.y = np.array(msg.relPosNED[1])+np.array(msg.relPosHPNED[1])
        self.relPos.z = np.array(msg.relPosNED[2])+np.array(msg.relPosHPNED[2])
        self.rover_relPos_stripped_pub_.publish(self.relPos)
        # self.rover_extrapolated_relPos_pub_.publish(msg)
    
    def basePosVelEcefCallback(self,msg):
        if not self.refLlaSet:
            return
        baseVelocityEcef = [msg.velocity[0],msg.velocity[1],msg.velocity[2]]
        baseVelocityNed = navpy.ecef2ned(baseVelocityEcef,self.lat_ref,self.lon_ref,self.alt_ref)
        self.baseVelocity.x = baseVelocityNed[0]
        self.baseVelocity.y = baseVelocityNed[1]
        self.baseVelocity.z = baseVelocityNed[2]
        self.base_velocity_pub_.publish(self.baseVelocity)

    def posVelEcefCallback(self,msg):
        if not self.refLlaSet:
            self.lat_ref = msg.lla[0]
            self.lon_ref = msg.lla[1]
            self.alt_ref = msg.lla[2]
            self.refLlaSet = True
        ned = navpy.lla2ned(msg.lla[0],msg.lla[1],msg.lla[2],self.lat_ref,self.lon_ref,self.alt_ref)
        covariance = np.zeros(36)
        covariance[0] = msg.horizontal_accuracy
        covariance[7] = msg.horizontal_accuracy
        covariance[14] = msg.vertical_accuracy
        covariance[21] = 1000000 #essentially infinite
        covariance[28] = 1000000
        covariance[35] = 1000000 
        self.pose_update.header = msg.header
        self.pose_update.pose.pose.position.x = ned[0]
        self.pose_update.pose.pose.position.y = ned[1]
        self.pose_update.pose.pose.position.z = ned[2]
        self.pose_update.pose.pose.orientation.x = 0.0
        self.pose_update.pose.pose.orientation.y = 0.0
        self.pose_update.pose.pose.orientation.z = 0.0
        self.pose_update.pose.pose.orientation.w = 1.0
        self.pose_update.pose.covariance = covariance
        self.pose_update_pub_.publish(self.pose_update)

if __name__ == '__main__':
    rospy.init_node('nav', anonymous=True)
    try:
        nav = Nav()
    except:
        rospy.ROSInterruptException
    pass
