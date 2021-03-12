#!/usr/bin/env python3

import rospy
import navpy
import numpy as np
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from ublox.msg import RelPos
from ublox.msg import PosVelEcef
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion


class Nav:
    def __init__(self):
        self.baseVelocity = Point()
        self.rover2Base_relPos = Point()
        self.pose_update = PoseWithCovarianceStamped()

        self.roverQuatNED = Quaternion()

        self.roll_covariance = 1000000 #essentially infinite
        self.pitch_covariance = 1000000
        self.yaw_covariance = 1000000 
        self.mocap_covariance = 0.6 #high to simulate outdoors.  Not sure what this value should really be though.
        self.orientation = [0.0,0.0,0.0,1.0]
        self.base_orientation = Vector3()
        self.refLlaSet = False
        #TODO we may need to speed up command inputs in order to have good performance.
        #this node can be used to extrapolate relpos messages given rover estimate updates.
        #or we could potentially use base estimation for the extrapolation as well.
        # self.rover_extrapolated_relPos_pub_ = rospy.Publisher('extrap_relPos', RelPos, queue_size=5, latch=True)
        self.rover2Base_relPos_stripped_pub_ = rospy.Publisher('rover2Base_relPos_stripped', Point, queue_size=5, latch=True)
        self.base_velocity_pub_ = rospy.Publisher('base_velocity', Point, queue_size=5, latch=True)
        self.pose_update_pub_ = rospy.Publisher('pose_update', PoseWithCovarianceStamped, queue_size=5, latch=True)
        self.base_heading_pub_ = rospy.Publisher('base_heading', Vector3, queue_size=5, latch=True)
        self.base2rover_relPos_sub_ = rospy.Subscriber('base2Rover_relPos', RelPos, self.base2RoverRelPosCallback, queue_size=5)
        self.posVelEcef_sub_ = rospy.Subscriber('posVelEcef', PosVelEcef, self.posVelEcefCallback, queue_size=5)
        self.base_posVelEcef_sub_ = rospy.Subscriber('base_posVelEcef', PosVelEcef, self.basePosVelEcefCallback, queue_size=5)
        self.compass_relPos_sub_ = rospy.Subscriber('compass_relPos', RelPos, self.compassRelPosCallback, queue_size=5)
        self.rover_pose_4_heading_sub_ = rospy.Subscriber('rover_pose_4_heading', PoseStamped, self.roverPose4HeadingCallback, queue_size=5)
        # self.px4_estimate_sub_ = rospy.Subscriber('/px4_estimate', PoseStamped, self.px4EstimateCallback, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def base2RoverRelPosCallback(self, msg):
        self.rover2Base_relPos.x = -np.array(msg.relPosNED[0])-np.array(msg.relPosHPNED[0])
        self.rover2Base_relPos.y = -np.array(msg.relPosNED[1])-np.array(msg.relPosHPNED[1])
        self.rover2Base_relPos.z = -np.array(msg.relPosNED[2])-np.array(msg.relPosHPNED[2])
        self.rover2Base_relPos_stripped_pub_.publish(self.rover2Base_relPos)
        # self.rover_extrapolated_relPos_pub_.publish(msg)

    def compassRelPosCallback(self, msg):
        self.base_orientation.z = msg.relPosHeading
        self.base_heading_pub_.publish(self.base_orientation)
    
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
        covariance[21] = self.roll_covariance
        covariance[28] = self.pitch_covariance
        covariance[35] = self.yaw_covariance
        self.pose_update.header = msg.header
        self.pose_update.pose.pose.position.x = ned[0]
        self.pose_update.pose.pose.position.y = ned[1]
        self.pose_update.pose.pose.position.z = ned[2]
        self.pose_update.pose.pose.orientation.x = self.orientation[0]
        self.pose_update.pose.pose.orientation.y = self.orientation[1]
        self.pose_update.pose.pose.orientation.z = self.orientation[2]
        self.pose_update.pose.pose.orientation.w = self.orientation[3]
        self.pose_update.pose.covariance = covariance
        self.pose_update_pub_.publish(self.pose_update)

    def roverPose4HeadingCallback(self,msg):
        roverQuatENU = [msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w]

        roverRENU = R.from_quat(roverQuatENU)
        roverEulerENU = roverRENU.as_euler('xyz', degrees=True)
        roverHeadingENU = roverEulerENU[2]
        roverHeadingNED = self.wrap((roverHeadingENU-90.0),-180.0,180.0)*-1.0
        roverHeadingNEDNoise = roverHeadingNED + 30.0
        print('heading = ', roverHeadingNED)
        print('noisy heading = ', roverHeadingNEDNoise)
        roverRNED = R.from_euler('z', roverHeadingNEDNoise, degrees=True)
        roverQuatNED = roverRNED.as_quat()

        self.orientation[0] = roverQuatNED[0]
        self.orientation[1] = roverQuatNED[1]
        self.orientation[2] = roverQuatNED[2]
        self.orientation[3] = roverQuatNED[3]
        # self.roll_covariance = self.mocap_covariance #May need to uncomment these.  Not sure how the covariance is used.
        # self.pitch_covariance = self.mocap_covariance
        self.yaw_covariance = self.mocap_covariance
    
    def wrap(self,angle,min,max):
        #Not fully functional for non -180 to 180 wrappings
        angle = (angle + max) % (2 * max) - max
        return angle

if __name__ == '__main__':
    rospy.init_node('nav', anonymous=True)
    try:
        nav = Nav()
    except:
        rospy.ROSInterruptException
    pass
