#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ublox.msg import PosVelEcef

from frames import *

class Frames:
    def __init__(self):
        self.frameError = Float32()
        self.VGPsN = [0.0,0.0,0.0]
        self.VRoverF = [0.0,0.0,0.0]

        self.frame_error_pub_ = rospy.Publisher('/frame_error', Float32, queue_size=5, latch=True)
        self.posVelEcef_sub_ = rospy.Subscriber('/rover/PosVelEcef', PosVelEcef, self.posVelEcefCallback, queue_size=5)
        self.px4_estimate_sub_ = rospy.Subscriber('/estimate', Odometry, self.px4EstimateCallback, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def posVelEcefCallback(self,msg):
        self.VGpsN = msg.velocity
        self.frameError.data = calculate_frame_error(self.VGpsN,self.VRoverF)
        self.frame_error_pub_.publish(self.frameError)

    def px4EstimateCallback(self,msg):
        VRoverFBody = [msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.VRoverF = rotate_to_inertial_frame(VRoverFBody,quat)


if __name__ == '__main__':
    rospy.init_node('frames', anonymous=True)
    try:
        frames = Frames()
    except:
        rospy.ROSInterruptException
    pass
