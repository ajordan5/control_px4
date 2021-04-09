#!/usr/bin/env python3
import numpy as np
import rospy
from pose2ublox_ros import Pose2Ublox_Ros
from geometry_msgs.msg import PoseStamped

class Mocap2Ublox(Pose2Ublox_Ros):
    def __init__(self):
        super().__init__()
        self.receivedRoverPose = False
        self.receivedBasePose = False
        # Subscribers
        self.rover_mocap_ned_sub_ = rospy.Subscriber('rover_mocap', PoseStamped, self.roverNedCallback, queue_size=5)
        self.base_mocap_ned_sub_ = rospy.Subscriber('base_mocap', PoseStamped, self.baseNedCallback, queue_size=5)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def roverNedCallback(self, msg):
        self.p2u.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])
        if not self.receivedPose:
            self.receivedRoverPose = True
            if self.receivedBasePose:
                self.receivedPose = True

    def baseNedCallback(self, msg):

        self.p2u.base_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])

        self.p2u.compass_quat = np.array([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        
        if not self.receivedPose:
            self.receivedBasePose = True
            if self.receivedRoverPose:
                self.receivedPose = True
if __name__ == '__main__':
    rospy.init_node('p2u', anonymous=True)
    try:
        p2u = Mocap2Ublox()
    except:
        rospy.ROSInterruptException
    pass
