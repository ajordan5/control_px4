#!/usr/bin/env python3
import numpy as np
import rospy
from pose2ublox_ros import Pose2Ublox_Ros
from geometry_msgs.msg import PoseStamped

class Mocap2Ublox(Pose2Ublox_Ros):
    def __init__(self):
        super().__init__()
        # Subscribers
        self.rover_mocap_ned_sub_ = rospy.Subscriber('rover_pose', PoseStamped, self.roverNedCallback, queue_size=5)
        self.base_mocap_ned_sub_ = rospy.Subscriber('base_mocap', PoseStamped, self.baseNedCallback, queue_size=5)

    def roverNedCallback(self, msg):
        
        self.m2u.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])


    def baseNedCallback(self, msg):

        self.m2u.base_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])

        self.m2u.compass_quat = np.array([msg.pose.orientation.w,
                                        msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z])
        
if __name__ == '__main__':
    rospy.init_node('m2u', anonymous=True)
    try:
        m2u = Mocap2Ublox()
    except:
        rospy.ROSInterruptException
    pass
