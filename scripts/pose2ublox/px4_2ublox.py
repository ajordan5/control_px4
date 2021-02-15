#!/usr/bin/env python3
import numpy as np
import rospy
from pose2ublox_ros import Pose2Ublox_Ros
from geometry_msgs.msg import PoseStamped

class Px4_2Ublox(Pose2Ublox_Ros):
    def __init__(self):
        super().__init__()
        # Subscribers
        self.rover_mocap_ned_sub_ = rospy.Subscriber('/rover_pose', PoseStamped, self.roverNedCallback, queue_size=5)
        self.base_mocap_ned_sub_ = rospy.Subscriber('/base_pose', PoseStamped, self.baseNedCallback, queue_size=5)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def roverNedCallback(self, msg):
        self.p2u.rover_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])

    def baseNedCallback(self, msg):
        self.p2u.base_ned = np.array([msg.pose.position.x,
                                   msg.pose.position.y,
                                   msg.pose.position.z])

        self.p2u.compass_quat = np.array([msg.pose.orientation.w,
                                        msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z])
        
if __name__ == '__main__':
    rospy.init_node('px2u', anonymous=True)
    try:
        px2u = Px4_2Ublox()
    except:
        rospy.ROSInterruptException
    pass
