#!/usr/bin/env python3

from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
import rospy

class Quat2Euler:
    def __init__(self):
        self.euler = Vector3Stamped()
        self.euler_pub_ = rospy.Publisher('euler',Vector3Stamped,queue_size=5,latch=True)
        self.odom_sub_ = rospy.Subscriber('/rover_odom', Odometry, self.odomCallback, queue_size=5)
        while not rospy.is_shutdown():
            rospy.spin()

    def odomCallback(self,msg):
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        self.euler.header = msg.header
        self.quat2euler(quat)

    def quat2euler(self,quat):
        r = R.from_quat(quat)
        euler = r.as_euler('xyz', degrees=True)
        self.euler.vector.x = euler[0]
        self.euler.vector.y = euler[1]
        self.euler.vector.z = euler[2]
        self.euler_pub_.publish(self.euler)

if __name__ == "__main__":
    rospy.init_node('quat2euler', anonymous=True)
    try:
        q2e = Quat2Euler()
    except:
        rospy.ROSInterruptException
    pass
