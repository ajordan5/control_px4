#!/usr/bin/env python3

import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

class Pose2Is:
    def __init__(self):
        self.imu = Imu()

        self.prevPosition = np.zeros(3)
        self.prevVelocity = np.zeros(3)
        self.prevEuler = np.zeros(3)
        self.prevAcceleration = np.zeros(3)
        self.prevAngularRates = np.zeros(3)
        self.prevTime = 0.0

        self.alpha = 0.1

        self.firstTime = False

        self.inertial_sense_imu_pub_ = rospy.Publisher('/boat/imu', Imu, queue_size=5, latch=True)
        self.base_sim_ned_sub_ = rospy.Subscriber('/base_pose', PoseStamped, self.baseNedCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def baseNedCallback(self,msg):
        time = msg.header.stamp.secs + msg.header.stamp.nsecs*1E-9
        if self.firstTime:
            self.prevTime = time
            self.firstTime = False
            return
        dt = time - self.prevTime
        self.prevTime = time

        position = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
        quat = np.array([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
        acceleration, angularRates = self.get_imu_data(dt,position,quat)

        self.publish_imu(msg.header.stamp, acceleration, angularRates)

    def get_imu_data(self,dt,position,quat):
        velocity = (position - self.prevPosition)/dt
        accelerationRaw = (velocity - self.prevVelocity)/dt
        accelerationLpf = self.low_pass_filter(accelerationRaw,self.prevAcceleration)

        euler = R.from_quat(quat).as_euler('xyz')
        eulerDot = (euler - self.prevEuler)/dt
        sphi = np.sin(euler.item(0))
        cphi = np.cos(euler.item(0))
        cth = np.cos(euler.item(1))
        sth = np.sin(euler.item(1))
        derivatives2Rates = np.array([[1.0, 0.0, -sth],
                                      [0.0, cphi, sphi*cth],
                                      [0.0, -sphi, cphi*cth]])
        angularRatesRaw = derivatives2Rates@eulerDot
        angularRatesLpf = self.low_pass_filter(angularRatesRaw,self.prevAngularRates)

        self.prevPosition = position
        self.prevVelocity = velocity
        self.prevEuler = euler
        self.prevAcceleration = accelerationLpf
        self.prevAngularRates = angularRatesLpf

        return accelerationLpf, angularRatesLpf

    def low_pass_filter(self,y,u):
        yNew = self.alpha*y+(1.0-self.alpha)*u
        return yNew

    def publish_imu(self, stamp, acceleration, angularRates):
        self.imu.header.stamp = stamp

        self.imu.angular_velocity.x = angularRates[0]
        self.imu.angular_velocity.y = angularRates[1]
        self.imu.angular_velocity.z = angularRates[2]

        self.imu.linear_acceleration.x = acceleration[0]
        self.imu.linear_acceleration.y = acceleration[1]
        self.imu.linear_acceleration.z = acceleration[2]

        self.inertial_sense_imu_pub_.publish(self.imu)

if __name__ == '__main__':
    rospy.init_node('p2i', anonymous=True)
    try:
        p2i = Pose2Is()
    except:
        rospy.ROSInterruptException
    pass

        
