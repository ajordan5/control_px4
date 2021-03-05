#!/usr/bin/env python3
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from pid_class import PID


class VelCntrl:
    def __init__(self):
        self.odom = [0.0,0.0,0.0]
        self.hlc = [0.0,0.0,0.0]
        self.baseVel = [0.0,0.0,0.0]
        self.prev_time = 0.0 #seconds
        self.beginLandingroutine = False
        self.integrators_on = False

        kpN = rospy.get_param('~kpN', 2.5)
        kiN = rospy.get_param('~kiN', 0.0)
        kdN = rospy.get_param('~kdN', 1.0)
        tauN = rospy.get_param('~tauN', 0.05)
        maxNDot = rospy.get_param('~maxNDot', 5.0)
        conditionalIntegratorThresholdN = rospy.get_param('~conditionalIntegratorThresholdN', 1000.0)
        self.northPid = PID(kpN,kiN,kdN,tauN,maxNDot,conditionalIntegratorThresholdN)
        self.kffN = 1.0+kdN

        kpE = rospy.get_param('~kpE', 2.5)
        kiE = rospy.get_param('~kiE', 0.0)
        kdE = rospy.get_param('~kdE', 1.0)
        tauE = rospy.get_param('~tauE', 0.05)
        maxEDot = rospy.get_param('~maxEDot', 5.0)
        conditionalIntegratorThresholdE = rospy.get_param('~conditionalIntegratorThresholdE', 1000.0)
        self.eastPid = PID(kpE,kiE,kdE,tauE,maxEDot,conditionalIntegratorThresholdE)
        self.kffE = 1.0+kdE

        kpD = rospy.get_param('~kpD', 2.5)
        kiD = rospy.get_param('~kiD', 0.0)
        kdD = rospy.get_param('~kdD', 1.0)
        tauD = rospy.get_param('~tauD', 0.05)
        maxDDot = rospy.get_param('~maxDDot', 5.0)
        conditionalIntegratorThresholdD = rospy.get_param('~conditionalIntegratorThresholdD', 1000.0)
        self.downPid = PID(kpD,kiD,kdD,tauD,maxDDot,conditionalIntegratorThresholdD)
        self.kffD = 1.0+kdD

        self.velCmd = Point()

        self.vel_cmd_pub_ = rospy.Publisher('vel_cmd',Point,queue_size=5,latch=True)
        print('before subscriber')
        self.switch_integrators_sub_ = rospy.Subscriber('switch_integrators',Bool,self.switchIntegratorsCallback,queue_size=5)
        print('after subscriber')
        self.odom_sub_ = rospy.Subscriber('odom',Odometry,self.odomCallback,queue_size=5)
        self.hlc_sub_ = rospy.Subscriber('hlc', PoseStamped, self.hlcCallback, queue_size=5) 
        self.boat_vel_sub_ = rospy.Subscriber('base_vel', Point, self.boatVelCallback, queue_size=5)
        self.begin_landing_routine_sub_ = rospy.Subscriber('begin_landing_routine', Bool, self.beginLandingroutineCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def switchIntegratorsCallback(self,msg):
        self.reset_integrators()
        self.integrators_on = msg.data

    def odomCallback(self,msg):
        self.odom = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        self.time = np.array(msg.header.stamp.secs) + np.array(msg.header.stamp.nsecs*1E-9)
        self.update_control()

    def hlcCallback(self,msg):
        self.hlc = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
    
    def boatVelCallback(self,msg):
        self.baseVel = [msg.x,msg.y,msg.z]

    def beginLandingroutineCallback(self,msg):
        self.beginLandingroutine = msg.data

    def publish_vel_cmd(self,velCmd):
        self.velCmd.x = velCmd[0]
        self.velCmd.y = velCmd[1]
        self.velCmd.z = velCmd[2]
        self.vel_cmd_pub_.publish(self.velCmd)

    def update_control(self):
        cmdVel = self.get_commands()
        if self.beginLandingroutine:
            cmdVel = self.add_feed_forward(cmdVel)
        self.publish_vel_cmd(cmdVel)

    def get_commands(self):
        dt = self.time - self.prev_time
        self.northPid.update_control(self.odom[0],self.hlc[0],dt,self.integrators_on)
        cmdX = self.northPid.command 
        self.eastPid.update_control(self.odom[1],self.hlc[1],dt,self.integrators_on)
        cmdY = self.eastPid.command
        self.downPid.update_control(self.odom[2],self.hlc[2],dt,self.integrators_on)
        cmdZ = self.downPid.command
        cmd = [cmdX,cmdY,cmdZ]
        self.prev_time = self.time

        # print('down integrator = ', self.downPid.integrator) #Use to make sure there is no integrator wind up.
        return cmd

    def add_feed_forward(self,cmdVel):
        cmdVel[0] = cmdVel[0] + self.kffN*self.baseVel[0]
        cmdVel[1] = cmdVel[1] + self.kffE*self.baseVel[1]
        cmdVel[2] = cmdVel[2] + self.kffD*self.baseVel[2]
        return cmdVel

    def reset_integrators(self):
        self.northPid.integrator = 0.0
        self.eastPid.integrator = 0.0
        self.downPid.integrator = 0.0

if __name__ == "__main__":
    rospy.init_node('vel_controller', anonymous=True)
    try:
        vel_cntrl = VelCntrl()
    except:
        rospy.ROSInterruptException
    pass
