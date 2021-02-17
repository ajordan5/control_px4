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

        kpN = 2.5
        kiN = 0.0 #0.05
        kdN = 1.0
        tauN = 0.05
        maxNDot = 5.0
        conditionalIntegratorThreshold = 1000.0 #high conditional integrator effectively keeps the integrator on
        self.northPid = PID(kpN,kiN,kdN,tauN,maxNDot,conditionalIntegratorThreshold)
        self.kffN = 1.0+kdN

        kpE = 2.5
        kiE = 0.0 #0.05
        kdE = 1.0
        tauE = 0.05
        maxEDot = 5.0
        conditionalIntegratorThreshold = 1000.0
        self.eastPid = PID(kpE,kiE,kdE,tauE,maxEDot,conditionalIntegratorThreshold)
        self.kffE = 1.0+kdE

        kpD = 2.5
        kiD = 0.0 #0.05
        kdD = 1.0
        tauD = 0.05
        maxDDot = 5.0
        conditionalIntegratorThreshold = 1000.0
        self.downPid = PID(kpD,kiD,kdD,tauD,maxDDot,conditionalIntegratorThreshold)
        self.kffD = 1.0+kdD

        self.velCmd = Point()

        self.vel_cmd_pub_ = rospy.Publisher('vel_cmd',Point,queue_size=5,latch=True)
        self.start_controller_sub = rospy.Subscriber('start_controller',Bool,self.startControllerCallback,queue_size=5)
        self.odom_sub_ = rospy.Subscriber('odom',Odometry,self.odomCallback,queue_size=5)
        self.hlc_sub_ = rospy.Subscriber('hlc', PoseStamped, self.hlcCallback, queue_size=5) 
        self.boat_vel_sub_ = rospy.Subscriber('base_vel', Point, self.boatVelCallback, queue_size=5)
        self.begin_landing_routine_sub_ = rospy.Subscriber('begin_landing_routine', Bool, self.beginLandingroutineCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def startControllerCallback(self,msg):
        if msg.data == True:
            self.reset_integrators()

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
        self.northPid.update_control(self.odom[0],self.hlc[0],dt)
        cmdX = self.northPid.command 
        self.eastPid.update_control(self.odom[1],self.hlc[1],dt)
        cmdY = self.eastPid.command
        self.downPid.update_control(self.odom[2],self.hlc[2],dt)
        cmdZ = self.downPid.command
        cmd = [cmdX,cmdY,cmdZ]
        self.prev_time = self.time
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
