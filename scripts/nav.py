#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point
from ublox.msg import RelPos
from ublox.msg import PosVelEcef

class Nav:
    def __init__(self):
        self.baseVelocity = Point()
        self.relPos = Point()
        self.lla = Point()

        #TODO we may need to speed up command inputs in order to have good performance.
        #this node can be used to extrapolate relpos messages given rover estimate updates.
        #or we could potentially use base estimation for the extrapolation as well.
        # self.rover_extrapolated_relPos_pub_ = rospy.Publisher('extrap_relPos', RelPos, queue_size=5, latch=True)
        self.rover_relPos_stripped_pub_ = rospy.Publisher('relPos_stripped', Point, queue_size=5, latch=True)
        self.base_velocity_pub_ = rospy.Publisher('base_velocity', Point, queue_size=5, latch=True)
        self.lla_pub_ = rospy.Publisher('ublox_lla', Point, queue_size=5, latch=True)

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
        self.baseVelocity.x = msg.velocity[0]
        self.baseVelocity.y = msg.velocity[1]
        self.baseVelocity.z = msg.velocity[2]
        self.base_velocity_pub_.publish(self.baseVelocity)

    def posVelEcefCallback(self,msg):
        self.lla.x = msg.lla[0]
        self.lla.y = msg.lla[1]
        self.lla.z = msg.lla[2]
        self.lla_pub_.publish(self.lla)

if __name__ == '__main__':
    rospy.init_node('nav', anonymous=True)
    try:
        nav = Nav()
    except:
        rospy.ROSInterruptException
    pass
