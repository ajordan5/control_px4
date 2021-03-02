#!/usr/bin/env python3

import numpy as np
import rospy

from pose2ublox import Pose2Ublox

from geometry_msgs.msg import PoseStamped
from ublox.msg import PosVelEcef
from ublox.msg import RelPos

class Pose2Ublox_Ros():
    def __init__(self):
        self.load_set_parameters()
        self.p2u = Pose2Ublox(self.Ts, self.global_horizontal_accuracy, \
            self.global_vertical_accuracy, self.global_speed_accuracy, \
            self.relative_horizontal_accuracy, self.relative_vertical_accuracy, \
            self.relative_speed_accuracy, self.accHeading, self.noise_on, self.ref_lla, self.sigma_rover_pos, \
            self.sigma_rover_vel, self.sigma_rover_relpos, self.sigma_base_pos, \
            self.sigma_base_vel, self.lpf_on, self.A, self.B)

        # Publishers
        self.rover_virtual_relpos_pub_ = rospy.Publisher('RelPos', RelPos, queue_size=5, latch=True)
        self.rover_virtual_PosVelEcef_pub_ = rospy.Publisher('PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        self.compass_virtual_relPos_pub_ = rospy.Publisher('boat/compass/RelPos', RelPos, queue_size=5, latch=True)
        self.base_virtual_PosVelEcef_pub_ = rospy.Publisher('boat/PosVelEcef', PosVelEcef, queue_size=5, latch=True)
        
        # Timer
        self.ublox_rate_timer_ = rospy.Timer(rospy.Duration(self.Ts), self.ubloxRateCallback)
    
    def ubloxRateCallback(self, event):
        # may need to replace this if the boat is working again.
        #if not self.receivedPose:
        #    return
            
        #publishes all the messages together like ublox would
        #TODO use event to get time
        time_stamp = rospy.Time.now()
        current_time = time_stamp.secs+time_stamp.nsecs*1e-9
        dt = current_time - self.prev_time
        self.prev_time = current_time

        #update messages
        self.p2u.update_rover_virtual_PosVelEcef(dt)
        self.p2u.update_rover_virtual_relPos()
        self.p2u.update_compass_virtual_relPos()
        self.p2u.update_base_virtual_PosVelEcef(dt)

        #publish messages
        self.publish_rover_virtual_PosVelEcef(time_stamp)
        self.publish_rover_virtual_relPos()
        self.publish_base_virtual_PosVelEcef(time_stamp)
        self.publish_compass_virtual_relPos()

    def publish_rover_virtual_PosVelEcef(self, time_stamp):

        self.rover_PosVelEcef.header.stamp = time_stamp
        self.rover_PosVelEcef.fix = 3
        self.rover_PosVelEcef.lla = self.p2u.rover_virtual_lla         
        self.rover_PosVelEcef.position = self.p2u.rover_virtual_pos_ecef
        self.rover_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
        self.rover_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
        self.rover_PosVelEcef.velocity = self.p2u.rover_virtual_vel_ecef
        self.rover_PosVelEcef.speed_accuracy = self.global_speed_accuracy

        self.rover_virtual_PosVelEcef_pub_.publish(self.rover_PosVelEcef)


    def publish_rover_virtual_relPos(self):

        self.rover_relPos.header.stamp = rospy.Time.now()
        self.rover_relPos.relPosNED[0] = self.p2u.rover_virtual_relpos[0]
        self.rover_relPos.relPosNED[1] = self.p2u.rover_virtual_relpos[1]
        self.rover_relPos.relPosNED[2] = self.p2u.rover_virtual_relpos[2]

        self.rover_virtual_relpos_pub_.publish(self.rover_relPos)

    def publish_base_virtual_PosVelEcef(self, time_stamp):

        self.base_PosVelEcef.header.stamp = time_stamp
        self.base_PosVelEcef.fix = 3
        self.base_PosVelEcef.lla = self.p2u.base_virtual_lla  #base lla is not currently being used            
        self.base_PosVelEcef.position = self.p2u.base_virtual_pos_ecef
        self.base_PosVelEcef.horizontal_accuracy = self.global_horizontal_accuracy
        self.base_PosVelEcef.vertical_accuracy = self.global_vertical_accuracy
        self.base_PosVelEcef.velocity = self.p2u.base_virtual_vel_ecef
        self.base_PosVelEcef.speed_accuracy = self.global_speed_accuracy

        self.base_virtual_PosVelEcef_pub_.publish(self.base_PosVelEcef)

    
    def publish_compass_virtual_relPos(self):

        self.compass_relPos.relPosHeading = self.p2u.compass_heading
        self.compass_relPos.accHeading = self.accHeading
        self.compass_virtual_relPos_pub_.publish(self.compass_relPos)

    
    def load_set_parameters(self):
        self.receivedPose = False

        ublox_frequency = rospy.get_param('~ublox_frequency', 5.0)
        self.Ts = 1.0/ublox_frequency
        self.global_horizontal_accuracy = rospy.get_param('~global_horizontal_accuracy', 0.4)
        self.global_vertical_accuracy = rospy.get_param('~global_vertical_accuracy', 0.6)
        self.global_speed_accuracy = rospy.get_param('~global_speed_accuracy', 0.4)
        self.relative_horizontal_accuracy = rospy.get_param('~relative_horizontal_accuracy', 0.02)
        self.relative_vertical_accuracy = rospy.get_param('~relative_vertical_accuracy', 0.06)
        self.relative_speed_accuracy = rospy.get_param('~relative_speed_accuracy', 0.02)
        self.accHeading = rospy.get_param('~heading_accuracy', 0.01) #The noise is currently not applied to the heading.
        self.noise_on = rospy.get_param('~noise_on', True)
        ref_lla = rospy.get_param('~ref_lla', [40.267320, -111.635629, 1387.0])
        self.ref_lla = np.array(ref_lla)
        self.sigma_rover_pos = rospy.get_param('~sigma_rover_pos', 5.0) 
        self.sigma_rover_vel = rospy.get_param('~sigma_rover_vel', 5.0)
        self.sigma_rover_relpos = rospy.get_param('~sigma_rover_relpos', 0.0)
        self.sigma_base_pos = rospy.get_param('~sigma_base_pos', 0.0)
        self.sigma_base_vel = rospy.get_param('~sigma_base_vel', 0.0)
        self.lpf_on = rospy.get_param('~lpf_on', False)
        self.A = rospy.get_param('~A', 6378137.0)
        self.B = rospy.get_param('~B', 6356752.314245)

        #message types
        self.rover_PosVelEcef = PosVelEcef()
        self.rover_relPos = RelPos()
        self.compass_relPos = RelPos()
        self.base_PosVelEcef = PosVelEcef()

        #used for updating dt
        self.prev_time = 0.0
