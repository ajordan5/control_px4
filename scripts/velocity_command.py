#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import navpy
import rospy

from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from pid_class import PID


class VelCntrl:
    def __init__(self):
        self.hlc = [0.0,0.0,0.0]
        self.prev_time = 0.0 #seconds
        self.lat_ref = 0.0
        self.lon_ref = 0.0
        self.alt_ref = 0.0
        self.ref_set = False
        self.time_set = False
        self.beginLandingroutine = False

        kpN = 1.0
        kiN = 0.1
        kdN = 0.2
        tauN = 0.0
        maxNDot = 8.0
        self.northPid = PID(kpN,kiN,kdN,tauN,maxNDot)
        self.kffN = 1.0+kdN

        kpE = 1.0
        kiE = 0.1
        kdE = 0.2
        tauE = 0.0
        maxEDot = 8.0
        self.eastPid = PID(kpE,kiE,kdE,tauE,maxEDot)
        self.kffE = 1.0+kdE

        kpD = 1.0
        kiD = 0.1
        kdD = 0.2
        tauD = 0.0
        maxDDot = 8.0
        self.downPid = PID(kpD,kiD,kdD,tauD,maxDDot)
        self.kffD = 1.0+kdD

        self.ref_lla = Point()
        self.rover_pos = Point()

        self.rover_pos_pub_ = rospy.Publisher('rover_pos',Point,queue_size=5,latch=True)
        self.ref_lla_pub_ = rospy.Publisher('ref_lla',Point,queue_size=5,latch=True)
        self.hlc_sub_ = rospy.Subscriber('hlc', Point, self.hlcCallback, queue_size=5) 
        self.boat_vel_sub_ = rospy.Subscriber('boat_vel', Point, self.boatVelCallback, queue_size=5)
        self.begin_landing_routine_sub_ = rospy.Subscriber('begin_landing_routine', Bool, self.beginLandingroutineCallback, queue_size=5)

    def hlcCallback(self,msg):
        self.hlc = [msg.x,msg.y,msg.z]
    
    def boatVelCallback(self,msg):
        self.boat_vel = [msg.x,msg.y,msg.z]

    def beginLandingroutineCallback(self,msg):
        self.beginLandingroutine = msg.data

    def publish_ref_lla(self):
        self.ref_lla.x = self.lat_ref
        self.ref_lla.y = self.lon_ref
        self.ref_lla.z = self.alt_ref
        self.ref_lla_pub_.publish(self.ref_lla)

    def publish_rover_pos(self,ned):
        self.rover_pos.x = ned[0]
        self.rover_pos.y = ned[1]
        self.rover_pos.z = ned[2]
        self.rover_pos_pub_.publish(self.rover_pos)

    async def run(self):
        """ Does Offboard control using velocity NED coordinates. """

        self.drone = System()
        await self.drone.connect(system_address="udp://:14540")

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered with UUID: {state.uuid}")
                break

        print("-- Arming")
        await self.drone.action.arm()

        print("-- Setting initial setpoint")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return

        asyncio.create_task(self.run_control_task())
        asyncio.create_task(self.run_time_task())

    async def run_control_task(self):
        async for lla in self.drone.telemetry.position():
            self.time_set = True
            if self.ref_set and self.time_set:
                ned = navpy.lla2ned(lla.latitude_deg,lla.longitude_deg,lla.absolute_altitude_m,self.lat_ref,self.lon_ref,self.alt_ref)
                self.publish_rover_pos(ned)
                cmdVel = self.get_commands(ned)
                if self.beginLandingroutine:
                    cmdVel = self.add_feed_forward(cmdVel)
                #try set_position_velocity.  It will add feed forward for you.
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(cmdVel[0],cmdVel[1],cmdVel[2], 0.0))
            else:
                self.lat_ref = lla.latitude_deg
                self.lon_ref = lla.longitude_deg
                self.alt_ref = lla.absolute_altitude_m
                self.ref_set = True
                self.publish_ref_lla()

    async def run_time_task(self):
        async for odom in self.drone.telemetry.odometry():
            if self.time_set:
                self.time = odom.time_usec/1000000
            else:
                self.prev_time = odom.time_usec/1000000
                self.time_set = True

    def get_commands(self,ned):
        dt = self.time - self.prev_time
        self.northPid.update_control(ned[0],self.hlc[0],dt)
        cmdX = self.northPid.command 
        self.eastPid.update_control(ned[1],self.hlc[1],dt)
        cmdY = self.eastPid.command
        self.downPid.update_control(ned[2],self.hlc[2],dt)
        cmdZ = self.downPid.command
        cmd = [cmdX,cmdY,cmdZ]
        self.prev_time = self.time
        return cmd

    def add_feed_forward(self,cmdVel):
        cmdVel[0] = cmdVel[0] + self.kffN*self.boat_vel[0]
        cmdVel[1] = cmdVel[1] + self.kffE*self.boat_vel[1]
        cmdVel[2] = cmdVel[2] + self.kffD*self.boat_vel[2]
        return cmdVel

if __name__ == "__main__":
    rospy.init_node('vel_cmd', anonymous=True)
    try:
        vel_cntrl = VelCntrl()
        asyncio.ensure_future(vel_cntrl.run())
        loop = asyncio.get_event_loop()
        loop.run_forever()
    except:
        rospy.ROSInterruptException
    pass
