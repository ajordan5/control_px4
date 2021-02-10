#!/usr/bin/env python3


import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import navpy
import rospy

from geometry_msgs.msg import Point

from pid_class import PID


class VelCntrl:
    def __init__(self):
        self.ref_pub_ = rospy.Publisher('ref',Point,queue_size=5,latch=True)
        self.boat_sub_ = rospy.Subscriber('boat_pos', Point, self.boatCallback, queue_size=5)
        self.ref_lla = Point()
        self.boat_ned = [0.0,0.0,0.0]
        self.rendevous_height = -2.0

        self.prev_time = 0.0 #seconds

        kpN = 1.0
        kiN = 0.0
        kdN = 0.3
        tauN = 0.0
        maxNDot = 1000
        self.northPid = PID(kpN,kiN,kdN,tauN,maxNDot)

        kpE = 1.0
        kiE = 0.0
        kdE = 0.3
        tauE = 0.0
        maxEDot = 1000
        self.eastPid = PID(kpE,kiE,kdE,tauE,maxEDot)

        kpD = 1.0
        kiD = 0.0
        kdD = 0.3
        tauD = 0.0
        maxDDot = 1000
        self.downPid = PID(kpD,kiD,kdD,tauD,maxDDot)

        self.ref_set = False
        self.time_set = False

    def boatCallback(self,msg):
        self.boat_ned = navpy.lla2ned(msg.x,msg.y,msg.z,self.lat_ref,self.lon_ref,self.alt_ref)

    def publish_ref(self):
        self.ref_lla.x = self.lat_ref
        self.ref_lla.y = self.lon_ref
        self.ref_lla.z = self.alt_ref
        self.ref_pub_.publish(self.ref_lla)

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
                cmdVel = self.get_commands(lla)
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(cmdVel[0],cmdVel[1],cmdVel[2], 0.0))
            else:
                self.lat_ref = lla.latitude_deg
                self.lon_ref = lla.longitude_deg
                self.alt_ref = lla.absolute_altitude_m
                self.publish_ref()
                self.ref_set = True

    async def run_time_task(self):
        async for odom in self.drone.telemetry.odometry():
            if self.time_set:
                self.time = odom.time_usec/1000000
            else:
                self.prev_time = odom.time_usec/1000000
                self.time_set = True

    def get_commands(self,lla):
        ned = navpy.lla2ned(lla.latitude_deg,lla.longitude_deg,lla.absolute_altitude_m,self.lat_ref,self.lon_ref,self.alt_ref)
        dt = self.time - self.prev_time
        self.northPid.update_control(ned[0],self.boat_ned[0],dt)
        cmdX = self.northPid.command 
        self.eastPid.update_control(ned[1],self.boat_ned[1],dt)
        cmdY = self.eastPid.command
        self.downPid.update_control(ned[2],self.boat_ned[2]+self.rendevous_height,dt)
        cmdZ = self.downPid.command

        self.prev_time = self.time

        cmd = [cmdX,cmdY,cmdZ]
        return cmd


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
