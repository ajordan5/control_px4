#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
from mavsdk.mocap import (AttitudePositionMocap,Quaternion,PositionBody,Covariance)
import navpy
import rospy
import numpy as np

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool

class CntrlPx4:
    def __init__(self):
        self.velCmd = [0.0,0.0,0.0]
        self.prevVelCmd = [0.0,0.0,0.0]
        self.prevPoseTime = 0.0
        self.estimate = Point()

        self.estimate_pub_ = rospy.Publisher('estimate',Point,queue_size=5,latch=True)
        self.vel_cmd_sub_ = rospy.Subscriber('velCmd', Point, self.velCmdCallback, queue_size=5)
        self.positiion_measurement_sub_ = rospy.Subscriber('position_measurement', PoseWithCovarianceStamped, self.positionMeasurementCallback, queue_size=5)
    
    def velCmdCallback(self,msg):
        self.velCmd = [msg.x,msg.y,msg.z]
        # self.update_control()

    def positionMeasurementCallback(self,msg):
        time = np.array(msg.header.stamp.secs) + np.array(msg.header.stamp.nsecs*1E-9) #check that this is what is needed
        q = Quaternion(1.0,0.0,0.0,0.0) #GPS has not orientation information.  Reflect an infinite covariance for orientation
        positionBody = PositionBody(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
        #Todo: need to get the actual covariance matrix
        covarianceMatrix = float('nan')
        poseCovariance = Covariance(covarianceMatrix)
        self.pose = AttitudePositionMocap(time,q,positionBody,poseCovariance)
        # await self.drone.offboard.set_attitude_position_mocap(self.pose)

    def publish_estimate(self,position_body):
        self.estimate.x = position_body.x_m
        self.estimate.y = position_body.y_m
        self.estimate.z = position_body.z_m
        self.estimate_pub_.publish(self.estimate)

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

        print("-- Go up 2 m/s")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(4)

        asyncio.create_task(self.offboard_position_measurement_callback())
        # asyncio.create_task(self.offboard_velocity_command_callback())
        async for odom in self.drone.telemetry.odometry():
            self.publish_estimate(odom.position_body)

    async def offboard_position_measurement_callback(self):
        while(1):
            if self.pose.time_usec != self.prevPoseTime:
                print('self.pose = ', self.pose)
                await self.drone.offboard.set_attitude_position_mocap(self.pose)
                self.prevPoseTime = self.pose.time_usec

    async def offboard_velocity_command_callback(self):
        while(1):
            if self.velCmd != self.prevVelCmd:
                await self.drone.offboard.set_velocity_ned(VelocityNedYaw(self.velCmd[0],self.velCmd[1],self.velCmd[2],0.0))
                self.prevVelCmd = self.velCmd

    # async def update_position(self):
    #     await self.drone.offboard.set_attitude_position_mocap(self.pose)


    #     asyncio.create_task(self.run_time_task())

    # async def update_control(self):
    #     await self.drone.offboard.set_velocity_ned(VelocityNedYaw(self.cmdVel[0],self.cmdVel[1],self.cmdVel[2], 0.0))

    # async def run_time_task(self):
    #     await get estimate

if __name__ == "__main__":
    rospy.init_node('control_px4', anonymous=True)
    try:
        cntrl_px4 = CntrlPx4()
        asyncio.ensure_future(cntrl_px4.run())
        loop = asyncio.get_event_loop()
        loop.run_forever()
    except:
        rospy.ROSInterruptException
    pass
