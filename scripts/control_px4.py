#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
from mavsdk.mocap import (AttitudePositionMocap,VisionPositionEstimate,Quaternion,PositionBody,AngleBody,Covariance)
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
        self.meas1_received = False

        self.estimate_pub_ = rospy.Publisher('estimate',Point,queue_size=5,latch=True)
        self.vel_cmd_sub_ = rospy.Subscriber('velCmd', Point, self.velCmdCallback, queue_size=5)
        self.positiion_measurement_sub_ = rospy.Subscriber('position_measurement', PoseWithCovarianceStamped, self.positionMeasurementCallback, queue_size=5)
    
    def velCmdCallback(self,msg):
        self.velCmd = [msg.x,msg.y,msg.z]
        # self.update_control()

    def positionMeasurementCallback(self,msg):
        time = np.array(msg.header.stamp.secs) + np.array(msg.header.stamp.nsecs*1E-9) #TODO this probably needs to be adjusted for the px4 time.
        time = int(round(time,6)*1E6)
        # # q = Quaternion(1.0,0.0,0.0,0.0) #GPS has not orientation information.  Reflect an infinite covariance for orientation
        angleBody = AngleBody(0.0,0.0,0.0) #Currently no angle information is given
        positionBody = PositionBody(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
        # #Todo: need to get the actual covariance matrix
        covarianceMatrix = [0.1,0.0,0.0,0.0,0.0,0.0, 0.1,0.0,0.0,0.0,0.0, 0.1,0.0,0.0,0.0, 0.1,0.0,0.0, 0.1,0.0, 0.1]
        poseCovariance = Covariance(covarianceMatrix)
        self.pose = VisionPositionEstimate(time,positionBody,angleBody,poseCovariance)
        self.meas1_received = True

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

        # print('getting parameter')
        # aidMask = await self.drone.param.get_param_int('EKF2_AID_MASK')
        # print('aidMask = ', aidMask)
        print('setting aid mask parameter')
        await self.drone.param.set_param_int('EKF2_AID_MASK',8)
        # print('getting parameter')
        # aidMask = await self.drone.param.get_param_int('EKF2_AID_MASK')
        # print('aidMask = ', aidMask)

        # print('getting parameter')
        # height_mode = await self.drone.param.get_param_int('EKF2_HGT_MODE')
        # print('height_mode = ', height_mode)
        print('setting height mode parameter')
        await self.drone.param.set_param_int('EKF2_HGT_MODE',3)
        # print('getting parameter')
        # height_mode = await self.drone.param.get_param_int('EKF2_HGT_MODE')
        # print('height_mode = ', height_mode)

        # print('getting parameter')
        # ev_delay = await self.drone.param.get_param_float('EKF2_EV_DELAY')
        # print('ev_delay = ', ev_delay)
        print('setting ev delay parameter')
        await self.drone.param.set_param_float('EKF2_EV_DELAY',175.0)
        # print('getting parameter')
        # ev_delay = await self.drone.param.get_param_float('EKF2_EV_DELAY')
        # print('ev_delay = ', ev_delay)

        # print('getting parameters')
        # ev_pos_x = await self.drone.param.get_param_float('EKF2_EV_POS_X')
        # ev_pos_y = await self.drone.param.get_param_float('EKF2_EV_POS_Y')
        # ev_pos_z = await self.drone.param.get_param_float('EKF2_EV_POS_Z')
        # print('ev_pos_x = ', ev_pos_x)
        # print('ev_pos_y = ', ev_pos_y)
        # print('ev_pos_z = ', ev_pos_z)
        print('setting ev pos parameters')
        await self.drone.param.set_param_float('EKF2_EV_POS_X',0.0)
        await self.drone.param.set_param_float('EKF2_EV_POS_Y',0.0)
        await self.drone.param.set_param_float('EKF2_EV_POS_Z',0.0)
        # print('getting parameters')
        # ev_pos_x = await self.drone.param.get_param_float('EKF2_EV_POS_X')
        # ev_pos_y = await self.drone.param.get_param_float('EKF2_EV_POS_Y')
        # ev_pos_z = await self.drone.param.get_param_float('EKF2_EV_POS_Z')
        # print('ev_pos_x = ', ev_pos_x)
        # print('ev_pos_y = ', ev_pos_y)
        # print('ev_pos_z = ', ev_pos_z)

        print("Start updating position")
        asyncio.create_task(self.offboard_position_measurement_callback())

        # print("-- Arming")
        # await self.drone.action.arm()

        # print("-- Setting initial setpoint")
        # await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        # print("-- Starting offboard")
        # try:
        #     await self.drone.offboard.start()
        # except OffboardError as error:
        #     print(f"Starting offboard mode failed with error code: \
        #         {error._result.result}")
        #     print("-- Disarming")
        #     await self.drone.action.disarm()
        #     return

        # print("-- Go up 2 m/s")
        # await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
        # await asyncio.sleep(4)

        asyncio.create_task(self.publish_telemetry_odom)
        # asyncio.create_task(self.offboard_velocity_command_callback())

    async def offboard_position_measurement_callback(self):
        while(1):
            if self.pose.time_usec != self.prevPoseTime and self.meas1_received:
                await self.drone.mocap.set_vision_position_estimate(self.pose)
                self.prevPoseTime = self.pose.time_usec

    async def publish_telemetry_odom(self):
        async for odom in self.drone.telemetry.odometry():
            self.publish_estimate(odom.position_body)

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
