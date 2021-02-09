#!/usr/bin/env python3


import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import navpy


class VelCntrl:
    def __init__(self):
        self.pos_desired = [0.0,0.0,-2.0]
        self.KpX = 1.0
        self.KpY = 1.0
        self.KpZ = 1.0
        self.ref_set = False

    async def run(self):
        """ Does Offboard control using velocity NED coordinates. """

        drone = System()
        await drone.connect(system_address="udp://:14540")

        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered with UUID: {state.uuid}")
                break

        print("-- Arming")
        await drone.action.arm()

        print("-- Setting initial setpoint")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

        async for lla in drone.telemetry.position():
            if self.ref_set:
                cmdVel = self.get_commands(lla)
                await drone.offboard.set_velocity_ned(VelocityNedYaw(cmdVel[0],cmdVel[1],cmdVel[2], 0.0))
            else:
                self.lat_ref = lla.latitude_deg
                self.lon_ref = lla.longitude_deg
                self.alt_ref = lla.absolute_altitude_m
                self.ref_set = True

        # await asyncio.sleep(50)

        # print("-- Go up 2 m/s")
        # await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
        # await asyncio.sleep(4)

        # print("-- Go North 2 m/s, turn to face East")
        # await drone.offboard.set_velocity_ned(VelocityNedYaw(2.0, 0.0, 0.0, 90.0))
        # await asyncio.sleep(4)

        # print("-- Go South 2 m/s, turn to face West")
        # await drone.offboard.set_velocity_ned(
        #     VelocityNedYaw(-2.0, 0.0, 0.0, 270.0))
        # await asyncio.sleep(4)

        # print("-- Go West 2 m/s, turn to face East")
        # await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, -2.0, 0.0, 90.0))
        # await asyncio.sleep(4)

        # print("-- Go East 2 m/s")
        # await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 2.0, 0.0, 90.0))
        # await asyncio.sleep(4)

        # print("-- Turn to face South")
        # await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 180.0))
        # await asyncio.sleep(2)

        # print("-- Go down 1 m/s, turn to face North")
        # await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 1.0, 0.0))
        # await asyncio.sleep(4)

        # print("-- Stopping offboard")
        # try:
        #     await drone.offboard.stop()
        # except OffboardError as error:
        #     print(f"Stopping offboard mode failed with error code: \
        #         {error._result.result}")

    def get_commands(self,lla):
        ned = navpy.lla2ned(lla.latitude_deg,lla.longitude_deg,lla.absolute_altitude_m,self.lat_ref,self.lon_ref,self.alt_ref)
        cmdX = self.KpX*(self.pos_desired[0] - ned[0])
        cmdY = self.KpY*(self.pos_desired[1] - ned[1])
        cmdZ = self.KpZ*(self.pos_desired[2] - ned[2])
        cmd = [cmdX,cmdY,cmdZ]
        return cmd


if __name__ == "__main__":
    vel_cntrl = VelCntrl()
    asyncio.ensure_future(vel_cntrl.run())
    loop = asyncio.get_event_loop()
    loop.run_forever()
