#!/usr/bin/env python3

import asyncio

from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0:921600")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    asyncio.create_task(get_status())
    print('sleep 5')
    await asyncio.sleep(5)
    await self.drone.action.arm()
    await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await self.drone.offboard.start()
    
async def get_status():
    async for status in drone.telemetry.status_text():
        print("status update: ", status.text)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
