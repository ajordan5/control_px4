import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyUSB0:921600")

    print("Waiting for drone to connect...")
    await asyncio.sleep(5)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
    counter = 1
    async for odom in drone.telemetry.odometry():
        print('in odom')
        counter = counter+1
        if counter == 10:
            break

    #message = await drone.telemetry.get_gps_global_origin()
    #print('message = ', message)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
