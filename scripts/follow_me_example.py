#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)

import rospy

from geometry_msgs.msg import Point


class FollowMe:
    async def initialize(self):
        self.ref_pub_ = rospy.Publisher('ref',Point,queue_size=5,latch=True)
        self.boat_sub_ = rospy.Subscriber('boat_pos', Point, self.boatCallback, queue_size=5)
        self.default_height = 8.0 #in Meters
        self.follow_distance = 1.0 #in Meters, this is the distance that the drone will remain away from Target while following it 
        #Direction relative to the Target 
        #Options are NONE, FRONT, FRONT_LEFT, FRONT_RIGHT, BEHIND
        self.direction = Config.FollowDirection.BEHIND
        self.responsiveness =  0.02
        self.ref_lla = Point()

    def boatCallback(self,msg):
        self.boat_lla = [msg.x,msg.y,msg.z]

    def publish_ref(self,ref_lat_deg,ref_lon_deg,ref_alt_asl_m):
        self.ref_lla.x = ref_lat_deg
        self.ref_lla.y = ref_lon_deg
        self.ref_lla.z = ref_alt_asl_m
        print('ref lla = ', [ref_lat_deg,ref_lon_deg,ref_alt_asl_m])
        self.ref_pub_.publish(self.ref_lla)

    async def fly_drone(self):
        print('in fly drone')
        self.drone = System()
        print('drone defined')
        await self.drone.connect(system_address="udp://:14540")
        print('drone connected')

        #This waits till a mavlink based drone is connected
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone with UUID: {state.uuid}")
                break

        await self.initialize()

        #Checking if Global Position Estimate is ok
        async for global_lock in self.drone.telemetry.health():
            if global_lock.is_global_position_ok:
                print("-- Global position state is good enough for flying.")
                break

        async for pos in self.drone.telemetry.position():
            self.publish_ref(pos.latitude_deg,pos.longitude_deg,pos.absolute_altitude_m)
            if pos.latitude_deg:
                break

        #Arming the drone
        print ("-- Arming")
        await self.drone.action.arm()
        
        #Follow me Mode requires some configuration to be done before starting the mode
        conf = Config(self.default_height, self.follow_distance, self.direction, self.responsiveness)
        await self.drone.follow_me.set_config(conf)
        
        print ("-- Taking Off")
        await self.drone.action.takeoff()
        await asyncio.sleep(8)
        print ("-- Starting Follow Me Mode")
        await self.drone.follow_me.start()
        await asyncio.sleep(8)

        while True:
            target = TargetLocation(self.boat_lla[0], self.boat_lla[1], 0, 0, 0, 0)
            await self.drone.follow_me.set_target_location(target)
            await asyncio.sleep(0.2)
        
        #Stopping the follow me mode
        print ("-- Stopping Follow Me Mode")
        await self.drone.follow_me.stop()
        await asyncio.sleep(5)
        
        print ("-- Landing")
        await self.drone.action.land()

if __name__ == "__main__":
    rospy.init_node('follow_me', anonymous=True)
    try:
        followMe = FollowMe()
        loop = asyncio.get_event_loop()
        loop.run_until_complete(followMe.fly_drone())
    except:
        rospy.ROSInterruptException
    pass

