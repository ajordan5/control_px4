#!/usr/bin/env python3

from os import stat_result
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from ublox.msg import RelPos
from ublox.msg import PosVelEcef
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R


class StateMachine:
    def __init__(self):
        self.missionState = 1 #0 - mission
                              #1 - rendezvous
                              #2 - descend
                              #3 - return
                              #4 - land
        self.waypoints = rospy.get_param('~waypoints', [[0.0,0.0,-2.0]])
        self.hlc = Odometry()
        self.antennaOffset = rospy.get_param('~antennaOffset', [0.0,0.0,0.0])

        self.missionThreshold = rospy.get_param('~missionThreshold', 0.3)
        self.rendezvousThreshold = rospy.get_param('~rendezvousThreshold', 0.3)
        self.rendezvousCylinder = rospy.get_param('~rendezvousCylinder', 0.3)
        self.rendezvousTime = rospy.get_param('~rendezvousTime', 0.1)
        self.rendezvousHeight = rospy.get_param('~rendezvousHeight', -2.0)
        self.landingThreshold = rospy.get_param('~landingThreshold', 0.1)
        self.landingCylinder = rospy.get_param('~landingCylinder', 0.1)
        self.landingTime = rospy.get_param('~landingTime', 0.02)
        self.baseXYAttitudeThreshold = rospy.get_param('~baseXYAttitudeThreshold',8.0)
        self.landingHeight = rospy.get_param('~landingHeight', -0.5)
        self.safeHeight = rospy.get_param('~landingHeight', -0.5)
        self.autoLand = rospy.get_param('~autoLand', False)
        self.cyclicalPath = rospy.get_param('~cyclicalPath', False)
        self.max_descend = rospy.get_param('~maxDescendRate', 5)

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]
        self.rover2BaseRelPos = [0.0,0.0,0.0]
        self.relVel = [0.0,0.0,0.0]
        self.Rb2i = R.from_quat([0.0,0.0,0.0,1.0]) 
        self.currentWaypointIndex = 0
        self.feedForwardVelocity = [0.0,0.0,0.0]
        self.hlcMsg = PoseStamped()
        self.beginLandingRoutineMsg = Bool()
        
        self.hlc_pub_ = rospy.Publisher('hlc',Odometry,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)
        self.mission_state_pub = rospy.Publisher('mission_state',PointStamped,queue_size=5,latch=True)

        # Velocity Control Parameters
        self.position_kp = np.array(rospy.get_param('~positionKp', [0.95, 0.95, 1.0]))
        self.landing_kp = np.array(rospy.get_param('~landingKp', [0.95, 0.95, 1.0]))

        # Mission threshold timing
        self.in_threshold = False   # Indicates whether the timer has started
        self.in_cylinder = False    # Indicates whether vehicle is in the spatial threshold regardless of the timer

        self.publish_mission_state()

        self.odom_sub_ = rospy.Subscriber('rover_odom',Odometry,self.odomCallback, queue_size=5)
        self.base_odom_sub_ = rospy.Subscriber('base_odom',Odometry,self.baseOdomCallback, queue_size=5)
        self.rel_vel_sub_ = rospy.Subscriber('rel_vel',Odometry,self.relVelCallback, queue_size=5)
        while not rospy.is_shutdown():
            rospy.spin()

    def odomCallback(self,msg):
        self.odom = [msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]
        self.update_hlc()
    
    def relVelCallback(self,msg):
        self.relVel[0] = msg.twist.twist.linear.x
        self.relVel[1] = msg.twist.twist.linear.y
        self.relVel[2] = 0 #msg.twist.twist.linear.z
        self.relVelNorm = np.linalg.norm(np.array(self.relVel))

    def baseOdomCallback(self,msg):
        self.rover2BaseRelPos[0] = msg.pose.pose.position.x
        self.rover2BaseRelPos[1] = msg.pose.pose.position.y
        self.rover2BaseRelPos[2] = msg.pose.pose.position.z
        self.feedForwardVelocity[0] = msg.twist.twist.linear.x
        self.feedForwardVelocity[1] = msg.twist.twist.linear.y
        self.feedForwardVelocity[2] = msg.twist.twist.linear.z
        #print(np.linalg.norm(np.array(self.feedForwardVelocity)))

        self.Rb2i = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])      

    def update_hlc(self):
        if self.missionState == 1:
            commands = self.rendezvous()
        elif self.missionState == 2:
            commands = self.descend()
        elif self.missionState == 3:
            commands = self.goaround()
        elif self.missionState == 4:
            commands = self.land()
        else:
            #commands = self.rendezvous()
            commands = self.fly_mission()

        self.publish_hlc(commands)

    def fly_mission(self):
        currentWaypoint = self.waypoints[self.currentWaypointIndex]
        error = np.linalg.norm(np.array(currentWaypoint)-np.array(self.odom))
        velocityCommand = self.position_kp * (np.array(currentWaypoint)-np.array(self.odom))
        if error < self.missionThreshold:
            print('reached waypoint ', self.currentWaypointIndex + 1)
            print(currentWaypoint)
            self.currentWaypointIndex += 1
            if self.currentWaypointIndex == len(self.waypoints) and self.autoLand == True:
                self.missionState = 1
                self.publish_mission_state()
                print('rendezvous state')
                self.beginLandingRoutineMsg.data = True
                self.begin_landing_routine_pub_.publish(self.beginLandingRoutineMsg)
            if self.cyclicalPath:
                self.currentWaypointIndex %= len(self.waypoints)            
            elif self.currentWaypointIndex == len(self.waypoints):
                self.currentWaypointIndex -=1
        return velocityCommand

    def rendezvous(self):
        # Error from desired waypoint. Determine if vehicle is within a cylinder around waypoint
        error = self.cylinderError(self.rendezvousHeight, self.rendezvousThreshold, self.rendezvousCylinder)

         # Entered threshold
        if self.in_cylinder and self.in_threshold == False:
            self.start_threshold_timer()  
        # Already in threshold
        elif self.in_cylinder and self.in_threshold == True:
            
            # Check how long inside threshold TODO instead of saving threshold time, you could just have the timer return the value
            if self.threshold_timer() > self.rendezvousTime:
                self.missionState = 2
                self.publish_mission_state()
                print('descend state')
                self.in_threshold = False
                self.in_cylinder = False
        # Exited threshold
        elif not self.in_cylinder and self.in_threshold == True:
            self.in_threshold = False
            print("EXITED THRESHOLD")

        velocityCommand = self.position_kp * error + self.feedForwardVelocity
        #print(velocityCommand)
        return velocityCommand

    def descend(self):
        # Descend towards pad if vehicle within cylinder theshold
        error = self.coneError()
        euler = self.Rb2i.as_euler('xyz')
        baseXYAttitude = np.abs(euler[0:2])
        max_tilt = np.amax(np.degrees(baseXYAttitude))
        velocityCommand = self.saturate(self.position_kp * error) + self.feedForwardVelocity
            
         # Entered threshold
        if self.in_cylinder and self.in_threshold == False:
            self.start_threshold_timer()
        # Already in threshold
        elif self.in_cylinder and self.in_threshold == True:
            
            # Check how long inside threshold
            if self.threshold_timer() > self.landingTime and max_tilt < self.baseXYAttitudeThreshold:
                self.missionState = 4
                self.publish_mission_state()
                print('land state')
        # Exited threshold
        elif not self.in_cylinder and self.in_threshold == True:
            self.in_threshold = False
            print("EXITED THRESHOLD")
      
        return velocityCommand

    def goaround(self):
        # Return to a safe height if vehicle is dangerously close to the pad, but fell out of the cone threshold
        # Error from desired waypoint. Determine if vehicle is within a cylinder around waypoint
        error = self.cylinderError(self.returnHeight, self.landingThreshold, self.landingCylinder)

         # Entered threshold
        if self.in_cylinder and self.in_threshold == False:
            self.start_threshold_timer()  
        # Already in threshold
        elif self.in_cylinder and self.in_threshold == True:  
            # Check how long inside threshold
            if self.threshold_timer() > self.rendezvousTime:
                self.missionState = 2
                self.publish_mission_state()
                print('descend state')
                self.in_threshold = False
                self.in_cylinder = False
        # Exited threshold
        elif not self.in_cylinder and self.in_threshold == True:
            self.in_threshold = False
            print("EXITED GOAROUND THRESHOLD")

        velocityCommand = self.position_kp * error + self.feedForwardVelocity
        #print(velocityCommand)
        return velocityCommand


    def land(self):
        # Land by aiming for a target slightly below the center of the landing pad
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,0.2]) + self.Rb2i.apply(np.array(self.antennaOffset))
        velocityCommand = self.saturate(self.landing_kp * error) + self.feedForwardVelocity
        return velocityCommand

    def threshold_timer(self):
        """Method to manage the timing of how long the rover has been in the threshold"""
        computer_time = rospy.Time.now()
        time = computer_time.secs + computer_time.nsecs * 1E-9
        timer = time - self.threshold_time_start
        print("TIME IN THRESHOLD:" ,timer)
        return timer
    
    def start_threshold_timer(self):
        """Start timing threshold"""
        print("ENTERED THRESHOLD")
        self.in_threshold = True
        computer_time = rospy.Time.now()
        self.threshold_time_start = computer_time.secs + computer_time.nsecs * 1E-9

    def cylinderError(self, height, radius, height_cylinder):
        """Manage the error to determine if vehicle is within a cylindrical threshold"""
        
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,height]) + self.Rb2i.apply(np.array(self.antennaOffset))
        # Simulate heave
        # ros_time = rospy.Time.now()
        # t = ros_time.secs + ros_time.nsecs * 1E-9
        # heave= 0.254*np.sin(2*np.pi/2 * t)
        # error[2] += heave
        xyError = np.linalg.norm(error[:2])
        zError = abs(error[2])
        if xyError < radius and zError < height_cylinder:
            self.in_cylinder=True
            self.returnHeight = error[2]
        else:
            self.in_cylinder=False
        return error
    
    def coneError(self):
        """Manage the error in descend state to determine if vehicle is within a conical threshold"""
        error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.landingHeight]) + self.Rb2i.apply(np.array(self.antennaOffset))
        xyError = np.linalg.norm(error[:2])
        zError = error[2]
        if xyError < self.coneRadius(-zError) and -zError < self.landingHeight and -zError > self.rendezvousHeight:
            self.in_cone=True
            self.safeReturn()
            if abs(zError) < self.landingCylinder:
                self.in_cylinder = True
            else:
                self.in_cylinder = False
        else:
            # Do not descend if outside cone, return to last known height within cone or a higher (safe) height if too close to the pad
            self.in_cone=False
            self.in_cylinder=False
            error = np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,self.returnHeight]) + self.Rb2i.apply(np.array(self.antennaOffset))
            self.missionState = 3
            self.publish_mission_state()
            print("return state")
            print("Exited cone, returning to a safe height")
        return error

    def coneRadius(self, zError):
        """Radius of defined cone for descent state"""
        return self.landingThreshold  + (self.rendezvousThreshold - self.landingThreshold) \
            *((zError - self.landingHeight) / (self.rendezvousHeight - self.landingHeight))

    def safeReturn(self, zError):
        """Save a safe return height in case vehicle exits cone"""
        if zError < self.safeHeight:
            self.returnHeight = zError
        else:
            self.returnHeight = self.safeHeight 


    def saturate(self, velocity):
        """Saturate the velocity to a safe descending speed"""
        if velocity[2] > self.max_descend:
            velocity[2] = self.max_descend
        return velocity

    def publish_hlc(self,commands):
        self.hlc.twist.twist.linear.x = commands[0]
        self.hlc.twist.twist.linear.y = commands[1]
        self.hlc.twist.twist.linear.z = commands[2]
        self.hlc.header.stamp = rospy.Time.now()
        self.hlc_pub_.publish(self.hlc)

    def publish_mission_state(self):
        missionState = PointStamped()
        missionState.header.stamp = rospy.Time.now()
        missionState.point.x = self.missionState
        self.mission_state_pub.publish(missionState)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        
