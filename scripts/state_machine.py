#!/usr/bin/env python3

from os import stat_result
from black import err
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
        self.safeHeight = rospy.get_param('~safeHeight', -1.5)
        self.transition = rospy.get_param('~transition', 0.05)
        self.order = rospy.get_param('~order', 4)
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
        self.returnHeight = self.rendezvousHeight
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
        # TODO, uncomment this, just testing ff in sim
        #self.feedForwardVelocity[0] = msg.twist.twist.linear.x
        #self.feedForwardVelocity[1] = msg.twist.twist.linear.y
        #self.feedForwardVelocity[2] = msg.twist.twist.linear.z
        #print(np.linalg.norm(np.array(self.feedForwardVelocity)))

        self.Rb2i = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])      

    def update_hlc(self):
        commands = self.land()

        self.publish_hlc(commands)


    def land(self):
        # Land by aiming for a target slightly below the center of the landing pad
        error = self.field_manager()#np.array(self.rover2BaseRelPos) + np.array([0.0,0.0,0.4]) + self.Rb2i.apply(np.array(self.antennaOffset))
        #velocityCommand = self.saturate(self.landing_kp * error) + self.feedForwardVelocity
        velocityCommand = self.landing_kp * error + self.feedForwardVelocity

        print("v", velocityCommand)
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

    def field_manager(self):
        """Determine where in the vector field the rover is located and return a velocity command in the corresponding direction"""
        error = np.array(self.rover2BaseRelPos) + self.Rb2i.apply(np.array(self.antennaOffset))
        xyError = np.linalg.norm(error[:2])
        zError = abs(error[2])
        cone_radius = self.coneRadius(zError)
        print("e:", error)
         
        # In Cone
        if xyError < cone_radius - self.transition:
            return error/2

        # Out of Cone
        elif xyError > cone_radius:
            # Evaluate the slope perpendicular to the specified polynomial
            perpendicular = np.arctan(self.order * xyError ** (self.order - 1)) + np.pi/2
            lateral = abs(np.cos(perpendicular))
            longitudinal = np.sin(perpendicular)
            xDir = error[0]/xyError * lateral#np.cos(theta) * lateral
            yDir = error[1]/xyError#np.sin(theta) * lateral
            return np.array([xDir, yDir, longitudinal])

        # Transition between the two
        else:
            # Linear transition of the two fields
            to_cone = cone_radius - xyError
            k1 = to_cone/self.transition
            k2 = (self.transition - to_cone)/ self.transition
            # Outer field
            perpendicular = np.arctan(self.order * xyError ** (self.order - 1)) + np.pi/2
            lateral = np.cos(perpendicular)
            longitudinal = np.sin(perpendicular)
            theta = np.arctan2(error[1], error[0])
            xDir = -error[0]/xyError * lateral#np.cos(theta) * lateral
            yDir = -error[1]/xyError * lateral#np.sin(theta) * lateral
            outer =  k2 * np.array([xDir, yDir, longitudinal])
            # Inner field
            inner = k1 * error/2
            return inner + outer

    def coneRadius(self, zError):
        """Radius of defined cone for descent state"""
        return self.landingThreshold  + (self.rendezvousThreshold - self.landingThreshold) \
            *((-zError + self.landingHeight) / (-self.rendezvousHeight + self.landingHeight))

    def safeReturn(self, zError):
        """Save a safe return height in case vehicle exits cone. Execute go around if too close to the pad"""
        if self.rover2BaseRelPos[2] > -self.safeHeight:
            self.returnHeight = -self.rover2BaseRelPos[2]
        else:
            self.returnHeight = self.safeHeight - 0.5
        #print("in", self.returnHeight, zError) 

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
        
