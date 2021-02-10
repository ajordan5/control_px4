import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class StateMachine:
    def __init__(self):
        self.missionState = 0 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land

        self.firstWaypoint = [0.0,0.0,-2.0]
        self.hlc = self.firstWaypoint
        self.antennaOffset = [0.0,0.0,-0.5]

        self.rendevousThreshold = 0.3
        self.descendThreshold = 0.3
        self.descendHeight = -2.0
        self.landingThreshold = 0.1
        self.landingHeight = -0.15

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]

        self.hlcMsg = Point()
        self.beginLandingRoutineMsg = Bool()

        self.boat_sub_ = rospy.Subscriber('boat_pos', Point, self.boatCallback, queue_size=5)
        self.rover_sub_ = rospy.Subscriber('rover_pos',Point,self.roverCallback,queue_size=5)
        self.hlc_pub_ = rospy.Publisher('hlc',Point,queue_size=5,latch=True)
        self.begin_landing_routine_pub_ = rospy.Publisher('begin_landing_routine',Bool,queue_size=5,latch=True)

        while not rospy.is_shutdown():
            rospy.spin()

    def boatCallback(self,msg):
        self.boatNed = [msg.x,msg.y,msg.z]

    def roverCallback(self,msg):
        self.roverNed = [msg.x,msg.y,msg.z]
        self.update_hlc()

    def update_hlc(self):
        if self.missionState == 1:
            self.rendevous()
        elif self.missionState == 2:
            self.descend()
        elif self.missionState == 3:
            self.land()
        else:
            self.fly_mission()

    def fly_mission(self):
        self.hlc = self.firstWaypoint
        self.publish_hlc()
        error = np.linalg.norm(np.array(self.hlc)-np.array(self.roverNed))
        if error < self.rendevousThreshold:
            self.missionState = 1
            print('rendevous state')
            self.beginLandingRoutineMsg.data = True
            self.begin_landing_routine_pub_.publish(self.beginLandingRoutineMsg)
        
    def rendevous(self):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,self.descendHeight]) + np.array(self.antennaOffset)
        self.publish_hlc()
        error = np.linalg.norm(np.array(self.hlc)-np.array(self.roverNed))
        if error < self.descendThreshold:
            self.missionState = 2
            print('descend state')

    def descend(self):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,self.landingHeight]) + np.array(self.antennaOffset)
        self.publish_hlc()
        error = np.linalg.norm(np.array(self.hlc)-np.array(self.roverNed))
        if error < self.landingThreshold:
            self.missionState = 3
            print('land state')

    def land(self):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,1.0]) + np.array(self.antennaOffset) #multirotor attemptes to drive itself into the platform 1 meter deep.
        self.publish_hlc()

    def publish_hlc(self):
        self.hlcMsg.x = self.hlc[0]
        self.hlcMsg.y = self.hlc[1]
        self.hlcMsg.z = self.hlc[2]
        self.hlc_pub_.publish(self.hlcMsg)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        