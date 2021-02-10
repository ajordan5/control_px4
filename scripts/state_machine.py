import numpy as np
import rospy
from geometry_msgs.msg import Point


class StateMachine:
    def __init__(self):
        self.missionState = 0 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land

        self.firstWaypoint = [0.0,0.0,-2.0]
        self.hlc = self.firstWaypoint
        self.hlcMsg = Point()

        self.rendevousThreshold = 0.3
        self.descendThreshold = 0.3
        self.descentHeight = -2.0
        self.landingThreshold = 0.3
        self.landingHeight = -0.15

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]

        self.boat_sub_ = rospy.Subscriber('boat_pos', Point, self.boatCallback, queue_size=5)
        self.rover_sub_ = rospy.Subscriber('rover_pos',Point,self.roverCallback,queue_size=5)
        self.hlc_pub_ = rospy.Publisher('hlc',Point,queue_size=5,latch=True)

    def boatCallback(self,msg):
        self.boatNed = [msg.x,msg.y,msg.z]

    def roverCallback(self,msg):
        self.roverNed = [msg.x,msg.y,msg.z]
        self.update_hlc()

    def update_hlc(self):
        error = np.linalg.norm(np.array(self.hlc)-np.array(self.roverNed))
        if self.missionState == 1:
            self.rendevous(error)
        elif self.missionState == 2:
            self.descend(error)
        elif self.missionState == 3:
            self.land()
        else:
            self.fly_mission(error)

    def fly_mission(self,error):
        self.hlc = self.firstWaypoint
        self.publish_hlc()
        if error < self.rendevousThreshold:
            self.missionState = 1
        
    def rendevous(self,error):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,self.descentHeight])
        self.publish_hlc()
        if error < self.descendThreshold:
            self.missionState = 2

    def descend(self,error):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,self.landingHeight])
        self.publish_hlc()
        if error < self.landingThreshold:
            self.missionState = 3

    def land(self):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,1.0]) #multirotor attemptes to drive itself into the platform 1 meter deep.
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
        