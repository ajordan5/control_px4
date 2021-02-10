import numpy as np
import rospy
from geometry_msgs.msg import Point


class StateMachine:
    def __init__(self):
        self.missionState = 0 #0 - mission
                              #1 - rendevous
                              #2 - descend
                              #3 - land

        self.first_waypoint = [0.0,0.0,-2.0]
        self.hlc = self.first_waypoint
        self.hlc_msg = Point()

        self.rendevous_threshold = 0.3
        self.descend_threshold = 0.3
        self.descent_height = -2.0
        self.landing_threshold = 0.3
        self.landing_height = -0.15

        self.roverNed = [0.0,0.0,0.0]
        self.boatNed = [0.0,0.0,0.0]

        self.boat_sub_ = rospy.Subscriber('boat_pos', Point, self.boatCallback, queue_size=5)
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
        self.hlc = self.first_waypoint
        self.publish_hlc()
        if error < self.rendevous_threshold:
            self.missionState = 1
        
    def rendevous(self,error):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,self.descent_height])
        self.publish_hlc()
        if error < self.descend_threshold:
            self.missionState = 2

    def descend(self,error):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,self.landing_height])
        self.publish_hlc()
        if error < self.landing_threshold:
            self.missionState = 3

    def land(self):
        self.hlc = np.array(self.boatNed) + np.array([0.0,0.0,1.0]) #multirotor attemptes to drive itself into the platform 1 meter deep.
        self.publish_hlc()

    def publish_hlc(self):
        self.hlc_msg.x = self.hlc[0]
        self.hlc_msg.y = self.hlc[1]
        self.hlc_msg.z = self.hlc[2]
        self.hlc_pub_.publish(self.hlc_msg)

if __name__ == "__main__":
    rospy.init_node('state_machine', anonymous=True)
    try:
        state_machine = StateMachine()
    except:
        rospy.ROSInterruptException
    pass
        