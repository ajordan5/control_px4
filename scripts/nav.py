#Not using this, until we have a need to
import rospy
from ublox.msg import RelPos

class Nav:
    def __init__(self):
        #TODO we may need to speed up command inputs in order to have good performance.
        #this node can be used to extrapolate relpos messages given rover estimate updates.
        #or we could potentially use base estimation for the extrapolation as well.
        self.rover_extrapolated_relPos_pub_ = rospy.Publisher('extrap_relPos', RelPos, queue_size=5, latch=True)

        self.rover_relPos_sub_ = rospy.Subscriber('/rover_relPos', RelPos, self.roverRelPosCallback, queue_size=5)
        # self.px4_estimate_sub_ = rospy.Subscriber('/px4_estimate', PoseStamped, self.px4EstimateCallback, queue_size=5)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()

    def roverRelPosCallback(self, msg):
        self.rover_extrapolated_relPos_pub_.publish(msg)
