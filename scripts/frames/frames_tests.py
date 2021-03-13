import numpy as np
from scipy.spatial.transform import Rotation as R

from frames import *
import unittest

class TestFrames(unittest.TestCase):
    def __init__(self):
        self.test_45_offset()
        self.test_180_offset()
        self.test_negative_90_offset()
        self.test_90_yaw()
        self.test_30_yaw()
        self.test_180_yaw()
        self.test_negative_45_yaw()
        self.test_everything()

    def test_45_offset(self):
        frame1 = [1.0,1.0,0.0]
        frame2 = [1.0,0.0,0.0]
        expectedError = 45.0
        self.test_error(frame1,frame2,expectedError)

    def test_180_offset(self):
        frame1 = [-1.0,0.0,0.0]
        frame2 = [1.0,0.0,0.0]
        expectedError = 180.0
        self.test_error(frame1,frame2,expectedError)

    def test_negative_90_offset(self):
        frame1 = [1.0,0.0,0.0]
        frame2 = [0.0,3.0,0.0]
        expectedError = -90.0
        self.test_error(frame1,frame2,expectedError)

    def test_90_yaw(self):
        velocityBody = [1.0,-1.0,0.0]
        yawDeg = 90.0
        velocityInertialExpected = [1.0,1.0,0.0]
        self.test_body_to_inertial(velocityBody,yawDeg,velocityInertialExpected)

    def test_30_yaw(self):
        velocityBody = [2.0,0.0,0.0]
        yawDeg = 30.0
        yawRad = yawDeg * np.pi/180.0
        velocityInertialExpected = [2.0*np.cos(yawRad),2.0*np.sin(yawRad),0.0]
        self.test_body_to_inertial(velocityBody,yawDeg,velocityInertialExpected)

    def test_180_yaw(self):
        velocityBody = [-1.0,-1.0,0.0]
        yawDeg = 180.0
        velocityInertialExpected = [1.0,1.0,0.0]
        self.test_body_to_inertial(velocityBody,yawDeg,velocityInertialExpected)

    def test_negative_45_yaw(self):
        velocityBody = [2.0,-1.0,0.0]
        yawDeg = -45.0
        yawRad = yawDeg*np.pi/180.0
        velocityInertialExpected = [2.0*np.cos(yawRad)+np.sin(yawRad),2.0*np.sin(yawRad)-np.cos(yawRad),0.0]
        self.test_body_to_inertial(velocityBody,yawDeg,velocityInertialExpected)

    def test_everything(self):
        velocityBody = [1.0,-1.0,0.0]
        yawDeg = 90.0
        velocityFrame1 = [0.0,1.0,0.0]
        expectedError = 45.0

        Rb2i = R.from_euler('z', yawDeg, degrees=True)
        bodyQuat = Rb2i.as_quat()
        velocityInertialCalculated = rotate_to_inertial_frame(velocityBody,bodyQuat)
        
        calculatedError = calculate_frame_error(velocityFrame1,velocityInertialCalculated)
        self.assertAlmostEqual(calculatedError,expectedError)

    def test_error(self,frame1,frame2,expectedError):
        calculatedError = calculate_frame_error(frame1,frame2)
        self.assertAlmostEqual(calculatedError,expectedError)

    def test_body_to_inertial(self,velocityBody,yawDeg,velocityInertialExpected):
        Rb2i = R.from_euler('z', yawDeg, degrees=True)
        bodyQuat = Rb2i.as_quat()
        velocityInertialCalculated = rotate_to_inertial_frame(velocityBody,bodyQuat)
        self.assertAlmostEqual(velocityInertialCalculated[0],velocityInertialExpected[0])
        self.assertAlmostEqual(velocityInertialCalculated[1],velocityInertialExpected[1])
        self.assertAlmostEqual(velocityInertialCalculated[2],velocityInertialExpected[2])

if __name__ == '__main__':
    testFrames = TestFrames()