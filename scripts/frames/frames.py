#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R

def calculate_frame_error(velocityFrame1,velocityFrame2):
    errorRad = np.arctan2(velocityFrame2[0],velocityFrame2[1]) - np.arctan2(velocityFrame1[0],velocityFrame1[1])
    errorDeg = errorRad*180/np.pi
    return errorDeg

def rotate_to_inertial_frame(velocityBody,bodyQuat):
    Rb2i = R.from_quat(bodyQuat)
    velocityInertial = Rb2i.apply(velocityBody)
    return velocityInertial