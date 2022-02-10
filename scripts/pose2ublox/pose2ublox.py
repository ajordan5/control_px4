#!/usr/bin/env python3

import navpy
import numpy as np
from scipy.spatial.transform import Rotation as R

class Pose2Ublox():
    

    def __init__(self, Ts, gha, gva, gsa, rha, rva, rsa, cha, no, rl, srp, srv, srr, sbp, sbv, lo, A, B):

        #parameters
        self.Ts = Ts
        self.global_horizontal_accuracy = gha
        self.global_vertical_accuracy = gva
        self.global_speed_accuracy = gsa
        self.relative_horizontal_accuracy = rha
        self.relative_vertical_accuracy = rva
        self.relative_speed_accuracy = rsa
        self.compassing_heading_accuracy = cha
        self.noise_on = no
        self.ref_lla = rl
        self.sigma_rover_pos = srp 
        self.sigma_rover_vel = srv
        self.sigma_rover_relpos = srr
        self.sigma_base_pos = sbp
        self.sigma_base_vel = sbv
        self.lpf_on = lo
        self.A = A
        self.B = B
        self.Kgps = 1100
        self.eta_h = 0.03
        self.eta_a = 0.06

        #calc ref_ecef from ref_lla
        self.F = (self.A - self.B)/self.A     # Ellipsoid Flatness
        self.E2 = self.F * (2.0 - self.F);    # Square of Eccentricity    
        self.ref_ecef = navpy.lla2ecef(self.ref_lla[0],self.ref_lla[1],self.ref_lla[2])

        #combine standard deviations
        self.ned_std_dev_3d = [self.global_horizontal_accuracy, self.global_horizontal_accuracy, self.global_vertical_accuracy]
        self.vel_std_dev_3d = [self.global_speed_accuracy, self.global_speed_accuracy, self.global_speed_accuracy]
        self.relpos_std_dev_3d = [self.relative_horizontal_accuracy, self.relative_horizontal_accuracy, self.relative_vertical_accuracy]
        self.white_noise_3d = [self.eta_h, self.eta_h, self.eta_a]

        #other needed variables and arrays
        self.rover_ned = np.zeros(3)
        self.rover_ned_prev = np.zeros(3)
        self.rover_ned_lpf = np.zeros(3)
        self.rover_vel_lpf = np.zeros(3)
        self.rover_prev_time = 0.0
        self.rover_relpos_lpf = np.zeros(3)
        self.compass_quat = np.array([0.0,0.0,0.0,1.0])
        self.base_ned = np.zeros(3)
        self.base_ned_prev = np.zeros(3)
        self.base_ned_lpf = np.zeros(3)
        self.base_vel_lpf = np.zeros(3)
        self.base_prev_time = 0.0
        self.rover_ned_noise = np.zeros(3)
        self.rover_vel_noise = np.zeros(3)
        self.rover_vel_noise_prev = np.zeros(3)
        self.base_ned_noise = np.zeros(3)
        self.base_vel_noise = np.zeros(3)
        self.base_vel_noise_prev = np.zeros(3)

        #Outputs
        self.rover_virtual_pos_ecef = np.zeros(3)
        self.rover_virtual_lla = self.ref_lla
        self.rover_virtual_vel_ecef = np.zeros(3)
        self.rover_virtual_relpos = np.zeros(3)
        self.base_virtual_pos_ecef = np.zeros(3)
        self.base_virtual_vel_ecef = np.zeros(3)
        self.compass_heading = 0.0

    def update_rover_virtual_PosVelEcef(self, dt):

        #calculate virtual position in ecef frame with noise
        self.rover_ned_noise = self.add_gps_noise(self.white_noise_3d, self.rover_ned_noise, self.sigma_rover_pos)
        rover_ned_w_noise = self.rover_ned + self.rover_ned_noise
        self.rover_virtual_lla = navpy.ned2lla(rover_ned_w_noise,self.ref_lla[0],self.ref_lla[1],self.ref_lla[2])
        self.rover_virtual_pos_ecef = navpy.lla2ecef(self.rover_virtual_lla[0],self.rover_virtual_lla[1],self.rover_virtual_lla[2])

        #calculate virtual velocity in ecef frame with noise
        #make sure we do not divide by zero
        if dt != 0.0:
            rover_vel = (self.rover_ned - self.rover_ned_prev)/dt
        else:
            rover_vel = np.zeros(3)
        self.rover_vel_noise = self.add_noise_3d(np.zeros(3), self.white_noise_3d)
        self.rover_vel_lpf = self.lpf(self.rover_vel_noise, self.rover_vel_noise_prev, self.Ts, self.sigma_rover_vel)
        rover_vel_w_noise = rover_vel + self.rover_vel_lpf
        self.rover_virtual_vel_ecef = navpy.ned2ecef(rover_vel_w_noise,self.ref_lla[0],self.ref_lla[1],self.ref_lla[2]) #self.ned2ecef(rover_vel_w_noise, self.ref_lla)

        #update histories
        self.rover_ned_prev = self.rover_ned
        self.rover_vel_prev = rover_vel
        self.rover_vel_noise_prev = self.rover_vel_noise

    def update_rover_virtual_relPos(self):
        #calculate virtual relative position of the rover with respect to the base in ned frame with noise.
        relpos_array = self.rover_ned - self.base_ned
        rover_relpos_noise = self.add_noise_3d(relpos_array, self.relpos_std_dev_3d)
        self.rover_relpos_lpf = self.lpf(rover_relpos_noise, self.rover_relpos_lpf, self.Ts, self.sigma_rover_relpos)

        self.rover_virtual_relpos = self.rover_relpos_lpf

    def update_base_virtual_PosVelEcef(self, dt):

        #calculate virtual position in ecef frame with noise
        self.base_ned_noise = self.add_gps_noise(self.white_noise_3d, self.base_ned_noise, self.sigma_base_pos)
        base_ned_w_noise = self.base_ned + self.base_ned_noise
        self.base_virtual_lla = navpy.ned2lla(base_ned_w_noise,self.ref_lla[0],self.ref_lla[1],self.ref_lla[2])
        self.base_virtual_pos_ecef = navpy.lla2ecef(self.base_virtual_lla[0],self.base_virtual_lla[1],self.base_virtual_lla[2])
        
        #calculate virtual velocity in ecef frame with noise
        #make sure we do not divide by zero
        if dt != 0.0:
            base_vel = (self.base_ned - self.base_ned_prev)/dt
        else:
            base_vel = np.zeros(3)
        self.base_vel_noise = self.add_noise_3d(np.zeros(3), self.white_noise_3d)
        self.base_vel_lpf = self.lpf(self.base_vel_noise, self.base_vel_noise_prev, self.Ts, self.sigma_base_vel)
        base_vel_w_noise = base_vel + self.base_vel_lpf
        self.base_virtual_vel_ecef = navpy.ned2ecef(base_vel_w_noise,self.ref_lla[0],self.ref_lla[1],self.ref_lla[2]) #self.ned2ecef(base_vel_w_noise, self.ref_lla)

        #update histories
        self.base_ned_prev = self.base_ned
        self.base_vel_prev = base_vel
        self.base_vel_noise_prev = self.base_vel_noise

    def update_compass_virtual_relPos(self):
        euler = R.from_quat(self.compass_quat).as_euler('xyz',degrees=False)
        self.compass_heading = euler[2]
        if self.noise_on:
            self.compass_heading = np.random.normal(self.compass_heading, self.compassing_heading_accuracy)


    def add_noise_3d(self, value, std_dev):
        
        if self.noise_on:
            value_w_noise_1 = np.random.normal(value[0], std_dev[0])
            value_w_noise_2 = np.random.normal(value[1], std_dev[1])
            value_w_noise_3 = np.random.normal(value[2], std_dev[2])
            value_w_noise = np.array([value_w_noise_1, value_w_noise_2, value_w_noise_3])
            return value_w_noise

        else:
            return value


    def add_gps_noise(self, white_noise_3d, noise_prev, sigma):

        #Eq 7.17 Small Unmanned Aircraft (Beard, McLain) pg 139
        white_noise = self.add_noise_3d(np.zeros(3), self.white_noise_3d)
        color_noise = np.exp(-self.Kgps*self.Ts)*noise_prev
        total_noise = white_noise + color_noise
        noise = self.lpf(total_noise, noise_prev, self.Ts, sigma)

        return noise

    def lpf(self, xt, x_prev, dt, sigma):
        
        #low pass filter
        if self.lpf_on:
            x_lpf = xt*dt/(sigma+dt) + x_prev*sigma/(sigma+dt)
            return x_lpf
        
        else:
            return xt

