# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        dt = params.dt
        #State transition matrix, position x,y,z and velocity x, y, z. Constant velocity model.
        #Note:
        #Although it is an EKF, the state transition matrix is linear and therefore f and Fj are not needed
        return np.matrix([[1, 0, 0, dt,  0,  0],
                          [0, 1, 0,  0, dt,  0],
                          [0, 0, 1,  0,  0, dt],
                          [0, 0, 0,  1,  0,  0],
                          [0, 0, 0,  0,  1,  0],
                          [0, 0, 0,  0,  0,  1]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # Process noise covariance Q, it is assumed that position prediction is correct, 
        # but velocity prediction has uncertainty due to constant velocity model
        q = params.q
        dt = params.dt
        q1 = ((dt**3)/3) * q 
        q2 = ((dt**2)/2) * q 
        q3 = dt * q 
        return np.matrix([[q1, 0,  0, q2,  0,  0],
                          [0, q1,  0,  0, q2,  0],
                          [0,  0, q1,  0,  0, q2],
                          [q2, 0,  0, q3,  0,  0],
                          [ 0, q2, 0,  0, q3,  0],
                          [ 0,  0, q2, 0,  0, q3]])

        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # predict state and estimation error covariance to next timestep
        # It is an EKF but state transistion matrix is linear, so f(x) and Fj are not needed
        F = self.F()
        x = F * track.x # state prediction
        P = F * track.P * F.transpose() + self.Q() # covariance prediction

        #save x and P in track
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # update state x and covariance P with associated measurement
        # It is an EKF because measurment function may be non-linear for some sensor, so h(x) and Hj are needed
        x = track.x # get current state
        Hj = meas.sensor.get_H(x) # measurement matrix (Jacobian at current state)
        P = track.P # Uncertainty covariance
        S_l = self.S(track, meas, Hj) # covariance of residual
        K = P * Hj.transpose() * np.linalg.inv(S_l) # Kalman gain
        x = x + K * self.gamma(track, meas) # state update
        I = np.identity(x.shape[0])
        P = (I - K * Hj) * P # covariance update

        #save x and P in track
        track.set_x(x)
        track.set_P(P)

        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # calculate and return residual gamma
        # It is an EKF because measurment function may be non-linear for some sensor, so h(x) and Hj are needed
        x = track.x # get current state
        return meas.z - meas.sensor.get_hx(x) # residual at current state
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # calculate and return covariance of residual S
        return H * track.P * H.transpose() + meas.R # covariance of residual
        
        ############
        # END student code
        ############ 