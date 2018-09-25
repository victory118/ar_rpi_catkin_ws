#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers, pos_init):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = np.diag([0.02, 0.02])
        self.R_t = np.diag([1e-2, 1e-2, 1e-2])
        # YOUR CODE HERE
        
        self.x_t = pos_init # initialize state variables
        self.P_t = np.diag([10, 10, 10]) # initialize covariance matrix

    def prediction(self, v, omega, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        # YOUR CODE HERE
        #if self.last_time == None:
           # self.last_time = imu_meas[4][0]
          
        #dt = imu_meas[4][0] - self.last_time
        #self.last_time = imu_meas[4][0]
        dt = 1./1 # Called at 60Hz

        x_pre = self.x_t[0] + v*np.cos(self.x_t[2])*dt
        y_pre = self.x_t[1] + v*np.sin(self.x_t[2])*dt

        # if you do not have imu measurement, use last angular velocity command
        th_pre = self.x_t[2] + omega*dt

        # if you have imu measurement
        # th_pre = self.x_t[2] + imu_meas[3][0]*dt
        
        #print("v = ", v)
        #print("dt = ", dt)
        #print("x_pre = ", x_pre)
        #print("y_pre = ", y_pre)
        #print("th_pre = ", th_pre)
        
        dfdx = np.array([[1.0, 0, 0]]) # derivative of prediction model with respect to x position
        dfdy = np.array([[0, 1, 0]]) # derivative of prediction model with respect to y position
        dfdth = np.array([[-np.sin(self.x_t[2])*v*dt, np.cos(self.x_t[2])*v*dt, 1]]) # deriv. wrt theta
        dfdq = np.transpose(np.concatenate((dfdx, dfdy, dfdth), axis=0)) # Jacobian of prediction model wrt states
        #print("dfdx = ", dfdx)
        #print("dfdy = ", dfdy)
        #print("dfdth = ", dfdth)
        
        dfdnv = dt*np.array([[np.cos(self.x_t[2]), np.sin(self.x_t[2]), 0]]) # deriv. wrt velocity input uncertainty
        dfdnw = dt*np.array([[0, 0, 1]]) # deriv. wrt omega input uncertainty
        dfdn = np.transpose(np.concatenate((dfdnv, dfdnw), axis=0)) # Jacobian of prediction model wrt input uncertainty
        
        self.x_t = np.array([x_pre, y_pre, th_pre]) # predicted state
        #print("x_t (prediction) = ", self.x_t)
        #print("dfdq = ", dfdq)
        #print("dfdn = ", dfdn)
        #print("Q_t = ", self.Q_t)
        #print("dfdq.dot(self.P_t) = ", dfdq.dot(self.P_t))
        #print("dfdn.dot(self.Q_t) = ", dfdn.dot(self.Q_t))
        self.P_t = dfdq.dot(self.P_t).dot(dfdq.T) + dfdn.dot(self.Q_t).dot(dfdn.T) # predicted covariance
        
        return self.x_t, self.P_t

    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        
        dh = np.eye(3) # Jacobian of measurement model wrt states
        #print("P_t = ", self.P_t)
        #print("dh ", dh)
        #print("dh.T", dh.T)
        #print("R_t = ", self.R_t)
        #print("self.P_t.dot(dh.T)", self.P_t.dot(dh.T))
        #print("np.linalg.inv(dh.dot(self.P_t).dot(dh.T) + self.R_t)", np.linalg.inv(dh.dot(self.P_t).dot(dh.T) + self.R_t)) 
        #print(dh.dot(self.P_t).dot(dh.T) + self.R_t)
        K = self.P_t.dot(dh.T).dot(np.linalg.inv(dh.dot(self.P_t).dot(dh.T) + self.R_t))
        #print("K = ", K)
        
        for i in range(z_t.shape[0]):
        
            tagId = int(z_t[i][3])

            # Tag to robot frame homogeneous transformation matrix
            H_T2R = np.array([[np.cos(z_t[i][2]), -np.sin(z_t[i][2]), z_t[i][0]], \
            [np.sin(z_t[i][2]), np.cos(z_t[i][2]), z_t[i][1]], \
            [0, 0, 1]])
            #print("H_T2R.shape = " + str(H_T2R.shape))
            
            #print("z_t = ", z_t[i])
            #print("H_T2R = ", H_T2R)
            
            # Tag to world frame homogeneous transformation matrix
            #print("markers = " + str(self.markers))
            #print repr(self.markers)
            self.markers = list(np.array(self.markers, dtype=np.float32))
            H_T2W = np.array([[np.cos(self.markers[tagId][2]), -np.sin(self.markers[tagId][2]), self.markers[tagId][0]], \
            [np.sin(self.markers[tagId][2]), np.cos(self.markers[tagId][2]), self.markers[tagId][1]], \
            [0, 0, 1]])
            #print("H_T2W.shape = " + str(H_T2W.shape))
            
            #print("self.markers[i]", self.markers[i])
            #print("H_T2W = ", H_T2W)
            
            # Robot to world frame homogenous transformation matrix
            H_R2W = H_T2W.dot(np.linalg.inv(H_T2R))
            #print("H_R2W.shape = " + str(H_R2W.shape))
            #print("H_T2W = ", H_T2W)
            
            pose_meas = np.array([H_R2W[0,2], H_R2W[1,2], np.arctan2(H_R2W[1,0],H_R2W[0,0])])
            #print("pose_meas.shape = " + str(pose_meas.shape))
            #print("measured pose = ", pose_meas)
            
            self.x_t = self.x_t + K.dot(pose_meas-self.x_t)
            #print("x_t (update) = ", self.x_t)
        
        self.P_t = (np.eye(3) - K.dot(dh)).dot(self.P_t)
        
        return self.x_t, self.P_t
        
    def step_filter(self, v, omega, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, omega, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # YOUR CODE HERE
        # Prediction step
        print("x_t (pre-prediction) = ", self.x_t)
        #print("P_t (pre-prediction) = ", self.P_t)
        self.x_t, self.P_t = self.prediction(v, omega, imu_meas)
        print("x_t (post-prediction) = ", self.x_t)
        #print("P_t (post-prediction) = ", self.P_t)
        # Update step
        if z_t is not None and z_t != []:
            self.x_t, self.P_t = self.update(z_t)
            print("x_t (post-update) = ", self.x_t)
            #print("P_t (post-update) = ", self.P_t)
            
        return self.x_t
