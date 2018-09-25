#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
#from MyShortestPath import my_dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map, pos_init.flatten())
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.v_last = 0.0
        self.omega_last = 0.0
        self.goal = pos_goal

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        This function is called at 60Hz
        """
        # tag wrt robot in robot frame - (Nx4) np array (x,y,theta,id)
        meas = np.array(self.ros_interface.get_measurements())
        # (5,1) np array (xacc,yacc,zacc,omega,time)
        imu_meas = self.ros_interface.get_imu()
        print "meas = ", meas
        #print("type(meas) = " + str(type(meas)))
        print "imu_meas = ", imu_meas

        #done = False
        #print("time = " + str(self.robot_sim.last_meas_time))
        #if meas is not None and meas != []:
            #z_t = meas[:,:-1] # if meas is a np array, then access elements with tuple

        #else:
            #z_t = meas
        print "v_last (process_measurements) = ", self.v_last
        print "omega_last (process_measurements) = ", self.omega_last

        state = self.kalman_filter.step_filter(self.v_last, self.omega_last, imu_meas, np.array(meas))
        print "state = ", state
        print "goal = ", self.goal
        
        done = False
        if meas is not None:
            v, omega, done = self.diff_drive_controller.compute_vel(state, self.goal)
            print "v (compute_vel) = ", v
            print "omega (compute_vel) = ", omega
            print "done (compute_vel) = ", done
        self.ros_interface.command_velocity(v, omega)
        self.v_last = v
        self.omega_last = omega

        #print "v (compute_vel) = ", v
        #print "omega (compute_vel) = ", omega
        
        return done
    
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(1)
    done = False
    #time = rospy.get_time() # used for calibration
    while not rospy.is_shutdown() and not done:
        done = robotControl.process_measurements()
	# Calibration
	# command robot forward at 0.3 m/s for 1 sec and measure how far it travels
	#if rospy.get_time() - time < 1:
	   #robotControl.ros_interface.command_velocity(0.3,0)
	#else:
	    #robotControl.ros_interface.command_velocity(0,0)
        r.sleep()

    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


