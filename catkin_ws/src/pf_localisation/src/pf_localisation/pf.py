from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from . pf_base import PFLocaliserBase
import math
import rospy
import numpy as np

from . util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 200     # Number of readings to predict
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise
        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        self.ODOM_ROTATION_NOISE = 2  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.2   # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 2  # Odometry y axis (side-side) noise
        current_x = initialpose.pose.pose.position.x
        current_y = initialpose.pose.pose.position.y
        current_z = initialpose.pose.pose.position.z
        current_o = initialpose.pose.pose.orientation
        current_angle = getHeading(current_o)
        current = PoseArray()
        particle_lists = {}
        for i in range(self.NUMBER_PREDICTED_READINGS):
            new_x = np.random.normal(0, 0.1, 1) * self.ODOM_TRANSLATION_NOISE + current_x
            new_x = new_x[0]
            #print(new_x)
            new_y = np.random.normal(0, 0.1, 1) * self.ODOM_DRIFT_NOISE + current_y
            new_y = new_y[0]
            rand_angle = np.random.normal(0, 0.1, 1) * self.ODOM_ROTATION_NOISE 
            new_o = rotateQuaternion(current_o, rand_angle)
            estimatedpose = PoseWithCovarianceStamped()
            estimatedpose.pose.pose.position.x = new_x
            estimatedpose.pose.pose.position.y = new_y
            estimatedpose.pose.pose.position.z = current_z
            estimatedpose.pose.pose.orientation = new_o
            p = estimatedpose.pose.pose
            #print(p)
            current.poses.append(Pose(p.position,p.orientation))
        return current

 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
         """
        a = self.sensor_model
        i = 0
        likelihood = []
        sum = 0
        for p in self.particlecloud.poses:
        	#rint(p.position.x)
        	#print(p.position.y)
        	res = a.get_weight(scan,p)
        	#print(res)
        	sum = sum + res
        	likelihood.append(res)
        	i = i + 1
        	#rint(likelihood[i])
        #likelihood = likelihood / sum
        for x in range(self.NUMBER_PREDICTED_READINGS):
        	likelihood[x] = likelihood[x] / sum
        	#print(likelihood[x])
        
        #resampling	
        current = PoseArray()
        m = 1 / self.NUMBER_PREDICTED_READINGS
        r = np.random.uniform(0,1/self.NUMBER_PREDICTED_READINGS)
        order = 0
        c = 0
        u = 0
        likelihood.sort()
        for x in range(self.NUMBER_PREDICTED_READINGS):
            c = c + likelihood[x]
            u = r + x * m
            while u > c:
        	    order = order + 1
            current.poses.append(self.particlecloud.poses[order])
            print(order)
        	#print(self.particlecloud.poses[order])
        self.particlecloud = current

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers
        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        totalx = 0
        totaly = 0
        suma = 0
        totalz = 0
        for i in range(self.NUMBER_PREDICTED_READINGS):
        	totalx = totalx + self.particlecloud.poses[i].position.x
        	totaly = totaly + self.particlecloud.poses[i].position.y
        	totalz = totalz + self.particlecloud.poses[i].position.z
        	suma = suma + getHeading(self.particlecloud.poses[i].orientation)
        totalx = totalx / self.NUMBER_PREDICTED_READINGS
        totaly = totaly / self.NUMBER_PREDICTED_READINGS
        suma = suma / self.NUMBER_PREDICTED_READINGS
        totalz = totalz / self.NUMBER_PREDICTED_READINGS
        res =  PoseWithCovarianceStamped()
        res.pose.pose.position.x = totalx
        res.pose.pose.position.y = totaly
        res.pose.pose.position.z = totalz
        res.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),
                                                                    suma)
        p = res.pose.pose
            #print(p)
        print(p)
        return Pose(p.position,p.orientation)