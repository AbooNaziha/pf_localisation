from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from nav_msgs.msg import OccupancyGrid, Odometry##

from . util import rotateQuaternion, getHeading
import random
#from random import random

#from time import time
import time

#to graph data
import matplotlib.pyplot as plt


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        #filter parameters
        self.noise = 1 
        self. other_noise = 0.1
        self.Q =600

        self.ratio = 0.9 
        self.w_slow = 0
        self.w_fast = 0
        self.a_slow = 0.3 ### exponential smoothening according to PF textbook 8.3.7
        self.a_fast = 0.7

        self.NUMBER_PREDICTED_READINGS = 20 

        self._actual_pose = rospy.Subscriber("/base_pose_ground_truth", Odometry,self._actual_pose_callback,queue_size=1)
        self.xs = []#variables to store x,y and rads
        self.ys = []
        self.rads = []
        self.x_num = []
        self.xs_num =0


    def _actual_pose_callback(self, message):
        self.actual_pose = Pose()
        self.actual_pose = message.pose.pose
        self.actual_pose.position.x = self.actual_pose.position.x+15
        self.actual_pose.position.y = self.actual_pose.position.y+15

    def initialise_particle_cloud(self, initialpose):
        self.init_x = initialpose.pose.pose.position.x
        self.init_y = initialpose.pose.pose.position.y
        self.init_yaw = initialpose.pose.pose.orientation.z

        self.occupancy_map = rospy.wait_for_message("map", OccupancyGrid, 20)
        pose_array_object = PoseArray()
        for i in range(0,int(0.2*self.Q)):
           pose = Pose()
           pose.position.x = random.random()*self.occupancy_map.info.width*self.occupancy_map.info.resolution
           pose.position.y = random.random()*self.occupancy_map.info.height*self.occupancy_map.info.resolution
           pose.position.z = 0.0
           pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(0, math.pi))
           pose_array_object.poses.append(pose)
        #setting up poses for n particles
        for loop_variable in range(int(0.2*self.Q),self.Q):
           pose = Pose()
           pose.position.x = random.gauss(self.init_x, self.noise)
           pose.position.y = random.gauss(self.init_y, self.noise )
           pose.position.z = 0.0
           pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(0, math.pi))
           pose_array_object.poses.append(pose)


        return pose_array_object


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


        pass



    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        #time.sleep(2)
        self.weights_of_poses = [None]*self.Q
        self.previous_cloud = self.particlecloud
        self.roulett_weight = [0]*self.Q
        self.total_sum = 0;
        highest_weight = 0
        self.highest_weight_pos = 0
        w_avg=0
        for i in range(0,self.Q):
            #rospy.loginfo(i)
            weight = (self.sensor_model.get_weight(scan, self.particlecloud.poses[i]))
            w_avg = w_avg + ( 1/self.Q) * weight
            self.weights_of_poses[i] = weight
            if (weight > highest_weight):
                highest_weight = weight
                self.highest_weight_pos = i
            #for roulett wheel
            self.total_sum = self.total_sum + float(weight)
            self.roulett_weight[i] = self.total_sum
            #rospy.loginfo(self.roulett_weight)


        self.best_pose = Pose()
        self.best_pose = self.particlecloud.poses[self.highest_weight_pos]

        self.w_slow = self.w_slow + self.a_slow *(w_avg - self.w_slow)
        self.w_fast = self.w_fast + self.a_fast * (w_avg - self.w_fast)
        self.ratio = 1 -  max(0.0,1-(self.w_fast/self.w_slow))
        if self.ratio < 0.8 :  
            self.ratio = 0.8


        for i in range(int(self.ratio*self.Q),int(self.Q-1)):
           pose = Pose()
           pose.position.x = random.random()*self.occupancy_map.info.width*self.occupancy_map.info.resolution
           pose.position.y = random.random()*self.occupancy_map.info.height*self.occupancy_map.info.resolution
           pose.position.z = 0.0
           pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(0, math.pi))
           self.particlecloud.poses[i] = pose




        for p in range(0,int(self.ratio*self.Q)):
            rand_val = random.random() * self.total_sum
            #rospy.loginfo(rand_val)
            i = 0
            weight = self.roulett_weight[0]
            highest_weight = 0
            while(1):
                if (float(rand_val) < float(weight)):
                    break
                i =i+1
                weight = self.roulett_weight[i]

            #
            # rospy.loginfo("i:")
            # rospy.loginfo(i)
            #i = i+1
            pose = self.particlecloud.poses[p]
            pose.position.x= random.gauss(self.previous_cloud.poses[i].position.x, self.other_noise)
            pose.position.y= random.gauss(self.previous_cloud.poses[i].position.y, self.other_noise)
            _pose= Pose()
            pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(getHeading(self.previous_cloud.poses[i].orientation), math.pi/10))
            self.particlecloud.poses[p] = pose #sets particle cloud to that pose

        #rospy.loginfo(self.particlecloud.poses)
        #rospy.loginfo(str(self.init_x)+str(self.init_y))
        #pass

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

        _pose = Pose()
        sumy = sumqy = sumx = sumqx = sumqz = sumqw = 0
        for i in range(0,int(self.ratio*self.Q-1)):
            #sum  = sum + self.particlecloud.poses[i]
            weight_factor = (self.weights_of_poses[i]*1/self.total_sum)
            sumx = sumx + self.particlecloud.poses[i].position.x*weight_factor
            sumy = sumy + self.particlecloud.poses[i].position.y*weight_factor
            sumqx = sumqx + self.particlecloud.poses[i].orientation.x*weight_factor
            sumqy = sumqy + self.particlecloud.poses[i].orientation.y*weight_factor
            sumqz = sumqz + self.particlecloud.poses[i].orientation.z*weight_factor
            sumqw = sumqw + self.particlecloud.poses[i].orientation.w*weight_factor
            # sumcos = sumcos + math.degrees(math.cos(math.degrees(getHeading(self.particlecloud.poses[i].orientation))))#* (self.weights_of_poses[i]**1/3)
            # sumsin = sumsin + math.degrees(math.sin(math.degrees(getHeading(self.particlecloud.poses[i].orientation))))#* (self.weights_of_poses[i]**1/3)

        #get average pose
        _pose.position.x = sumx#/self.Q
        _pose.position.y = sumy#/self.Q
        _pose.orientation.x = sumqx#/self.Q
        _pose.orientation.y = sumqy#/self.Q
        _pose.orientation.z = sumqz#/self.Q
        _pose.orientation.w = sumqw#/self.Q


        _x_off= abs(1-self.actual_pose.position.x/(sumx))*100
        _y_off= abs(1-self.actual_pose.position.y/(sumy))*100
        _rad_off = abs(1-(getHeading(self.actual_pose.orientation)/getHeading(_pose.orientation)))*100


        rospy.loginfo("x percent off: " + str(_x_off) + "; y percent off: " + str(_y_off) + ";randian percent off: " + str(_rad_off))#displays percentage difference at each iteration


        return _pose
        #pass






