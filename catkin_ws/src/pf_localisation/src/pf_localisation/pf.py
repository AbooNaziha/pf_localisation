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
        self.noise = 1 # noise of starting noise in meters
        self. other_noise = 0.1# noise of resampling
        self.M =500#number of particles

        self.ratio = 0.9 # ratio of random particles to particles around resaampled particles
        self.w_slow = 0
        self.w_fast = 0
        self.a_slow = 0.3 ### exponential smoothening according to PF textbook 8.3.7
        self.a_fast = 0.7

        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

        self._actual_pose = rospy.Subscriber("/base_pose_ground_truth", Odometry,self._actual_pose_callback,queue_size=1)#sets up subsciber for actual pose
        self.xs = []#variables to store x,y and rads
        self.ys = []
        self.rads = []
        self.x_num = []
        self.xs_num =0#stores number of iterations


    def _actual_pose_callback(self, message):#gets actual pose to be compared with
        self.actual_pose = Pose()
        self.actual_pose = message.pose.pose
        self.actual_pose.position.x = self.actual_pose.position.x+15
        self.actual_pose.position.y = self.actual_pose.position.y+15

    def initialise_particle_cloud(self, initialpose):
        self.init_x = initialpose.pose.pose.position.x#gets initial position
        self.init_y = initialpose.pose.pose.position.y
        self.init_yaw = initialpose.pose.pose.orientation.z

        self.occupancy_map = rospy.wait_for_message("map", OccupancyGrid, 20)##occupancy map to generate random variables
        pose_array_object = PoseArray()#to store poses to be sent as particle array
        for i in range(0,int(0.2*self.M)):#random particles for kidnapped robot problem
           pose = Pose()#store pose of random paarticle
           pose.position.x = random.random()*self.occupancy_map.info.width*self.occupancy_map.info.resolution#generate each value of particle
           pose.position.y = random.random()*self.occupancy_map.info.height*self.occupancy_map.info.resolution
           pose.position.z = 0.0
           pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(0, math.pi))
           pose_array_object.poses.append(pose)#add poses to object
        #setting up poses for n particles
        for loop_variable in range(int(0.2*self.M),self.M):
           pose = Pose()#store pose of particle around initial pose
           pose.position.x = random.gauss(self.init_x, self.noise)#generate each value of particle with a normal distribution of mean being the initial pose
           pose.position.y = random.gauss(self.init_y, self.noise )
           pose.position.z = 0.0
           pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(0, math.pi))
           pose_array_object.poses.append(pose)#add pose to particle cloud


        return pose_array_object#return particle cloud


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
        self.weights_of_poses = [None]*self.M#create array to store particles
        self.previous_cloud = self.particlecloud#holt previous particle cloud
        self.roulett_weight = [0]*self.M#create array to hold roulette weights
        self.total_sum = 0;#sum of all the weights
        highest_weight = 0#store highest current wieght
        self.highest_weight_pos = 0#store postion of highest weight
        w_avg=0
        for i in range(0,self.M):#loop for number of particles, generates roulett wheel weights
            #rospy.loginfo(i)
            weight = (self.sensor_model.get_weight(scan, self.particlecloud.poses[i]))#get weight of particle
            w_avg = w_avg + ( 1/self.M) * weight
            self.weights_of_poses[i] = weight#add array of weights
            if (weight > highest_weight):#get position of highest weight
                highest_weight = weight
                self.highest_weight_pos = i
            #for roulett wheel
            self.total_sum = self.total_sum + float(weight)#update current total of the weights
            self.roulett_weight[i] = self.total_sum#update the roulet wheel weights
            #rospy.loginfo(self.roulett_weight)


        self.best_pose = Pose()#to store most likely pose
        self.best_pose = self.particlecloud.poses[self.highest_weight_pos]#gets most likely pose from previous particle cloud

        self.w_slow = self.w_slow + self.a_slow *(w_avg - self.w_slow)
        self.w_fast = self.w_fast + self.a_fast * (w_avg - self.w_fast)
        self.ratio = 1 -  max(0.0,1-(self.w_fast/self.w_slow))
        if self.ratio < 0.8 :  #threshold check...of 20% random particles
            self.ratio = 0.8


        for i in range(int(self.ratio*self.M),int(self.M-1)):#to generate random particles
           pose = Pose()
           pose.position.x = random.random()*self.occupancy_map.info.width*self.occupancy_map.info.resolution
           pose.position.y = random.random()*self.occupancy_map.info.height*self.occupancy_map.info.resolution
           pose.position.z = 0.0
           pose.orientation= rotateQuaternion(Quaternion(w=1.0),random.gauss(0, math.pi))
           self.particlecloud.poses[i] = pose




        for p in range(0,int(self.ratio*self.M)):#to generate particle rasamppled from previous particles
            rand_val = random.random() * self.total_sum#gets random number within max total roulette wheel weight
            #rospy.loginfo(rand_val)
            i = 0
            weight = self.roulett_weight[0]
            highest_weight = 0
            while(1):#gets psotion where the random value lies
                if (float(rand_val) < float(weight)):
                    break
                i =i+1
                weight = self.roulett_weight[i]

            #
            # rospy.loginfo("i:")
            # rospy.loginfo(i)
            #i = i+1
            pose = self.particlecloud.poses[p]
            pose.position.x= random.gauss(self.previous_cloud.poses[i].position.x, self.other_noise)#create particle around the previous particle
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
        sumy = sumqy = sumx = sumqx = sumqz = sumqw = 0#initialise variables
        for i in range(0,int(self.ratio*self.M-1)):#repeats for particles resampled around estimated pose
            #sum  = sum + self.particlecloud.poses[i]
            weight_factor = (self.weights_of_poses[i]*1/self.total_sum)
            sumx = sumx + self.particlecloud.poses[i].position.x*weight_factor#adds to average the product of the distance and the normalised weight
            sumy = sumy + self.particlecloud.poses[i].position.y*weight_factor
            sumqx = sumqx + self.particlecloud.poses[i].orientation.x*weight_factor
            sumqy = sumqy + self.particlecloud.poses[i].orientation.y*weight_factor
            sumqz = sumqz + self.particlecloud.poses[i].orientation.z*weight_factor
            sumqw = sumqw + self.particlecloud.poses[i].orientation.w*weight_factor
            # sumcos = sumcos + math.degrees(math.cos(math.degrees(getHeading(self.particlecloud.poses[i].orientation))))#* (self.weights_of_poses[i]**1/3)
            # sumsin = sumsin + math.degrees(math.sin(math.degrees(getHeading(self.particlecloud.poses[i].orientation))))#* (self.weights_of_poses[i]**1/3)

        #get average pose
        _pose.position.x = sumx#/self.M
        _pose.position.y = sumy#/self.M
        _pose.orientation.x = sumqx#/self.M
        _pose.orientation.y = sumqy#/self.M
        _pose.orientation.z = sumqz#/self.M
        _pose.orientation.w = sumqw#/self.M


        _x_off= abs(1-self.actual_pose.position.x/(sumx))*100#gets percentage different for x,y and orientation
        _y_off= abs(1-self.actual_pose.position.y/(sumy))*100
        _rad_off = abs(1-(getHeading(self.actual_pose.orientation)/getHeading(_pose.orientation)))*100

        # if(self.xs_num == 150):# for kidnapped robot problem
        #     time.sleep(15)
        #     self.xs_num = self.xs_num +1

        #comment out if graph not wanted
        # if(self.xs_num < 350): #adds the data to graoh
        #     self.xs_num = self.xs_num +1
        #     self.x_num.append(self.xs_num)
        #     self.xs.append(_y_off)
        #     self.ys.append(_x_off)
        #     self.rads.append(_rad_off)
        #     rospy.loginfo(self.xs_num)
        #
        # elif(self.xs_num > 150):# displays graphs
        #
        #     rospy.loginfo("plotting")
        #     plt.plot(self.x_num,self.xs, label = "x percent off")
        #     plt.plot(self.x_num,self.ys, label = "y percent off")
        #     plt.ylim(0,100)
        #     plt.show()
        #     plt.plot(self.x_num,self.rads, label = "rad percent off")
        #     plt.ylim(0,100)
        #     plt.show()

        rospy.loginfo("x percent off: " + str(_x_off) + "; y percent off: " + str(_y_off) + ";randian percent off: " + str(_rad_off))#displays percentage difference at each iteration


        return _pose
        #pass






# from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
# from . pf_base import PFLocaliserBase
# import math
# import rospy
# import numpy as np

# from . util import rotateQuaternion, getHeading
# from random import random

# from time import time


# class PFLocaliser(PFLocaliserBase):
       
#     def __init__(self):
#         # ----- Call the superclass constructor
#         super(PFLocaliser, self).__init__()
        
#         # ----- Set motion model parameters
 
#         # ----- Sensor model parameters
#         self.NUMBER_PREDICTED_READINGS = 200     # Number of readings to predict
        
       
#     def initialise_particle_cloud(self, initialpose):
#         """
#         Set particle cloud to initialpose plus noise
#         Called whenever an initialpose message is received (to change the
#         starting location of the robot), or a new occupancy_map is received.
#         self.particlecloud can be initialised here. Initial pose of the robot
#         is also set here.
        
#         :Args:
#             | initialpose: the initial pose estimate
#         :Return:
#             | (geometry_msgs.msg.PoseArray) poses of the particles
#         """
#         self.ODOM_ROTATION_NOISE = 2  # Odometry model rotation noise
#         self.ODOM_TRANSLATION_NOISE = 0.2   # Odometry x axis (forward) noise
#         self.ODOM_DRIFT_NOISE = 2  # Odometry y axis (side-side) noise
#         current_x = initialpose.pose.pose.position.x
#         current_y = initialpose.pose.pose.position.y
#         current_z = initialpose.pose.pose.position.z
#         current_o = initialpose.pose.pose.orientation
#         current_angle = getHeading(current_o)
#         current = PoseArray()
#         particle_lists = {}
#         for i in range(self.NUMBER_PREDICTED_READINGS):
#             new_x = np.random.normal(0, 0.1, 1) * self.ODOM_TRANSLATION_NOISE + current_x
#             new_x = new_x[0]
#             #print(new_x)
#             new_y = np.random.normal(0, 0.1, 1) * self.ODOM_DRIFT_NOISE + current_y
#             new_y = new_y[0]
#             rand_angle = np.random.normal(0, 0.1, 1) * self.ODOM_ROTATION_NOISE 
#             new_o = rotateQuaternion(current_o, rand_angle)
#             estimatedpose = PoseWithCovarianceStamped()
#             estimatedpose.pose.pose.position.x = new_x
#             estimatedpose.pose.pose.position.y = new_y
#             estimatedpose.pose.pose.position.z = current_z
#             estimatedpose.pose.pose.orientation = new_o
#             p = estimatedpose.pose.pose
#             #print(p)
#             current.poses.append(Pose(p.position,p.orientation))
#         return current

 
    
#     def update_particle_cloud(self, scan):
#         """
#         This should use the supplied laser scan to update the current
#         particle cloud. i.e. self.particlecloud should be updated.
        
#         :Args:
#             | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
#          """
#         a = self.sensor_model
#         i = 0
#         likelihood = []
#         sum = 0
#         for p in self.particlecloud.poses:
#         	#rint(p.position.x)
#         	#print(p.position.y)
#         	res = a.get_weight(scan,p)
#         	#print(res)
#         	sum = sum + res
#         	likelihood.append(res)
#         	i = i + 1
#         	#rint(likelihood[i])
#         #likelihood = likelihood / sum
#         for x in range(self.NUMBER_PREDICTED_READINGS):
#         	likelihood[x] = likelihood[x] / sum
#         	#print(likelihood[x])
        
#         #resampling	
#         current = PoseArray()
#         m = 1 / self.NUMBER_PREDICTED_READINGS
#         r = np.random.uniform(0,1/self.NUMBER_PREDICTED_READINGS)
#         order = 0
#         c = 0
#         u = 0
#         likelihood.sort()
#         for x in range(self.NUMBER_PREDICTED_READINGS):
#             c = c + likelihood[x]
#             u = r + x * m
#             while u > c:
#         	    order = order + 1
#             current.poses.append(self.particlecloud.poses[order])
#             print(order)
#         	#print(self.particlecloud.poses[order])
#         self.particlecloud = current

#     def estimate_pose(self):
#         """
#         This should calculate and return an updated robot pose estimate based
#         on the particle cloud (self.particlecloud).
        
#         Create new estimated pose, given particle cloud
#         E.g. just average the location and orientation values of each of
#         the particles and return this.
        
#         Better approximations could be made by doing some simple clustering,
#         e.g. taking the average location of half the particles after 
#         throwing away any which are outliers
#         :Return:
#             | (geometry_msgs.msg.Pose) robot's estimated pose.
#          """
#         totalx = 0
#         totaly = 0
#         suma = 0
#         totalz = 0
#         for i in range(self.NUMBER_PREDICTED_READINGS):
#         	totalx = totalx + self.particlecloud.poses[i].position.x
#         	totaly = totaly + self.particlecloud.poses[i].position.y
#         	totalz = totalz + self.particlecloud.poses[i].position.z
#         	suma = suma + getHeading(self.particlecloud.poses[i].orientation)
#         totalx = totalx / self.NUMBER_PREDICTED_READINGS
#         totaly = totaly / self.NUMBER_PREDICTED_READINGS
#         suma = suma / self.NUMBER_PREDICTED_READINGS
#         totalz = totalz / self.NUMBER_PREDICTED_READINGS
#         res =  PoseWithCovarianceStamped()
#         res.pose.pose.position.x = totalx
#         res.pose.pose.position.y = totaly
#         res.pose.pose.position.z = totalz
#         res.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),
#                                                                     suma)
#         p = res.pose.pose
#             #print(p)
#         print(p)
#         return Pose(p.position,p.orientation)