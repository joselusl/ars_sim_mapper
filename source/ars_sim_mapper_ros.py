#!/usr/bin/env python

import numpy as np
from numpy import *

import os

import copy




# ROS

import rospy

import rospkg

import std_msgs.msg
from std_msgs.msg import Header


import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import visualization_msgs.msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray



import tf_conversions

import tf2_ros




#
import ars_lib_helpers





class ArsSimMapperRos:

  #######

  # Robot frame
  world_frame = None

  # Covariance on mapping of position
  cov_map_stat_pos = None
  cov_map_dyna_pos = None

  # Covariance on mapping of sizes
  cov_map_stat_siz = None
  cov_map_dyna_siz = None

  # Robot pose subscriber
  robot_pose_sub = None

  # Obstacles static sub
  obstacles_static_sub = None
  
  # Obstacles dynamic sub
  obstacles_dynamic_sub = None

  # Obstacles detected pub
  flag_pub_obstacles_detected_world = False
  obstacles_detected_world_pub = None

  # Obstacles static
  obstacles_static_msg = None

  # Obstacles dynamic
  obstacles_dynamic_msg = None

  # Obstacles detected
  obstacles_detected_world_msg = None

  # Obstacle Detection loop
  # freq
  obstacle_detect_loop_freq = None
  # Timer
  obstacle_detect_loop_timer = None
  


  #########

  def __init__(self):

    # Robot frame
    self.world_frame = 'world'


    # Covariance on mapping of position
    self.cov_map_stat_pos = {'x': 0.0001, 'y': 0.0001, 'z': 0.000001}
    self.cov_map_dyna_pos = {'x': 0.01, 'y': 0.01, 'z': 0.00001}

    # Covariance on mapping of sizes
    self.cov_map_stat_siz = {'R': 0.0001, 'h': 0.000001}
    self.cov_map_dyna_siz = {'R': 0.01, 'h': 0.0001}

    #
    self.obstacles_static_msg = MarkerArray()

    #
    self.obstacles_dynamic_msg = MarkerArray()

    #
    self.obstacles_detected_world_msg = MarkerArray()

    # Obstacle Detection loop
    # freq
    self.obstacle_detect_loop_freq = 0.1
    # Timer
    self.obstacle_detect_loop_timer = None


    # end
    return


  def init(self, node_name='ars_sim_mapper_node'):
    #

    # Init ROS
    rospy.init_node(node_name, anonymous=True)

    
    # Package path
    pkg_path = rospkg.RosPack().get_path('ars_sim_mapper')
    

    #### READING PARAMETERS ###
    
    # TODO

    ###


    
    # End
    return


  def open(self):


    # Subscribers
    
    # 
    self.obstacles_static_sub = rospy.Subscriber('obstacles_static', MarkerArray, self.obstaclesStaticCallback, queue_size=1)
    #
    self.obstacles_dynamic_sub = rospy.Subscriber('obstacles_dynamic', MarkerArray, self.obstaclesDynamicCallback, queue_size=1)


    # Publishers

    # 
    self.obstacles_detected_world_pub = rospy.Publisher('estim_map_world', MarkerArray, queue_size=1, latch=True)



    # Timers
    #
    self.obstacle_detect_loop_timer = rospy.Timer(rospy.Duration(1.0/self.obstacle_detect_loop_freq), self.obstacleDetectorLoopTimerCallback)


    # End
    return


  def run(self):

    rospy.spin()

    return


  def obstaclesStaticCallback(self, obstacles_static_msg):

    self.obstacles_static_msg = obstacles_static_msg

    #
    return


  def obstaclesDynamicCallback(self, obstacles_dynamic_msg):

    self.obstacles_dynamic_msg = obstacles_dynamic_msg

    #
    return


  def detectObstacles(self):

    #
    self.obstacles_detected_world_msg = MarkerArray()
    self.obstacles_detected_world_msg.markers = []


    # Obstacles static
    for obst_i_msg in self.obstacles_static_msg.markers:

      if(obst_i_msg.action == 0):

        # Check distance
        if(obst_i_msg.type == 3):

          obst_i_posi_world = np.zeros((3,), dtype=float)
          obst_i_posi_world[0] = obst_i_msg.pose.position.x
          obst_i_posi_world[1] = obst_i_msg.pose.position.y
          obst_i_posi_world[2] = obst_i_msg.pose.position.z

          obst_i_rad = obst_i_msg.scale.x/2.0

            
          # Noises
          #
          posi_noise = np.zeros((3,), dtype=float)
          posi_noise[0] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_stat_pos['x']))
          posi_noise[1] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_stat_pos['y']))
          posi_noise[2] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_stat_pos['z']))
          #
          radius_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_stat_siz['R']))
          height_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_stat_siz['h']))
      


          ############
          # obstacle wrt World 
          obst_i_world_msg = []
          obst_i_world_msg = copy.deepcopy(obst_i_msg)

          # Change color
          obst_i_world_msg.color.r = 0.0
          obst_i_world_msg.color.g = 0.0
          obst_i_world_msg.color.b = 1.0
          obst_i_world_msg.color.a = 0.6

          # Lifetime
          obst_i_world_msg.lifetime = rospy.Duration(2.0*1.0/self.obstacle_detect_loop_freq)

          #
          obst_i_world_msg.pose.position.x = obst_i_posi_world[0] + posi_noise[0]
          obst_i_world_msg.pose.position.y = obst_i_posi_world[1] + posi_noise[1]
          obst_i_world_msg.pose.position.z = obst_i_posi_world[2] + posi_noise[2]

          # Sizes with noise
          obst_i_world_msg.scale.x += 2.0*radius_noise
          obst_i_world_msg.scale.y += 2.0*radius_noise
          obst_i_world_msg.scale.z += height_noise


          # Append world
          self.obstacles_detected_world_msg.markers.append(obst_i_world_msg)


        else:
          print("Unknown obstacle type:"+obst_i_msg.type)



    # Obstacles dynamic
    for obst_i_msg in self.obstacles_dynamic_msg.markers:

      if(obst_i_msg.action == 0):

        # Check distance
        if(obst_i_msg.type == 3):

          obst_i_posi_world = np.zeros((3,), dtype=float)
          obst_i_posi_world[0] = obst_i_msg.pose.position.x
          obst_i_posi_world[1] = obst_i_msg.pose.position.y
          obst_i_posi_world[2] = obst_i_msg.pose.position.z

          obst_i_rad = obst_i_msg.scale.x/2.0

          
          # Noises
          #
          posi_noise = np.zeros((3,), dtype=float)
          posi_noise[0] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_dyna_pos['x']))
          posi_noise[1] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_dyna_pos['y']))
          posi_noise[2] = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_dyna_pos['z']))
          #
          radius_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_dyna_siz['R']))
          height_noise = np.random.normal(loc = 0.0, scale = math.sqrt(self.cov_map_dyna_siz['h']))
      

          ############
          # obstacle wrt World 
          obst_i_world_msg = []
          obst_i_world_msg = copy.deepcopy(obst_i_msg)

          # Change color
          obst_i_world_msg.color.r = 0.0
          obst_i_world_msg.color.g = 0.0
          obst_i_world_msg.color.b = 1.0
          obst_i_world_msg.color.a = 0.6

          # Lifetime
          obst_i_world_msg.lifetime = rospy.Duration(2.0*1.0/self.obstacle_detect_loop_freq)

          #
          obst_i_world_msg.pose.position.x = obst_i_posi_world[0] + posi_noise[0]
          obst_i_world_msg.pose.position.y = obst_i_posi_world[1] + posi_noise[1]
          obst_i_world_msg.pose.position.z = obst_i_posi_world[2] + posi_noise[2]

          # Sizes with noise
          obst_i_world_msg.scale.x += 2.0*radius_noise
          obst_i_world_msg.scale.y += 2.0*radius_noise
          obst_i_world_msg.scale.z += height_noise


            

          # Append world
          self.obstacles_detected_world_msg.markers.append(obst_i_world_msg)

            

        else:
          print("Unknown obstacle type!!")


    # Publish
    self.obstacles_detected_world_pub.publish(self.obstacles_detected_world_msg)

    #
    return


  def obstacleDetectorLoopTimerCallback(self, timer_msg):

    # Get time
    time_stamp_current = rospy.Time.now()

    #
    self.detectObstacles()

    #
    return
