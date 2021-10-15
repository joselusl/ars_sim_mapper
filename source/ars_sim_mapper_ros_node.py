#!/usr/bin/env python

import numpy as np
from numpy import *

import os


import rospy


from ars_sim_mapper_ros import *




def main():

  ars_sim_mapper_ros = ArsSimMapperRos()

  ars_sim_mapper_ros.init()
  ars_sim_mapper_ros.open()

  try:
    ars_sim_mapper_ros.run()
  except rospy.ROSInterruptException:
    pass


  return 0



''' MAIN '''
if __name__ == '__main__':

  main()