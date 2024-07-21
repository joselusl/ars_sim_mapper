#!/usr/bin/env python3

import rclpy

from ars_sim_mapper.ars_sim_mapper_ros import ArsSimMapperRos


def main(args=None):

  rclpy.init(args=args)

  ars_sim_mapper_ros = ArsSimMapperRos()

  ars_sim_mapper_ros.open()

  try:
      ars_sim_mapper_ros.run()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      # Graceful shutdown on interruption
      pass
  finally:
    ars_sim_mapper_ros.destroy_node()
    rclpy.try_shutdown()

  return 0


''' MAIN '''
if __name__ == '__main__':

  main()