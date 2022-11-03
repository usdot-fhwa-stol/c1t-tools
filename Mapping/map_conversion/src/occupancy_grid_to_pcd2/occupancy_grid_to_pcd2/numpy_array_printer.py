#!/usr/bin/env python3

import time, os, sys
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2 as pc2
import numpy as np
import ros2_numpy as rnp

class occ_grid_to_numpy(Node):

    def __init__(self):
        super().__init__('occ_grid_printer')
        self.topic_prefix = 'numpy_map/'
        self.occ_subscriber = self.create_subscription(OccupancyGrid, 'map', self.occ_grid_callback, 10)
        # self.pcd2_subscriber = self.create_subscription(pc2, 'laser_pcd2', self.pcd2_callback,10)
        self.pcd2_publisher = self.create_publisher(pc2,self.topic_prefix + 'map',10)
        self.get_logger().info("initializing")

    def pcd2_callback(self,msg):
        data = rnp.numpify(msg)
        a = np.asarray(data)
        np.savetxt(str(int(round(time.time(),2)*100)),data,delimiter=",")
        new_pcd2 = rnp.msgify(pc2, data)
        self.pcd2_publisher.publish(new_pcd2)


    def occ_grid_callback(self, msg):
        data = rnp.numpify(msg)
        a = np.asarray(data)
        # new_data = rnp.msgify(PointCloud2,data)
        self.get_logger().info('got map')
        np.savetxt(str(int(round(time.time(),2)*100)),data,delimiter=",")

    def occupancy_grid_numpy_to_pcd_numpy(self,data):
        
        return data


def main(args=None):
    rclpy.init(args=args)
    csv_maker = occ_grid_to_numpy()
    rclpy.spin(csv_maker)

    csv_maker.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()