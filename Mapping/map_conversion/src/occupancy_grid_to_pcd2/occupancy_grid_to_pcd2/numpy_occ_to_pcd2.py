#!/usr/bin/env python3
#
# Written by the USDOT Volpe National Transportation Systems Center
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import time
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros2_numpy as rnp

class occ_grid_to_numpy(Node):

    def __init__(self):
        super().__init__('occ_grid_printer')
        self.topic_prefix = 'numpy_map/'
        self.occ_subscriber = self.create_subscription(OccupancyGrid, '/map', self.occ_grid_callback, 10)
        # self.pcd2_subscriber = self.create_subscription(pc2, 'laser_pcd2', self.pcd2_callback,10)
        self.pcd2_publisher = self.create_publisher(PointCloud2,self.topic_prefix + 'pointcloud',10)
        self.get_logger().info("initializing")
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.05

    def pcd2_callback(self,msg):
        data = rnp.numpify(msg)
        a = np.asarray(data)
        np.savetxt(str(int(round(time.time(),2)*100)),data,delimiter=",")
        self.get_logger().info("file made")
        new_pcd2 = rnp.msgify(PointCloud2, data)
        self.pcd2_publisher.publish(new_pcd2)

    def occ_grid_callback(self, msg):
        data = rnp.numpify(msg)
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.get_logger().info('got map')
        self.occupancy_grid_numpy_to_pcd_numpy_(data)

    def origin_shifter_(self):
        pass

    def occupancy_grid_numpy_to_pcd_numpy_(self,data):
        r,c = data.shape[0],data.shape[1]

        x0,y0 = self.origin_x, self.origin_y
        i_count,j_count = 0,0
        resolution = self.resolution
        # get number of points in the occupancy grid so an array can be made
        num_points = 0
        for j in reversed(range(0,r)):
            for i in range(0,c):
                if data[j,i]>0:
                    num_points += 1
                i_count += 1
            j_count += 1
            i_count = 0
        # create array of size num_points
        points_arr = np.zeros((num_points,), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('r', np.uint8),
            ('g', np.uint8),
            ('b', np.uint8)
        ])
        # convert filled grid locations from occupancy grid array into
        # an array formatted for pointcloud2
        i_point = 0
        for j in reversed(range(0,r)):
            for i in range(0,c):
                if data[j,i]>0:
                    x = x0 + resolution*i_count
                    y = y0 + resolution*j_count
                    z = 0.0
                    red = 230
                    green = 150
                    blue = int(max(100-data[j,i],0))
                    points_arr[i_point]=(x,y,z,red,green,blue)
                    i_point += 1
                i_count += 1
            j_count += 1
            i_count = 0
        
        # create message and publish
        new_msg = PointCloud2()
        new_msg = rnp.msgify(PointCloud2,points_arr)
        # np.savetxt("pcd2" + str(int(round(time.time(),2)*100)),new_data,delimiter=",")
        self.pcd2_publisher.publish(new_msg)
        self.get_logger().info('Created and Published PointCloud2 based on OccupancyGrid')



def main(args=None):
    rclpy.init(args=args)
    csv_maker = occ_grid_to_numpy()
    rclpy.spin(csv_maker)

    csv_maker.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()