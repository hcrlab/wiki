#!/usr/bin/env python

from __future__ import division
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def main():
    rospy.init_node('sub_pcl');
    processor = Processor(rospy.Time.now())
    rospy.Subscriber('/pointcloud2_in', PointCloud2, processor.callback,
        queue_size=1)
    rospy.spin()

class Processor(object):
    """Computes and logs the time taken to process a point cloud."""

    def __init__(self, prev_time):
        self._prev_time = prev_time
        self._time_taken = rospy.Duration(0)
        self._num_calls = 0

    def callback(self, data):
        """Computes the average point in a point cloud and saves timing info.
        """
        points = pc2.read_points(data, field_names=['x', 'y', 'z'],
            skip_nans=True)
    
        num_points = 0
        avg_x = 0
        avg_y = 0
        avg_z = 0
        for x, y, z in points:
            num_points += 1
            avg_x += x
            avg_y += y
            avg_z += z
        if num_points > 0:
            avg_x /= num_points
            avg_y /= num_points
            avg_z /= num_points

        current_time = rospy.Time.now()
        self._time_taken += current_time - self._prev_time
        self._prev_time = current_time
        self._num_calls += 1

        rospy.loginfo('x: {}, y: {}, z: {}, time/point: {}s'.format(
            avg_x, avg_y, avg_z, self._time_taken.to_sec() / self._num_calls))


if __name__ == '__main__':
    main()
