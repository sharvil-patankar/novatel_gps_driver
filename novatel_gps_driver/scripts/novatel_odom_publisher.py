#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from novatel_gps_msgs.msg import NovatelUtmPosition, NovatelVelocity, Inscov
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_multiply, quaternion_conjugate
import message_filters


class NovatelOdomPublisher:
    def __init__(self):
        self.pub_odom = rospy.Publisher('novatel/odom', Odometry, queue_size=10)
        self.utm_sub = message_filters.Subscriber('novatel/bestutm', NovatelUtmPosition)
        self.inscov_sub = message_filters.Subscriber('novatel/inscov', Inscov)
        self.vel_sub = message_filters.Subscriber('novatel/bestvel', NovatelVelocity)
        self.imu_sub = message_filters.Subscriber('novatel/imu', Imu)

        self.odom = Odometry()
        self.odom.header.frame_id = 'utm'
        self.odom.child_frame_id = 'gps_imu'

    def sub_and_pub(self):
        self.ts = message_filters.ApproximateTimeSynchronizer([
            self.utm_sub, self.inscov_sub, self.vel_sub, self.imu_sub], 10, 0.01)
        self.ts.registerCallback(self.publish_odom)
        rospy.spin()

    def publish_odom(self, utm, inscov, vel, imu):
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = utm.easting
        self.odom.pose.pose.position.y = utm.northing
        self.odom.pose.pose.position.z = utm.height
        self.odom.pose.pose.orientation = imu.orientation

        self.position_covariance = np.diag([utm.easting_sigma**2, utm.northing_sigma**2, utm.height_sigma**2])
        self.odom.pose.covariance = ((np.block([
            [self.position_covariance, np.zeros([3, 3])                              ],
            [np.zeros([3, 3]),         np.reshape(imu.orientation_covariance, (3, 3))]
                                                ])).flatten()).tolist()
        self.odom.twist.twist.angular = imu.angular_velocity
        self.pure_v = [np.cos(np.radians(vel.track_ground)) * vel.horizontal_speed,
                       np.sin(np.radians(vel.track_ground)) * vel.horizontal_speed,
                       vel.vertical_speed,
                       0                                                      ]
        self.q = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        self.body_v = quaternion_multiply(quaternion_multiply(quaternion_conjugate(self.q), self.pur_v), self.q)
        self.odom.twist.twist.linear.x = self.body_v[0]
        self.odom.twist.twist.linear.y = self.body_v[1]
        self.odom.twist.twist.linear.z = self.body_v[2]
        self.odom.twist.covariance = ((np.block([
            [np.reshape(inscov.velocity_covariance, (3, 3)), np.zeros([3, 3])                                  ],
            [np.zeros([3, 3]),                               np.reshape(imu.angular_velocity_covariance, (3, 3))]
                                                ])).flatten()).tolist()
        self.pub_odom.publish(self.odom)


if __name__ == '__main__':
    rospy.init_node('novatel_odom_publisher', anonymous=True)
    try:
        novatel_odom_publisher_class = NovatelOdomPublisher()
        novatel_odom_publisher_class.sub_and_pub()
    except rospy.ROSInterruptException:
        pass
