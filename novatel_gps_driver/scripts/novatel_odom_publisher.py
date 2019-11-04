#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
# from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
# import tf2_geometry_msgs.tf2_geometry_msgs as tf2_gm
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Point
from nav_msgs.msg import Odometry
from novatel_gps_msgs.msg import NovatelUtmPosition, NovatelVelocity, Insstdev
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_multiply, quaternion_conjugate
import message_filters


class NovatelOdomPublisher:
    def __init__(self):
        self.rate = rospy.Rate(30)
        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
	self.utm_sub = message_filters.Subscriber('bestutm', NovatelUtmPosition)
        self.insstdev_sub = message_filters.Subscriber('insstdev', Insstdev)
        self.vel_sub = message_filters.Subscriber('bestvel', NovatelVelocity)
        self.imu_sub = message_filters.Subscriber('imu', Imu)

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.buff = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buff)
        self.lis.unregister()
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_link'

    def sub_and_pub(self, origin):
        self.origin = Point()
        self.origin.x = origin['x']
        self.origin.y = origin['y']
        self.origin.z = origin['z']
        self.ts = message_filters.ApproximateTimeSynchronizer([
            self.utm_sub, self.insstdev_sub, self.vel_sub, self.imu_sub], 10, 0.01)
        self.ts.registerCallback(self.publish_odom)
        rospy.spin()

    def publish_odom(self, utm, insstdev, vel, imu):
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = utm.easting - self.origin.x
        self.odom.pose.pose.position.y = utm.northing - self.origin.y
        self.odom.pose.pose.position.z = utm.height - self.origin.z
        self.odom.pose.pose.orientation = imu.orientation

        self.position_covariance = np.diag([utm.easting_sigma**2, utm.northing_sigma**2, utm.height_sigma**2])
        self.odom.pose.covariance = ((np.bmat([
            [self.position_covariance, np.zeros([3, 3])                              ],
            [np.zeros([3, 3]),         np.reshape(imu.orientation_covariance, (3, 3))]
                                              ])).A.flatten()).tolist()

        self.odom.twist.twist.angular = imu.angular_velocity

        self.pure_v = [np.cos(np.radians(vel.track_ground)) * vel.horizontal_speed,
                       np.sin(np.radians(vel.track_ground)) * vel.horizontal_speed,
                       vel.vertical_speed,
                       0                                                          ]
        self.q = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        self.body_v = quaternion_multiply(quaternion_multiply(quaternion_conjugate(self.q), self.pure_v), self.q)
        self.odom.twist.twist.linear.x = self.body_v[0]
        self.odom.twist.twist.linear.y = self.body_v[1]
        self.odom.twist.twist.linear.z = self.body_v[2]
        self.lin_vel_cov = np.diag(np.array([insstdev.east_velocity_dev**2,
                                             insstdev.north_velocity_dev**2,
                                             insstdev.up_velocity_dev**2]))
        self.odom.twist.covariance = ((np.bmat([
            [self.lin_vel_cov, np.zeros([3, 3])                                  ],
            [np.zeros([3, 3]), np.reshape(imu.angular_velocity_covariance, (3, 3))]
            ])).A.flatten()).tolist()
        self.pub_odom.publish(self.odom)

        self.t.header.stamp = rospy.Time.now()
	self.t.transform.translation = self.odom.pose.pose.position
        self.t.transform.rotation = self.odom.pose.pose.orientation
        self.br.sendTransform(self.t)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('novatel_odom_publisher', anonymous=True)
    try:
        novatel_odom_publisher_class = NovatelOdomPublisher()
        odom_origin = rospy.get_param("novatel/origin")
        novatel_odom_publisher_class.sub_and_pub(odom_origin)
    except rospy.ROSInterruptException:
        pass
