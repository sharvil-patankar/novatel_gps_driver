#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from novatel_gps_msgs.msg import NovatelUtmPosition, NovatelVelocity, Insstdev
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_multiply, quaternion_conjugate
import message_filters


class NovatelOdomPublisher:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.pub_odom = rospy.Publisher('novatel/odom', Odometry, queue_size=10)
        self.utm_sub = message_filters.Subscriber('novatel/bestutm', NovatelUtmPosition)
        self.insstdev_sub = message_filters.Subscriber('novatel/insstdev', Insstdev)
        self.vel_sub = message_filters.Subscriber('novatel/bestvel', NovatelVelocity)
        self.imu_sub = message_filters.Subscriber('novatel/imu', Imu)

        self.odom = Odometry()
        self.odom.header.frame_id = 'map'
        self.odom.child_frame_id = 'base_link'

        self.buff = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buff)
        # self.trans = self.buff.lookup_transform('gps_imu','base_footprint', rospy.Time.now(), rospy.Duration(1.0))
        # self.trans.transform()
        self.lis.unregister()
        self.temp_pose = PoseStamped
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = 'map'
        self.t.child_frame_id = 'bese_footprint'

    def sub_and_pub(self):
        self.ts = message_filters.ApproximateTimeSynchronizer([
            self.utm_sub, self.insstdev_sub, self.vel_sub, self.imu_sub], 10, 0.01)
        rospy.loginfo("time synchronized")
	self.ts.registerCallback(self.publish_odom)
        rospy.spin()

    def publish_odom(self, utm, insstdev, vel, imu):
        rospy.loginfo("In callback")
	self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = utm.easting
        self.odom.pose.pose.position.y = utm.northing
        self.odom.pose.pose.position.z = utm.height
        self.odom.pose.pose.orientation = imu.orientation

        self.position_covariance = np.diag([utm.easting_sigma**2, utm.northing_sigma**2, utm.height_sigma**2])
        self.odom.pose.covariance = ((np.bmat([
            [self.position_covariance, np.zeros([3, 3])                              ],
            [np.zeros([3, 3]),         np.reshape(imu.orientation_covariance, (3, 3))]
                                               ])).A.flatten()).tolist()

        self.temp_pose.header = self.odom.header
        self.temp_pose.pose = self.odom.pose.pose
        self.temp_pose = self.buff.transform(self.temp_pose,'base_footprint')
        self.odom.header = self.temp_pose.header
        self.odom.pose = self.temp_pose.pose

        self.odom.twist.twist.angular = imu.angular_velocity

        self.pure_v = [np.cos(np.radians(vel.track_ground)) * vel.horizontal_speed,
                       np.sin(np.radians(vel.track_ground)) * vel.horizontal_speed,
                       vel.vertical_speed,
                       0                                                      ]
        self.q = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        self.body_v = quaternion_multiply(quaternion_multiply(quaternion_conjugate(self.q), self.pure_v), self.q)
        self.odom.twist.twist.linear.x = self.body_v[0]
        self.odom.twist.twist.linear.y = self.body_v[1]
        self.odom.twist.twist.linear.z = self.body_v[2]
	self.lin_vel_cov = np.diag(np.array([ insstdev.east_velocity_dev**2, insstdev.north_velocity_dev**2, insstdev.up_velocity_dev**2]))
        self.odom.twist.covariance = ((np.bmat([
            [ self.lin_vel_cov, np.zeros([3, 3])                                  ],
            [ np.zeros([3, 3]), np.reshape(imu.angular_velocity_covariance, (3, 3))]
                                                ])).A.flatten()).tolist()
        self.pub_odom.publish(self.odom)

        self.t_temp.header.stamp = self.odom.header.stamp
        self.t_temp.transform.translation = self.odom.pose.pose.position
        self.t_temp.transform.rotation = self.odom.pose.pose.orientation
        self.t = self.buff.transform(self.t_temp,'base_footprint')
        self.br.sendTransform(self.t)

        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('novatel_odom_publisher', anonymous=True)
    try:
        novatel_odom_publisher_class = NovatelOdomPublisher()
        novatel_odom_publisher_class.sub_and_pub()
    except rospy.ROSInterruptException:
        pass
