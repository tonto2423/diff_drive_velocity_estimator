#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import tf.transformations

class Pose2DToOdometry:
    def __init__(self):
        # サブスクライバの定義
        rospy.Subscriber('/estimated_pose', Pose2D, self.pose_callback)
        rospy.Subscriber('/estimated_velocity', Twist, self.velocity_callback)
        
        # パブリッシャの定義
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Odometryメッセージのインスタンスを作成
        self.odom_msg = Odometry()
        
        self.delta_t = 0.1  # サンプリング周期 [sec]

    def pose_callback(self, data):
        # 位置データのセット
        self.odom_msg.pose.pose.position.x = data.x
        self.odom_msg.pose.pose.position.y = data.y
        self.odom_msg.pose.pose.position.z = 0  # 2Dのため、zは0とする

        # 姿勢データのセット（四元数に変換）
        quaternion = tf.transformations.quaternion_from_euler(0, 0, data.theta)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        
    def velocity_callback(self, data):
        # 速度データの格納
        self.odom_msg.twist.twist = data

    def run(self):
        rate_frequency = 1.0 / self.delta_t     # サンプリング周期の逆数を計算
        rate = rospy.Rate(rate_frequency)       # 計算された頻度でrospy.Rateを設定
        while not rospy.is_shutdown():
            rospy.loginfo("データ変換なう")
            # 変換したデータをパブリッシュ
            self.pub.publish(self.odom_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pose2d_to_odometry')
    converter = Pose2DToOdometry()
    converter.run()
    # rospy.spin()