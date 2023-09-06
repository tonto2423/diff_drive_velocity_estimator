#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from diff_drive_velocity_estimator.msg import EncoderData
import math

# パブリッシャのテスト
class OdometryNode:
    def __init__(self):
        # Publisherの定義
        self.velocity_pub = rospy.Publisher('/estimated_velocity', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/estimated_pose', Pose2D, queue_size=10)
        self.delta_t = 0.010        # サンプリング周期 [sec]

    def compute_velocity(self):
        # メッセージへの格納
        velocity_msg = Twist()
        velocity_msg.linear.x = 1.0
        velocity_msg.angular.z = 2.0
        return velocity_msg

    def compute_pose(self):
        # メッセージへの格納
        pose_msg = Pose2D()
        pose_msg.x = 3.0
        pose_msg.y = 4.0
        pose_msg.theta = 5.0
        return pose_msg

    def run(self):
        rate_frequency = 1.0 / self.delta_t     # サンプリング周期の逆数を計算
        rate = rospy.Rate(rate_frequency)       # 計算された頻度でrospy.Rateを設定
        while not rospy.is_shutdown():
            # データを計算
            velocity_msg = self.compute_velocity()
            pose_msg = self.compute_pose()

            # データをpublish
            self.velocity_pub.publish(velocity_msg)
            self.pose_pub.publish(pose_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odometry_node', anonymous=True)
    node = OdometryNode()
    node.run()