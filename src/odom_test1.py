#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from diff_drive_velocity_estimator.msg import EncoderData
import math

# サブスクライブのテスト
class OdometryNode:
    def __init__(self):
        # Subscriberの定義
        rospy.Subscriber('/EncoderData', EncoderData, self.encoder_callback)
        
        # Publisherの定義
        self.velocity_pub = rospy.Publisher('/estimated_velocity', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/estimated_pose', Pose2D, queue_size=10)
        
        # エンコーダ速度受け取り
        self.countLeft = 0          # 左のカウント
        self.omegaLeft = 0.0        # 左の角速度 [rad/sec]
        self.countRight = 0         # 右のカウント
        self.omegaRight = 0.0       # 右の角速度 [rad/sec]
        
        self.delta_t = 0.010        # サンプリング周期 [sec]
        
    def encoder_callback(self, data):
        # エンコーダデータを受け取るコールバック関数
        self.countLeft = data.countLeft
        # self.omegaLeft = 2.0 * self.pi * data.countSpeedLeft / self.resolution
        self.countRight = data.countRight
        # self.omegaRight = 2.0 * self.pi * data.countSpeedRight / self.resolution
        self.omegaLeft = data.countSpeedLeft
        self.omegaRight = data.countSpeedRight
        

    def compute_velocity(self):
        # メッセージへの格納
        velocity_msg = Twist()
        velocity_msg.linear.x = self.omegaLeft
        velocity_msg.angular.z = self.omegaRight
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
            rospy.loginfo("受け取ったデータ横流し中卍")

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