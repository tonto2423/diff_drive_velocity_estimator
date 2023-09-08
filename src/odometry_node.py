#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from diff_drive_velocity_estimator.msg import EncoderData
import math

# サブスクライブのテスト
class OdometryNode:
    def __init__(self):
        # Subscriberの定義
        rospy.Subscriber('/encoder_data', EncoderData, self.encoder_callback)
        
        # Publisherの定義
        self.velocity_pub = rospy.Publisher('/estimated_velocity', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/estimated_pose', Pose2D, queue_size=10)

        # エンコーダ速度受け取り
        self.countLeft = 0          # 左のカウント
        self.omegaLeft = 0.0        # 左の角速度 [rad/sec]
        self.countRight = 0         # 右のカウント
        self.omegaRight = 0.0       # 右の角速度 [rad/sec]
        self.resolution = 540.0     # タイヤ分解能 (ギヤ比、逓倍も含む)
        self.pi = 3.1415926535      # 円周率
        # 速度計算
        self.tireRadius = 0.033     # タイヤ半径 [m]
        self.tireDist = 0.146       # タイヤ間距離 [m]
        self.delta_t = 0.1          # サンプリング周期 [sec]
        # 位置計算
        self.x_position = 0.0       # 初期位置x [m]
        self.y_position = 0.0       #　初期位置y [m]
        self.theta = 0.0            # 初期角度位置theta [m]

        self.data_updated = False   # publish実行フラグ

    def encoder_callback(self, data):
        # エンコーダデータを受け取るコールバック関数
        self.countLeft = data.countLeft
        self.omegaLeft = 2.0 * self.pi * data.countSpeedLeft / self.resolution
        self.countRight = data.countRight
        self.omegaRight = 2.0 * self.pi * data.countSpeedRight / self.resolution
        self.data_updated = True  # フラグをセット

    def compute_velocity(self):
        # エンコーダデータからロボットの速度を計算する関数
        linearVel = self.tireRadius * (self.omegaRight - self.omegaLeft) / 2.0              # ロボット線速度 [m/sec]
        angularVel = self.tireRadius * (self.omegaRight + self.omegaLeft) / self.tireDist   # ロボット角速度 [rad/sec]
        # メッセージへの格納
        velocity_msg = Twist()
        velocity_msg.linear.x = linearVel
        velocity_msg.angular.z = angularVel
        # velocity_msg.linear.x = self.omegaLeft
        # velocity_msg.angular.z = self.omegaRight
        return velocity_msg

    def compute_pose(self, v, omega):
        # エンコーダデータからロボットの自己位置を計算する関数
        self.x_position += v * math.cos(self.theta) * self.delta_t
        self.y_position += v * math.sin(self.theta) * self.delta_t
        self.theta += omega * self.delta_t
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        # メッセージへの格納
        pose_msg = Pose2D()
        pose_msg.x = self.x_position
        pose_msg.y = self.y_position
        pose_msg.theta = self.theta
        return pose_msg

    def run(self):
        rate_frequency = 1.0 / self.delta_t     # サンプリング周期の逆数を計算
        rate = rospy.Rate(rate_frequency)       # 計算された頻度でrospy.Rateを設定
        while not rospy.is_shutdown():
            if self.data_updated:  # フラグをチェック
                rospy.loginfo("受け取ったデータ横流し中卍")

                # データを計算
                velocity_msg = self.compute_velocity()
                pose_msg = self.compute_pose(velocity_msg.linear.x, velocity_msg.angular.z)

                # データをpublish
                self.velocity_pub.publish(velocity_msg)
                self.pose_pub.publish(pose_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odometry_node', anonymous=True)
    node = OdometryNode()
    node.run()