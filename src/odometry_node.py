#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose2D
import math

class OdometryNode:
    def __init__(self):
        # Subscriberの定義
        rospy.Subscriber('/EncoderData', Float64, self.encoder_callback)
        
        # Publisherの定義
        self.velocity_pub = rospy.Publisher('/estimated_velocity', Twist, queue_size=10)
        self.pose_pub = rospy.Publisher('/estimated_pose', Pose2D, queue_size=10)

        # 変数の初期化
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
        self.delta_t = 0.010        # サンプリング周期 [sec]

    def encoder_callback(self, data):
        # エンコーダデータを受け取るコールバック関数
        self.countLeft = data.countLeft
        self.omegaLeft = 2.0 * self.pi * data.countSpeedLeft / self.resolution
        self.countRight = data.countRight
        self.omegaRight = 2.0 * self.pi * data.countSpeedRight / self.resolution

    def compute_velocity(self):
        # エンコーダデータからロボットの速度を計算する関数
        linearVel = self.tireRadius / 2.0 * (self.omegaRight - self.omegaLeft)              # ロボット線速度 [m/sec]
        angularVel = self.tireRadius / self.tireDist * (self.omegaRight + self.omegaLeft)   # ロボット角速度 [rad/sec]
        # メッセージへの格納
        velocity_msg = Twist()
        velocity_msg.linear.x = linearVel
        velocity_msg.angular.z = angularVel
        return velocity_msg

    def compute_pose(self):
        # エンコーダデータからロボットの自己位置を計算する関数
        v = compute_velocity().linear.x                     # ロボット線速度の取得
        omega = compute_velocity().angular.z                # ロボット角速度の取得
        x_position += v * math.cos(theta) * self.delta_t
        y_position += v * math.sin(theta) * self.delta_t
        theta += omega * self.delta_t
        theta = (theta + math.pi) % (2 * math.pi) - math.pi
        # メッセージへの格納
        pose_msg = Pose2D()
        pose_msg.x = x_position
        pose_msg.y = y_position
        pose_msg.theta = theta
        return pose_msg

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if self.encoder_data:
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
