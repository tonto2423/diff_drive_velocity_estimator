#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf
import tf.transformations

class odomToBaseLink:
    def __init__(self):
        # サブスクライバの定義
        rospy.Subscriber('/odom', Odometry, self.callback)
        # ブロードキャスターのインスタンスを生成
        self.br = tf.TransformBroadcaster()        
        # 変数の初期化
        self.current_time = rospy.Time.now()    # 時刻
        self.x = 0.0                            # x変位
        self.y = 0.0                            # y変位
        self.z = 0.0                            # z変位 - これが欠けていました
        self.theta = 0.0                        # 角変位
        self.delta_t = 0.1  # サンプリング周期 [sec]

    def callback(self, data):
        # 現在の時刻を取得
        self.current_time = rospy.Time.now()
        
        # PoseWithCovarianceStampedメッセージから位置と姿勢を取得
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        rospy.loginfo("odom to base_link")        
        # Transformを送信
        self.br.sendTransform(
            (self.x, self.y, self.z),
            tf.transformations.quaternion_from_euler(0, 0, self.theta),
            self.current_time,
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    rospy.init_node('odom_to_base_link')
    converter = odomToBaseLink()
    rospy.spin()  # これを追加して、ノードが持続的に実行されるようにします
