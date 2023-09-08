#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class base_linkTobase_frame():
    def __init__(self):
        # amcl_poseトピックをサブスクライブ
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        # ブロードキャスターのインスタンスを生成
        self.br = tf.TransformBroadcaster()
        # 変数の初期化
        self.current_time = rospy.Time.now()    # 時刻
        self.x = 0.0                            # x変位
        self.y = 0.0                            # y変位
        self.theta = 0.0                        # 角変位
    
    def pose_callback(self, data):
        # 現在の時刻を取得
        self.current_time = rospy.Time.now()
        
        # PoseWithCovarianceStampedメッセージから位置と姿勢を取得
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        _, _, self.theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    
    def run(self):
        delta_t = 0.1                       # サンプリング周期 [sec]
        rate_frequency = 1.0 / delta_t      # サンプリング周期の逆数を計算
        rate = rospy.Rate(rate_frequency)   # 計算された頻度でrospy.Rateを設定
        
        # base_linkからbase_frameまでの変換パラメータを定義
        x = self.x
        y = self.y
        z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = self.theta
            
        while not rospy.is_shutdown():            
            # Transformを送信
            self.br.sendTransform(
                (x, y, z),
                tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                self.current_time,
                "base_link",
                "base_frame"
            )
            # 次のループまで待機
            rate.sleep()

if __name__ == '__main__':
    # ノードの初期化
    rospy.init_node('base_link_to_base_frame_tf_broadcaster')
        
    converter = base_linkTobase_frame()
    try:
        converter.run()
    except rospy.ROSInterruptException:
        pass
