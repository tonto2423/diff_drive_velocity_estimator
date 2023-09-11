#!/usr/bin/env python3
# mapにおけるbase_linkの位置を決める
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class base_linkToMap():
    def __init__(self):
        topic_name = rospy.get_param('~pose_topic', '/amcl_pose')
        # amcl_poseトピックをサブスクライブ
        rospy.Subscriber(topic_name, PoseWithCovarianceStamped, self.pose_callback)
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
        
        rospy.loginfo("base_linkをmap上に出すぜ")
        # Transformを送信
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            tf.transformations.quaternion_from_euler(0.0, 0.0, self.theta),
            self.current_time,
            "map",
            "base_link"
        )

if __name__ == '__main__':
    # ノードの初期化
    rospy.init_node('base_link_to_map_tf_broadcaster')
    converter = base_linkToMap()
    rospy.spin()

    
