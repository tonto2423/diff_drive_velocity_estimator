#!/usr/bin/env python3
import rospy
import tf

def publish_transform():
    # ノードの初期化
    rospy.init_node('lidar_tf_broadcaster')

    # tf broadcasterのインスタンスを作成
    br = tf.TransformBroadcaster()

    # ループレートを定義
    rate = rospy.Rate(10.0)
    
    # LiDARの位置と姿勢を定義
    x = -0.005
    y = 0
    z = 0.145
    roll = 0
    pitch = 0
    yaw = 0
        
    while not rospy.is_shutdown():
        # 現在の時刻を取得
        current_time = rospy.Time.now()
        # Transformを送信
        br.sendTransform((x, y, z),
                         tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                         current_time,
                         "laser",
                         "base_link")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass