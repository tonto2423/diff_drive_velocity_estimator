#!/usr/bin/env python3
import rospy
from diff_drive_velocity_estimator.msg import EncoderData

# 偽エンコーダパブリッシャ
def EncoderTalker():
    rospy.init_node('EncoderTalker')
    pub = rospy.Publisher('/EncoderData', EncoderData, queue_size=10)
    
    encoder_data = EncoderData() # インスタンスの生成
    encoder_data.countLeft = 2
    encoder_data.countRight = -2
    encoder_data.countSpeedLeft = 20.0
    encoder_data.countSpeedRight = -20.0
    
    r = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("データ送信中卍")
        pub.publish(encoder_data)
        r.sleep()
        
if __name__ == '__main__':
    try:
        EncoderTalker()
    except rospy.ROSInterruptException: pass