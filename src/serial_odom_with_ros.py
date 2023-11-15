#!/usr/bin/env python3
# rosライブラリのインポート
import rospy
# 通信関係
import serial
import serial.tools.list_ports
# import time # rate.sleep()で代用
# 自作ライブラリ
import odometry_class_with_ros as odometry_class
import cobs_class
import os
import numpy as np

# rosの初期化
rospy.init_node('odometry_node', anonymous=True)
rate = rospy.Rate(10) # 10hz
rospy.loginfo("odometry_nodeを開始します。")

# シリアルポートの選択
ports = list(serial.tools.list_ports.comports())
for port in ports:
    description=port.description
    if 'STMicroelectronics STLink Virtual COM Port' in description:
        print(port.device)
serial_port = port.device  # シリアルポート
baud_rate = 115200  # ボーレート（デバイスに合わせて設定）

# 各クラスの初期化
ser = serial.Serial(serial_port, baud_rate, timeout=1) # シリアル通信の初期化
odom = odometry_class.Odometry() # オドメトリの初期化
cobs_class = cobs_class.CobsClass() # COBSデコード関数のインスタンス化

# デバッグ用前回変数の初期化
previous_counts = [0, 0, 0]
previous_x = 0
previous_y = 0
previous_theta = 0

try:
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            # データを読み取り
            data = ser.read(ser.in_waiting)
            messages = cobs_class.split_data(data)

            for message in messages:
                counts, speeds = cobs_class.decode_cobs_data(message)
                if counts and speeds:
                    # オドメトリの計算
                    delta_x, delta_y, delta_theta = odom.calculate_velocity(counts[0], counts[1], counts[2])
                    x, y, theta = odom.calculate_pose(delta_x, delta_y, delta_theta)
                    # os.system('cls')
                    # print("Counts:", counts, "Speeds:", speeds, "x: ", int(x * 1000.0), "[mm]", "y: ", int(y * 1000.0), "[mm]", "theta: ", theta * 180.0 / np.pi, "[deg]")
                    # /odomのtfを送信
                    odom.tf_broadcast()
                    # デバッグ用の処理
                    if (x - previous_x > 10 or y - previous_y > 10):
                        print("x, yが10を超えました。")
                        print("previous_counts:", previous_counts, "previous_x:", previous_x, "previous_y:", previous_y, "previous_theta:", previous_theta)
                        print("counts:", counts, "x:", x, "y:", y, "theta:", theta)
                        break
                    # 前回データの保存
                    previous_counts = counts
                    previous_x = x
                    previous_y = y
                    previous_theta = theta

        # シリアルポートの読み取り間隔（必要に応じて調整）
        # time.sleep(0.04)
        rate.sleep()
    rospy.spin()

except KeyboardInterrupt:
    print("プログラムを終了します。")

finally:
    # シリアルポートを閉じる
    ser.close()