#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import rospy
from std_msgs.msg import Bool

# rosの初期化
rospy.init_node('motor_on', anonymous=True)
arduino_port = rospy.get_param('~arduino_port', '/dev/ttyACM0')  # パラメータを取得、デフォルト値は '/dev/ttyACM0'
rate = rospy.Rate(10) # 10hz
rospy.loginfo("motor_onを開始します。")
motor_pub = rospy.Publisher('/motor_on', Bool, queue_size=10)

ser = serial.Serial(arduino_port, 9600, timeout=1)
ser.flush()

def send_command(command):
    ser.write(f"{command}\n".encode('utf-8'))

try:
    while rospy.is_shutdown() is False:
        command = input("Enter command (start/stop): ")
        # Arduinoにコマンドを送信
        if command == "start":
            motor_pub.publish(True)
        elif command == "stop":
            motor_pub.publish(False)
        else:
            print(f"Invalid command: {command}")
            continue
        send_command(command)
        print(f"Command sent: {command}")

        rate.sleep() # 10hz
        
except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
