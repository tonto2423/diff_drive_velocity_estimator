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

# def find_arduino_uno_port():
#     ports = serial.tools.list_ports.comports()
#     for port in ports:
#         if "Arduino Uno" in port.description:
#             return port.device
#     return None

# arduino_port = find_arduino_uno_port()
# if arduino_port is None:
#     print("Arduino Uno not found")
# else:
#     print(f"Arduino Uno found on port: {arduino_port}")
#     # ここでシリアル接続を開始
#     ser = serial.Serial(arduino_port, 9600, timeout=1)
#     ser.flush()

ser = serial.Serial(arduino_port, 9600, timeout=1)
ser.flush()

def send_command(command):
    ser.write(f"{command}\n".encode('utf-8'))

def read_distance():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        if line.startswith("Distance:"):
            distance = float(line.split(" ")[1])
            print(f"Distance: {distance} m")

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

        # Arduinoからのデータを読み取る
        read_distance()

        rate.sleep() # 10hz
        
except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
