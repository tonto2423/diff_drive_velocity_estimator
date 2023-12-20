import serial
import time
import sys

def get_arduino_port():
    if len(sys.argv) > 1:
        return sys.argv[1]  # コマンドラインからポート名を取得

arduino_port = get_arduino_port()

ser = serial.Serial(arduino_port, 9600, timeout=1)
ser.flush()
time.sleep(2)

def send_command(command):
    ser.write(f"{command}\n".encode('utf-8'))

def read_distance():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').rstrip()
        if line.startswith("Distance:"):
            distance = float(line.split(" ")[1])
            print(f"Distance: {distance} m")

try:
    while True:
        command = input("Enter command (start/stop): ")
        send_command(command)

        # Arduinoからのデータを読み取る
        read_distance()

        time.sleep(1)

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
