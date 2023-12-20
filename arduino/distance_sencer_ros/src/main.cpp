// 参考：https://rb-station.com/blogs/article/hc-sr04-arduino
// 起動コマンド: rosrun rosserial_python serial_node.py __name:=distance _port:=/dev/ttyACM0 _baud:=115200
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#define MONITOR_SPEED 115200

ros::NodeHandle nh;

std_msgs::Float64 distance_msg;
ros::Publisher pub("distance", &distance_msg);

#define TRIG 3
#define ECHO 2
double duration = 0.0;
double distance = 0.0;
double speed_of_sound = 331.5 + 0.6 * 25.0; // 25℃の気温の想定

void setup() {
  nh.getHardware()->setBaud(MONITOR_SPEED); // ROSとのシリアル通信のボーレートの設定
  Serial.begin(MONITOR_SPEED);              // ハードウェアシリアルのボーレート

  nh.initNode();
  nh.advertise(pub);

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
}

void loop() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  if (duration > 0) {
    duration = duration / 2; // 往路にかかった時間
    distance = duration * speed_of_sound / 1000000.0; // 距離をメートル単位で計算
    distance_msg.data = distance;
    pub.publish(&distance_msg);
  }
  nh.spinOnce(); // この関数を呼び出さないとメッセージが送信されない
  delay(10);
}