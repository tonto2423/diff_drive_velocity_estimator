#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Bool bool_msg;
ros::Publisher pub("pin_state", &bool_msg);

byte pin[6] = {
    2, // ENA
    3, // IN1
    4, // IN2
    5, // ENB
    6, // IN4
    7  // IN3
};

void setup() {
    for(int i=0; i<6; i++) {
        pinMode(pin[i], OUTPUT);
    }
    pinMode(8, INPUT);
    
    nh.getHardware()->setBaud(115200); // ROSとのシリアル通信のボーレートの設定
    Serial.begin(115200);
    nh.initNode();
    nh.advertise(pub);

    Serial.println("ROS node started");
}

void loop() {
    bool pin_state = digitalRead(8);
    if(pin_state){
        Serial.println("manji");
        digitalWrite(2, HIGH); // ENA
        digitalWrite(3, LOW);  // IN1
        digitalWrite(4, HIGH); // IN2
        digitalWrite(5, HIGH); // ENB
        digitalWrite(6, HIGH); // IN4
        digitalWrite(7, LOW);  // IN3
    } else {
        for(int i=0; i<6; i++){
            digitalWrite(pin[i], LOW);
        }
    }

    // Publish message
    bool_msg.data = pin_state;
    pub.publish(&bool_msg);
    nh.spinOnce();

    // Add a small delay to prevent overwhelming the serial line
    delay(100);
}
