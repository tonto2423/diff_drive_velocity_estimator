// Encorder_class.hの実装
#include <Arduino.h>
#include "Encorder_class.h"

// 参考：https://www.shujima.work/entry/2018/07/29/013935
void Encorder::count_A() { count += digitalRead(constants.pinA) == digitalRead(constants.pinB) ? -1 : 1; }
void Encorder::count_B() { count += digitalRead(constants.pinA) == digitalRead(constants.pinB) ? 1 : -1; }
double Encorder::omega() {
    // 角速度 [rad/sec]を返す関数
    return 2 * pi * (count - preCount) / (constants.periodTime * constants.resolution) * 1000;
}
double Encorder::countSpeed() {
    // カウント速度 [count/sec]を返す関数
    return (count - preCount) / constants.periodTime * 1000;
}
void Encorder::reserveCount() {
    // 前回変数の保存
    preCount = count;
}