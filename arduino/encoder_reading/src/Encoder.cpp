// Encoder_class.hの実装
#include <Arduino.h>
#include "Encoder.h"
#include "constants.h"

// 参考：https://www.shujima.work/entry/2018/07/29/013935
void Encoder::countA() {
    int A = digitalRead(pinA);
    int B = digitalRead(pinB);
    if (A == B) { count += -1; }
    else { count += 1; }
}
void Encoder::countB() {
    int A = digitalRead(pinA);
    int B = digitalRead(pinB);
    if (A == B) { count += 1; }
    else { count += -1; }
}
double Encoder::omega()
{
    // 角速度 [rad/sec]を返す関数
    return 2 * constants::pi * (count - preCount) / (periodTime * resolution) * 1000;
}
double Encoder::countSpeed()
{
    // カウント速度 [count/sec]を返す関数
    return (count - preCount) / periodTime * 1000;
}
void Encoder::reserveCount()
{
    // 前回変数の保存
    preCount = count;
}