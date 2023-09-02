// A相、B相の割り込みで四逓倍カウントを行うプログラム
// タイマ割り込みでシリアルプリントする
#include <Arduino.h>
#include <MsTimer2.h>
#include "Encoder.h"

const double periodTime = 100.0;      // サンプリング周期 [msec]
const double resolution = 540.0;      // 分解能: 45(減速比) * 3(エンコーダの分解能) * 4(逓倍)

// インスタンス
Encoder enc_Right(2, 3, periodTime, resolution);
Encoder enc_Left(21, 20, periodTime, resolution);

// attachInterrupt関数がクラスのインスタンスメソッドを直接サポートしていないため
void count_A_wrapper_R() {enc_Right.count_A();}
void count_B_wrapper_R() {enc_Right.count_B();}

void count_A_wrapper_L() {enc_Left.count_A();}
void count_B_wrapper_L() {enc_Left.count_B();}

// タイマ割り込みする関数の宣言
void monitor() {
  // 角速度の表示
  Serial.print("Right:");
  Serial.print(enc_Right.omega());
  Serial.print(",\t");
  Serial.print("Left:");
  Serial.println(enc_Left.omega());
  // 前回変数の保存
  enc_Right.reserveCount();
  enc_Left.reserveCount();
}

void setup() {
  Serial.begin(115200);
  //入力ピンが変化した時に割り込みを入れる
  attachInterrupt(digitalPinToInterrupt(enc_Right.getPinA()), count_A_wrapper_R, CHANGE);  // Aピンが変化した時count_A()を呼び出す
  attachInterrupt(digitalPinToInterrupt(enc_Right.getPinB()), count_B_wrapper_R, CHANGE);  // Bピンが変化した時count_B()を呼び出す
  attachInterrupt(digitalPinToInterrupt(enc_Left.getPinA()), count_A_wrapper_L, CHANGE);   // Aピンが変化した時count_A()を呼び出す
  attachInterrupt(digitalPinToInterrupt(enc_Left.getPinB()), count_B_wrapper_L, CHANGE);   // Bピンが変化した時count_B()を呼び出す
  MsTimer2::set((int)periodTime, monitor);
  MsTimer2::start();
}

void loop() {}