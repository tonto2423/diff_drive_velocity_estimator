/**
 * @file main.cpp

 * @brief A相、B相の割り込みで四逓倍カウントを行うプログラム．一定のサンプリング周期でタイマ割り込みをする．
**/
#include <Arduino.h>
#include <MsTimer2.h>
#include "Encoder.h"

#include <ros.h>
#include <diff_drive_velocity_estimator/EncoderData.h>  // 追加されたヘッダーファイルへのパス。必要に応じて変更する。

const double periodTime = 100.0;      // サンプリング周期 [msec]
const double resolution = 540.0;      // 分解能: 45(減速比) * 3(エンコーダの分解能) * 4(逓倍)

// インスタンス
Encoder encRight(2, 3, periodTime, resolution);
Encoder encLeft(21, 20, periodTime, resolution);

// attachInterrupt関数がクラスのインスタンスメソッドを直接サポートしていないため
void countA_wrapper_R() {encRight.countA();}
void countB_wrapper_R() {encRight.countB();}

void countA_wrapper_L() {encLeft.countA();}
void countB_wrapper_L() {encLeft.countB();}

// タイマ割り込みする関数の宣言
void monitor() {
  // 角速度の表示
  Serial.print("Right:");
  Serial.print(encRight.omega());
  Serial.print(",\t");
  Serial.print("Left:");
  Serial.println(encLeft.omega());
  // 前回変数の保存
  encRight.reserveCount();
  encLeft.reserveCount();
}

void setup() {
  Serial.begin(115200);
  //入力ピンが変化した時に割り込みを入れる
  attachInterrupt(digitalPinToInterrupt(encRight.getPinA()), countA_wrapper_R, CHANGE);  // Aピンが変化した時countA()を呼び出す
  attachInterrupt(digitalPinToInterrupt(encRight.getPinB()), countB_wrapper_R, CHANGE);  // Bピンが変化した時countB()を呼び出す
  attachInterrupt(digitalPinToInterrupt(encLeft.getPinA()), countA_wrapper_L, CHANGE);   // Aピンが変化した時countA()を呼び出す
  attachInterrupt(digitalPinToInterrupt(encLeft.getPinB()), countB_wrapper_L, CHANGE);   // Bピンが変化した時countB()を呼び出す
  MsTimer2::set((int)periodTime, monitor);
  MsTimer2::start();
}

void loop() {}