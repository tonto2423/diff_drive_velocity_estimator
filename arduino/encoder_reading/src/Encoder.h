/**
 * @file Encoder.h

 * @brief エンコーダのパルスを読んで角速度やカウント速度を出力するクラス
**/
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder
{
private:
  // 定数の宣言
  const int pinA;          // A相
  const int pinB;          // B相
  const double periodTime; // サンプリング周期 [msec]
  const double resolution; // 分解能: 45(減速比) * 3(エンコーダの分解能) * 4(逓倍)

  // 変数の宣言
  volatile long count = 0;    // カウント数を記録する変数
  volatile long preCount = 0; // 前回の呼び出し時のカウント数

public:
  // コンストラクタ
  Encoder(int a, int b, double pt, double res)
      : pinA(a), pinB(b), periodTime(pt), resolution(res)
  {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
  }

  // メンバ関数
  int getPinA() const { return pinA; } // pinAのgetter
  int getPinB() const { return pinB; } // pinBのgetter
  void countA();                      // A相のカウント
  void countB();                      // B相のカウント
  double omega();                      // 角速度の計算
  double countSpeed();                 // カウント速度の計算
  void reserveCount();                 // 前回変数の保存
};

#endif