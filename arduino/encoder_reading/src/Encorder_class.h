#include <Arduino.h>

const double pi = 3.141592653589793;  // 円周率

struct Encorder {
  // 定数の宣言
  struct Constants {
    const int pinA;
    const int pinB;
    const double periodTime;  // サンプリング周期 [msec]
    const double resolution;  // 分解能: 45(減速比) * 3(エンコーダの分解能) * 4(逓倍)
    Constants(int a, int b, double pt, double res)
      : pinA(a), pinB(b), periodTime(pt), resolution(res) {}
  } constants;
  
  // コンストラクタ(変数の格納をするとこ)
  Encorder(int a, int b, double pt, double res)
    : constants(a, b, pt, res) {}

  // 変数の宣言
  volatile long count = 0;     // カウント数を記録する変数
  volatile long preCount = 0;  // 前回の呼び出し時のカウント数

  // 割り込み用の関数の宣言
  void count_A(); // A相のカウント
  void count_B(); // B相のカウント
  double omega(); // 角速度の計算
  double countSpeed(); // カウント速度の計算
  void reserveCount(); // 前回変数の保存
};