/**
 * @file LineTracer.h
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 
 * @version 0.1
 * @date 2020-10-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EV3_APP_LINETRACER_H
#define EV3_APP_LINETRACER_H

#include "odometry/ColorSensorCalculator.h"
#include "control/PIDController.h"
#include "control/MotorRunner.h"

#define LIGHT_WHITE 40                                   // 白色の光センサ値
#define LIGHT_BLACK 0                                    // 黒色の光センサ値
#define TARGET_REFLECT ((LIGHT_WHITE + LIGHT_BLACK) / 2) // PID目標,光反射値, 黒vs白のreflection中間値
#define TARGET_HSV 59                                    // PID目標,HSV値, 青vs白のsaturation中間値
#define MOTOR_POWER 70                                   //前進速度

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   ライントレーサー クラス
 * 
 * @class   LineTracer
 */
class LineTracer
{
private:
    int turn; // turn ratio
public:
    LineTracer(); // Constructor

    // PID制御の実行
    void run(PIDController *PIDreflect,
             PIDController *PIDhsv,
             ColorSensorCalculator *ColorSensor,
             MotorRunner *Motor);

    int getTurnRatio(); // turn ratio(舵角)の取得
};

// ******** 以降の実装部はリリース時にcppに分離 ******** ******** ********

// Constructor
LineTracer::LineTracer()
    : turn(0)
{
}

/**
 * @brief PID制御の実行
 * 
 * @fn    void LineTracer::run(PIDController*,PIDController*,ColorSensorCalculator*,MotorRunner*)
 * @param PIDreflect    (PIDController*)HSV明度によるPID制御
 * @param PIDhsv        (PIDController*)HSV彩度によるPID制御
 * @param ColorSensor   (ColorSensorCalculator*)RGB=>HSVへの変換
 * @param Motor         (MotorRunner*)モーター制御
 * @return 無し
 */
void LineTracer::run(PIDController *PIDreflect,
                     PIDController *PIDhsv,
                     ColorSensorCalculator *ColorSensor,
                     MotorRunner *Motor)
{
    // -------- HSV値PID --------
    PIDhsv->setPIDactual(ColorSensor->getHSVsat()); // 現在satuation値取得
    PIDhsv->calc(TARGET_HSV, _EDGE);
    // -------- 光反射値PID --------
    PIDreflect->setPIDactual(ColorSensor->getHSVval()); // 現在value値取得
    PIDreflect->calc(TARGET_REFLECT, -1 * _EDGE);
    // -------- 青色判断 --------
    if (ColorSensor->getHSVsat() >= TARGET_HSV)
        turn = PIDhsv->getPIDvalue(); //青色検知したらHSVに切り替えてSaturationで制御する
    else                              //if (ColorSensor->hsv->sat <= 40) // 戻りが遅くなるので黒のしきい値やめる
        turn = PIDreflect->getPIDvalue();

    // -------- モーター出力 --------
    Motor->run(MOTOR_POWER, turn);
}

/**
 * @brief   turn ratio(舵角)の取得
 * 
 * @fn      int LineTracer::getTurnRatio()
 * @return  int turn: 舵角
 */
inline int LineTracer::getTurnRatio()
{
    return turn;
}

#endif // EV3_APP_LINETRACER_H