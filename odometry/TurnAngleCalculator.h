/**
 * @file TurnAngleCalculator.h
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 
 * @version 0.1
 * @date 2020-10-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EV3_APP_TURNANGLECALCULATOR_H
#define EV3_APP_TURNANGLECALCULATOR_H

#include "ev3api.h"
#include "etrobo_env.h"
#include "cmath"

#define HALFTRACK 77   /* 1/2トレッド */
#define WHEELRADIUS 50 /* 車輪半径 */

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   左右のホイール回転角,回転半径,回転角
 * 
 * @struct  turnangle_t
 * @note    サイズは20byte= int(4byte) x5
 */
typedef struct
{
    int leftWheel_deg;  /* 左ホイール回転角 */
    int rightWheel_deg; /* 右ホイール回転角 */
    int radius;         /* 車両の回転半径 */
    int omega;          /* 車両の回転角 */
    int MODE_straight;  /* 直進中判定モード,1(true)=直進,0(false)=直進以外 */
} turnangle_t;

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief 回転半径と回転角の測定クラス
 * 
 * @class TurnAngleCalculator
 */
class TurnAngleCalculator
{
private:
public:
    TurnAngleCalculator();                   // Constructor
    void calc(turnangle_t *angle, int turn); // 回転半径と回転角の計算
};

// ******** 以降の実装部はリリース時にcppに分離 ******** ******** ********

// Constructor
TurnAngleCalculator::TurnAngleCalculator()
{
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
}

/**
 * @brief   回転半径と回転角の計算
 * 
 * @fn      void TurnAngleCalculator::calc(turnangle_t *angle, int turn)
 * @param   angle   (turnangle_t*)回転半径と回転角の構造体
 * @param   turn    (int)ev3_motor_steerのturn_ratio
 * @return  無し
 */
void TurnAngleCalculator::calc(turnangle_t *angle, int turn)
{
    /* -------- ホイール回転角取得 -------- */
    angle->leftWheel_deg = ev3_motor_get_counts(left_motor);
    angle->rightWheel_deg = ev3_motor_get_counts(right_motor);
    /* 車両回転角:ω = r/2d*(θleft-θright), 車輪半径:r=50,トレッド:2d=154 */
    angle->omega = (angle->leftWheel_deg - angle->rightWheel_deg) * WHEELRADIUS / (2 * HALFTRACK);

    if (!angle->MODE_straight) /* -------- 直進中でなければ -------- */
    {
        if (turn == 0) /* -------- 舵角が0なら回転角リセット -------- */
        {
            ev3_motor_reset_counts(left_motor);
            ev3_motor_reset_counts(right_motor);
            angle->radius = 0;
            angle->MODE_straight = true;
        }
        else /* -------- 舵角が0以外なら計算 -------- */
        {
            /* 回転半径:ρ = d*(θleft+θright)/(θleft-θright), トレッド/2:d=77 */
            if ((angle->leftWheel_deg - angle->rightWheel_deg) == 0) /* 0除算回避 */
                angle->radius = 0;
            else
                angle->radius = 77 * (angle->leftWheel_deg + angle->rightWheel_deg) / (angle->leftWheel_deg - angle->rightWheel_deg);
        }
    }
    else /* -------- 直進モード -------- */
    {
        if (std::abs(turn) > 10) /* -------- 舵角が10を超えるまで何もしない -------- */
        {
            ev3_motor_reset_counts(left_motor);
            ev3_motor_reset_counts(right_motor);
            angle->MODE_straight = false;
        }
    }
}

#endif // EV3_APP_TURNANGLECALCULATOR_H