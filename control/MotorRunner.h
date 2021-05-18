/**
 * @file MotorRunner.h
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 
 * @version 0.1
 * @date 2020-10-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EV3_APP_MOTORRUNNER_H
#define EV3_APP_MOTORRUNNER_H

#include "ev3api.h"
#include "etrobo_env.h"

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief モーター出力クラス
 * 
 * @class MotorRunner
 * @attention ev3組み込みのMotorクラスはリンクエラーが出るのでやむなく作った
 */
class MotorRunner
{
private:
    //int power;                 // 前進速度 (-100 to 100)
    //int turn;                  // 舵角 (-100 to 100)
    const motor_port_t left_motor;
    const motor_port_t right_motor;

public:
    MotorRunner();
    void config();
    void run(int power, int turn);
    void stop();
    void reset();
};

// ******** 以降の実装部はリリース時にcppに分離 ******** ******** ********

// Constructor
MotorRunner::MotorRunner()
    : left_motor(EV3_PORT_C),
      right_motor(EV3_PORT_B)
{
    // this->config();
    this->reset();
}

/**
 * @brief モーター出力ポートの設定
 * 
 * @fn void MotorRunner::config()
 */
inline void MotorRunner::config()
{
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
}

/**
 * @brief モーター出力
 * 
 * @fn void MotorRunner::run(int power, int turn)
 * @param power (int)前進速度 (-100 to 100)
 * @param turn  (int)舵角 (-100 to 100)
 * @return 無し
 */
inline void MotorRunner::run(int power, int turn)
{
    ev3_motor_steer(left_motor, right_motor, power, turn);
}

/**
 * @brief モーター停止
 * @fn void MotorRunner::stop()
 */
inline void MotorRunner::stop()
{
    ev3_motor_stop(left_motor, true);
    ev3_motor_stop(right_motor, true);
}

/**
 * @brief 走行モーターエンコーダーリセット
 * @fn void MotorRunner::reset()
 */
inline void MotorRunner::reset()
{
    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);
}

#endif // EV3_APP_MOTORRUNNER_H