/**
 * @file PIDController.h
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 
 * @version 0.1
 * @date 2020-10-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EV3_APP_PIDCONTROLLER_H
#define EV3_APP_PIDCONTROLLER_H

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   反射光のPID係数
 * @def     Kp_reflect,Ki_reflect,Kd_reflect
 * @note    現状は Kp=3.7, Ki=0.07, Kd=1.6, 微分先行型[3.7, 0.07, 1.0]
 */
#define Kp_reflect 3.7  /* 比例パラメータ */
#define Ki_reflect 0.07 /* 積分パラメータ */
#define Kd_reflect 1.6  /* 微分パラメータ */
/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   HSVのPID係数
 * @def     Kp_hsv,Ki_hsv,Kd_hsv
 * @note    現状は Kp=0.75, Ki=0.015, Kd=0.33, 微分先行型[0.75, 0.015, 0.13]
 */
#define Kp_hsv 0.75  /* 比例パラメータ */
#define Ki_hsv 0.015 /* 積分パラメータ */
#define Kd_hsv 0.33  /* 微分パラメータ */

#define I_ARRAY_MAX 20 // 積分計算用,センサ値過去履歴回数
/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief PID計算情報の構造体
 * 
 * @struct pidStatus_t
 * @note
 */
typedef struct
{
    int actual;                      /* センサ現在値 */
    int diff;                        /* センサ値の差分 */
    int prev_diff;                   /* いっこまえのセンサ値の差分 */
    float Kp, Ki, Kd;                /* PID係数 */
    float p_value, i_value, d_value; /* PIDの各項 */
    int i_array[I_ARRAY_MAX];        /* 積分計算用,センサ値過去履歴,20回 */
    int i_index;                     /* 積分計算用,過去履歴インデックス */
} pidStatus_t;

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief PID制御 クラス
 * 
 * @class PIDController
 */
class PIDController
{
private:
    pidStatus_t pid; // PID計算情報の構造体
    float pid_value; // PID計算結果

public:
    PIDController(); // Constructor

    void calc(int target, int edge);                // PIDの計算
    float getPIDvalue();                            // PID計算結果の取得
    void setPIDactual(int actual);                  // 現在センサ値の設定
    void setPIDparam(float Kp, float Ki, float Kd); // PIDパラメータの設定
};

// ******** 以降の実装部はリリース時にcppに分離 ******** ******** ********

// Constructor
PIDController::PIDController()
    : pid({0}),
      pid_value(0)
{
}

/**
 * @brief PIDの計算
 * 
 * @fn  void PIDController::calc(int target, int edge)
 * @param target    (int)目標値
 * @param edge      (int)ライン検知左右エッジ
 * @return 無し
 * @attention HSVの場合はエッジ値を逆にすること
 */
void PIDController::calc(int target, int edge)
{
    int i_history; // 積分計算用,過去履歴移動平均
    int i;

    pid.diff = pid.actual - target; // 目標との差分

    // -------- Kp --------
    pid.p_value = pid.Kp * pid.diff;
    // -------- Ki --------
    pid.i_array[pid.i_index] = pid.diff;           // センサー履歴をひとつ更新
    pid.i_index = (pid.i_index + 1) % I_ARRAY_MAX; // 次のループ用に配列インデックス+1
    i_history = 0;                                 // 履歴を消去して
    for (i = 0; i < I_ARRAY_MAX; i++)
        i_history += pid.i_array[i]; // 履歴の合計を出す
    i_history /= I_ARRAY_MAX;        // 過去20回分移動平均
    pid.i_value = pid.Ki * i_history;
    // -------- Kd --------
    pid.d_value = pid.Kd * (pid.diff - pid.prev_diff);
    //pid.d_value = pid.Kd * pid.actual * -1;
    // -------- 差分値の保存 --------
    pid.prev_diff = pid.diff;
    // -------- PID --------
    pid_value = (pid.p_value + pid.i_value + pid.d_value) * edge;
    if (pid_value > 100)
        pid_value = 100.0;
    else if (pid_value < -100)
        pid_value = -100.0;
}

/**
 * @brief PID計算結果の取得
 * 
 * @fn float PIDController::getPIDvalue()
 * @return float pid_value: PID計算結果
 */
inline float PIDController::getPIDvalue()
{
    return this->pid_value;
}

/**
 * @brief 現在センサ値の設定
 * 
 * @fn      void PIDController::setPIDactual(int actual)
 * @param   actual (int)現在センサ値
 * @return  無し
 */
inline void PIDController::setPIDactual(int actual)
{
    this->pid.actual = actual;
}

/**
 * @brief PIDパラメータの設定
 * 
 * @fn      void PIDController::setPIDparam(float Kp, float Ki, float Kd)
 * @param   Kp  (float)PID 比例パラメータ 
 * @param   Ki  (float)PID 積分パラメータ
 * @param   Kd  (float)PID 微分パラメータ
 * @return 無し
 */
inline void PIDController::setPIDparam(float Kp, float Ki, float Kd)
{
    this->pid.Kp = Kp;
    this->pid.Ki = Ki;
    this->pid.Kd = Kd;
}

#endif // EV3_APP_PIDCONTROLLER_H