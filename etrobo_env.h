/**
 * @file etrobo_env.h
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 
 * @version 0.1
 * @date 2020-10-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef EV3_APP_ETROBOENV_H_
#define EV3_APP_ETROBOENV_H_

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#if defined(DEBUG)
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#include "ev3api.h"

/**
 * シミュレータかどうかの定数を定義します
 */
#if defined(MAKE_SIM)
static const int _SIM = 1;
#elif defined(MAKE_EV3)
static const int _SIM = 0;
#else
static const int _SIM = 0;
#endif

/**
 * 左コース/右コース向けの設定を定義します
 * デフォルトは左コース(ラインの右エッジをトレース)です
 */
#if defined(MAKE_RIGHT)
static const int _LEFT = 0;
#define _EDGE -1
#else
static const int _LEFT = 1;
#define _EDGE 1
#endif

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor = EV3_PORT_1,
    color_sensor = EV3_PORT_2,
    sonar_sensor = EV3_PORT_3,
    gyro_sensor = EV3_PORT_4;

static const motor_port_t
    left_motor = EV3_PORT_C,
    right_motor = EV3_PORT_B,
    arm_motor = EV3_PORT_A,
    tail_motor = EV3_PORT_D;

// 下記のマクロは個体/環境に合わせて変更する必要があります
// sample_c1マクロ
// #define LIGHT_WHITE 40 // 白色の光センサ値
// #define LIGHT_BLACK 0  // 黒色の光センサ値
// sample_c2マクロ
#define SONAR_ALERT_DISTANCE 13 // 超音波センサによる障害物検知距離[cm]
// sample_c4マクロ
//#define DEVICE_NAME     "ET0"  // Bluetooth名 sdcard:\ev3rt\etc\rc.conf.ini LocalNameで設定
//#define PASS_KEY        "1234" // パスキー    sdcard:\ev3rt\etc\rc.conf.ini PinCodeで設定
#define CMD_START '1' // リモートスタートコマンド

// LCDフォントサイズ
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6 /*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8 /*TODO: magic number*/)

#endif // EV3_APP_ETROBOENV_H_