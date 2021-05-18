/**
 * @file app.cpp
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 二輪差動型ライントレースロボットのTOPPERS/HRP3用CPPサンプルプログラム
 * @version 0.1
 * @date 2020-10-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "ev3api.h"
#include "etroboc_ext.h"
#include "app.h"
#include "etrobo_env.h"

#include "odometry/TurnAngleCalculator.h"
#include "control/LineTracer.h"

// デストラクタ問題の回避
// https://github.com/ETrobocon/etroboEV3/wiki/problem_and_coping
void *__dso_handle = 0;

// Bluetooth設定
#if defined(MAKE_BT_DISABLE)
static const int _bt_enabled = 0;
#else
static const int _bt_enabled = 1;
#endif

static int bt_cmd = 0;  // Bluetoothコマンド 1:リモートスタート
static FILE *bt = NULL; // Bluetoothファイルハンドル

// クラスオブジェクトの定義
static ColorSensorCalculator *gColorSensorCalculator; // ColorSensorCalculatorクラス
static TurnAngleCalculator *gTurnAngleCalculator;     // TurnAngleCalculatorクラス
static PIDController *gPIDreflect;                    // PIDControllerクラス, HSV明度のPID制御
static PIDController *gPIDhsv;                        // PIDControllerクラス, HSV彩度のPID制御
static MotorRunner *gMainMotor;                       // MotorRunnerクラス, メインモーター制御
static LineTracer *gLineTracer;                       // LineTracerクラス

// 構造体の定義
static turnangle_t st_angle = {0}; // 車両回転角情報の構造体

#define MAIN_CYCLE 4                // メインサイクル周期[ms]
static unsigned int COUNT_time = 0; // 開始からの経過時間[ms]
static int DrivingStage = 0;        // 区間判定モード兼走行モード
static int distance;                // 障害物との距離[cm]
static int arm_deg;                 // アーム角
#define ARM_SPEED 20                // アームのモーター速度
#define ARM_ZERO -53                // アームのゼロ点角度
#define ARM_SWINGUP 40              // アームの振り上げ最大角
#define ARM_SWINGBACK -70           // アームの後方振り最大角
static int gyro_deg;                // ジャイロ角

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   オブジェクトの初期化
 * @fn      void user_system_create()
 */
static void user_system_create()
{
    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(arm_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, MEDIUM_MOTOR);

    // クラスオブジェクトの作成
    gColorSensorCalculator = new ColorSensorCalculator();
    gTurnAngleCalculator = new TurnAngleCalculator();
    gPIDreflect = new PIDController();
    gPIDhsv = new PIDController();
    gMainMotor = new MotorRunner();
    gLineTracer = new LineTracer();

    gPIDreflect->setPIDparam(Kp_reflect, Ki_reflect, Kd_reflect);
    gPIDhsv->setPIDparam(Kp_hsv, Ki_hsv, Kd_hsv);

    // swingarm
    ev3_motor_reset_counts(arm_motor);
    arm_deg = ev3_motor_get_counts(arm_motor);
    ev3_gyro_sensor_reset(gyro_sensor);
    gyro_deg = ev3_gyro_sensor_get_angle(gyro_sensor);
}
/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   オブジェクトの廃棄
 * @fn      void user_system_destroy()
 */
static void user_system_destroy()
{
    gMainMotor->reset();
    gMainMotor->stop();

    delete gLineTracer;
    delete gMainMotor;
    delete gPIDreflect;
    delete gPIDhsv;
    delete gTurnAngleCalculator;
    delete gColorSensorCalculator;

    if (_bt_enabled)
    {
        ter_tsk(BT_TASK);
        fclose(bt);
    }
}

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   データロガー
 * @fn      void datalogging()
 */
void datalogging()
{
    int turn;
    int drivin;
    int hsv_val;

    drivin = st_angle.MODE_straight * 100;         // 直進コーナー判定:ログ見づらいので適当にN*100倍する
    turn = gLineTracer->getTurnRatio();            // 舵角取得
    hsv_val = gColorSensorCalculator->getHSVval(); // 現在センサ値取得

    // 各種変数のログ出力
    fwrite(&COUNT_time, sizeof(COUNT_time), 1, bt);         // unsigned int,'I', 4byte
    fwrite(&drivin, sizeof(drivin), 1, bt);                 // int 'i', 4byte
    fwrite(&turn, sizeof(turn), 1, bt);                     // int 'i', 4byte
    fwrite(&st_angle.omega, sizeof(st_angle.omega), 1, bt); // int 'i', 4byte
    fwrite(&hsv_val, sizeof(hsv_val), 1, bt);               // int 'i', 4byte
    fwrite(&distance, sizeof(distance), 1, bt);             // int 'i', 4byte
    fwrite(&gyro_deg, sizeof(gyro_deg), 1, bt);             // int 'i', 4byte

    // ログ計算用ループカウンタ
    COUNT_time += MAIN_CYCLE;
}

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief 障害物検知
 * @fn      int ObstacleCalc()
 * @return true : 障害物を検知, false : 未検知
 */
static int ObstacleCalc()
{
    // 障害物検知
    if (COUNT_time % 40 == 0) // 約40msec周期毎に障害物検知
    {
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
            return true;
        else
            return false;
    }
    return false;
}

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   アームを動かす
 * @fn      int SwingArm(int power, int degree)
 * @param   power   (int)アームのモーター速度[deg]
 * @param   degree  (int)アームの角度[deg]
 * @return  true: 引数degreeに到達, false: 引数degreeに未到達 
 */
static int SwingArm(int power, int degree)
{
    arm_deg = ev3_motor_get_counts(arm_motor);

    ev3_motor_set_power(arm_motor, power);

    if (arm_deg == degree)
        return true;
    else
        return false;
}

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   周期タスク
 * @fn      void tracer_task(intptr_t exinf)
 * @note    app.cfgで設定した周期で呼び出される
 */
void tracer_task(intptr_t exinf)
{
    if (ev3_button_is_pressed(BACK_BUTTON)) // バックボタン押下
        wup_tsk(MAIN_TASK);

    //カラーセンサー取得＆計算
    gColorSensorCalculator->calc();
    //車両姿勢取得
    gTurnAngleCalculator->calc(&st_angle, gLineTracer->getTurnRatio());
    gyro_deg = ev3_gyro_sensor_get_angle(gyro_sensor);

    //状態遷移
    switch (DrivingStage)
    {
    case 0:
        // 走行
        gLineTracer->run(gPIDreflect, gPIDhsv, gColorSensorCalculator, gMainMotor);

        //障害物検知
        if (ObstacleCalc())
        {
            gMainMotor->stop();
            DrivingStage = 101;
        }
        break;

    case 101: // 段差を上る為にアームを上げる
        gMainMotor->stop();
        if (SwingArm(ARM_SPEED, ARM_SWINGUP))
            DrivingStage = 102;
        break;

    case 102: // 段差を上がる
        gMainMotor->run(30, 0);
        if (st_angle.leftWheel_deg >= 480)
            DrivingStage = 103;
        break;

    case 103: // 止まってアームを下げる
        gMainMotor->stop();
        if (SwingArm(-1 * ARM_SPEED, ARM_ZERO))
        {
            ev3_motor_stop(arm_motor, false);
            DrivingStage = 999;
        }
        break;

    case 999:
        wup_tsk(MAIN_TASK);
        break;
    default:
        break;
    }

    // ロギング
    datalogging();

    ext_tsk();
}

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   メインタスク
 * @fn      void main_task(intptr_t unused)
 * @note    最優先で最初に実行される
 */
void main_task(intptr_t unused)
{
    user_system_create(); // センサやモータの初期化処理

    if (_bt_enabled)
    {
        /* Open Bluetooth file */
        bt = ev3_serial_open_file(EV3_SERIAL_BT);
        assert(bt != NULL);

        /* Bluetooth通信タスクの起動 */
        act_tsk(BT_TASK);
    }

    /* スタート待機 */
    while (1)
    {
        if (bt_cmd == 1)
            break; /* リモートスタート */
        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
            break; /* タッチセンサが押された */

        tslp_tsk(10 * 1000U); /* 10secウェイト */
    }

    // 周期ハンドラ開始
    sta_cyc(TRACER_CYC);

    slp_tsk(); // バックボタンが押されるまで待つ
    // while(!ev3_button_is_pressed(BACK_BUTTON))
    //     tslp_tsk(50 * 1000U); /* 50msec周期起動 */

    // 周期ハンドラ停止
    stp_cyc(TRACER_CYC);

    user_system_destroy(); // 終了処理

    ETRoboc_notifyCompletedToSimulator(); // 競技終了通知

    ext_tsk();
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while (1)
    {
        if (_bt_enabled)
        {
            uint8_t c = fgetc(bt); /* 受信 */
            switch (c)
            {
            case '1':
                bt_cmd = 1;
                break;
            default:
                break;
            }
            fputc(c, bt); /* エコーバック */
        }
    }
}
