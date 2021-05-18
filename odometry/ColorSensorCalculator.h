/**
 * @file ColorSensorCalculator.h
 * @author kengo hara (kengo.hara@veriserve.co.jp)
 * @brief 
 * @version 0.1
 * @date 2020-10-20
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef EV3_APP_COLORSENSORCALCULATOR_H
#define EV3_APP_COLORSENSORCALCULATOR_H

#include "ev3api.h"
#include "etrobo_env.h"

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   HSV値(色相,彩度,明度)
 * 
 * @struct  hsv_t
 * @note    サイズは12byte= int(4byte) x3 
 */
typedef struct
{
    int hue; // hue
    int sat; // saturation
    int val; // value aka brightness
} hsv_t;

/** ******** ******** ******** ******** ******** ******** ******** ********
 * @brief   RGB値の測定とHSVの計算クラス
 * 
 * @class   ColorSensorCalculator
 */
class ColorSensorCalculator
{
private:
    rgb_raw_t rgb; // RGBの構造体
    hsv_t hsv;     // HSVの構造体

public:
    ColorSensorCalculator(); // Constructor
    void calc();             // RGBからHSVに変換
    int getHSVsat();         // saturation値を取得
    int getHSVval();         // value値を取得
};

// ******** 以降の実装部はリリース時にcppに分離 ******** ******** ********

// Constructor
ColorSensorCalculator::ColorSensorCalculator()
    : rgb({0}),
      hsv({0})
{
}

/**
 * @brief   RGBからHSVに変換
 * 
 * @fn      void ColorSensorCalculator::calc()
 * @return  なし
 */
void ColorSensorCalculator::calc()
{
    int rgb_max, rgb_min; // RGB最大値,最小値
    int r, g, b;          // RGB,int型に変換
    //int color_id;         // 識別カラーid
    //unsigned int ambient; // 環境光

    //color_id = ev3_color_sensor_get_color(color_sensor);    // 黄色しか識別しない,役に立たん
    //ambient = ev3_color_sensor_get_ambient(color_sensor);   // 環境光はふらつきまくりで役に立たん

    // RGB値取得
    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);
    r = rgb.r;
    g = rgb.g;
    b = rgb.b;

    // -------- HSVの計算 --------
    // 最大・最小値
    rgb_max = r;
    rgb_max = (g > rgb_max) ? g : rgb_max;
    rgb_max = (b > rgb_max) ? b : rgb_max;
    rgb_min = r;
    rgb_min = (g < rgb_min) ? g : rgb_min;
    rgb_min = (b < rgb_min) ? b : rgb_min;

    // value:明度,正規化する,但し100倍して単位は%
    hsv.val = 100 * rgb_max / 256;
    // saturation:彩度,正規化する,但し100倍して単位は%
    hsv.sat = 100 * (rgb_max - rgb_min) / rgb_max;
    // hue:色相
    if ((rgb_max - rgb_min) == 0) // 0除算回避
    {
        hsv.hue = 0;
    }
    else // RGBから変換,数式上正規化不要,単位はdeg
    {
        hsv.hue = (60 * (r - g)) / (rgb_max - rgb_min) + 240;
        if (rgb_max == r)
            hsv.hue = (60 * (g - b)) / (rgb_max - rgb_min);
        if (rgb_max == g)
            hsv.hue = (60 * (b - r)) / (rgb_max - rgb_min) + 120;
        if (hsv.hue < 0)
            hsv.hue += 360;
    }
}

/**
 * @brief   saturation値を取得
 * 
 * @fn      int ColorSensorCalculator::getHSVsat()
 * @return  int hsv.sat: HSV彩度
 */
inline int ColorSensorCalculator::getHSVsat()
{
    return this->hsv.sat;
}

/**
 * @brief   value値を取得
 * 
 * @fn      int ColorSensorCalculator::getHSVval()
 * @return  int hsv.val: HSV明度
 */
inline int ColorSensorCalculator::getHSVval()
{
    return this->hsv.val;
}

#endif // EV3_APP_COLORSENSORCALCULATOR_H