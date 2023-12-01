/**
 * This sample program balances a two-wheeled Segway type robot such as Gyroboy in EV3 core set.
 *
 * References:
 * http://www.hitechnic.com/blog/gyro-sensor/htway/
 * http://www.cs.bgu.ac.il/~ami/teaching/Lejos-2013/classes/src/lejos/robotics/navigation/Segoway.java
 */

#include "ev3api.h"
#include "app.h"

#include "libcpp-test.h"

#include <algorithm>
using namespace std;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * 数値の代わりに変数名でモードを指定する
 */
const int NORMAL_MODE = 0; // 通常モード
const int RAPID_MODE  = 1; // 爆速モード

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    arm_motor       = EV3_PORT_A,
    right_motor     = EV3_PORT_B,
    left_motor      = EV3_PORT_C;

/**
 * バックライトの色を指定する
 */
static const ledcolor_t
    green_light     = LED_GREEN,
    orange_light    = LED_ORANGE;

/////////////////////////////////////////削除するかも（ここから↓）///////////////////////////////
/**
 * カラーセンサの色に対応する値を指定する
 */
// const int COLOR_NONE_   = 0;
// const int COLOR_BLACK_  = 1;
// const int COLOR_BLUE_   = 2;
// const int COLOR_GREEN_  = 3;
// const int COLOR_YELLOW_ = 4;
// const int COLOR_RED_    = 5;
// const int COLOR_WHITE_  = 6;
// const int COLOR_BROWN_  = 7;
/////////////////////////////////////////削除するかも（ここまで↑）///////////////////////////////



/**
* Button クラス
*/
class Button {
    public:
        // コンストラクタ
        Button() {
        }

        // 操作
        // 押下状態を取得する
        bool isPressed() {
            return ( ev3_touch_sensor_is_pressed( touch_sensor ) );
        }
};

/**
 * Backlight クラス
 */
class Backlight {
    public:
        // コンストラクタ
        Backlight() {
        }

        // 操作
        // 緑色に変更
        void setGreenLED() {
            ev3_led_set_color(green_light);
        }

        // オレンジ色に変更
        void setOrangeLED() {
            ev3_led_set_color(orange_light);
        }
};

/**
 * SonarSensor クラス
 */
class SonarSensor {
    public:
        // コンストラクタ
        SonarSensor() {
        }

        // 操作
        // 距離を取得する
        int sonardetection() {
            return ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        }
};

/**
 * HatChecker クラス
 */
class HatCheckerClass {
    public:
        // コンストラクタ
        HatCheckerClass() {
        }

        // 操作
        // サンタ帽子の有無を判定する
        bool hatChecker() {
            if (sonarSensor.sonardetection() > hatCheckDistance) {
                // 帽子がないとき
                return false;
            } else {
                // 帽子があるとき
                return true;
            }
        }
    private:
        // 属性
        int hatCheckDistance = 5; // 被っているとみなす距離
    // 関連
    SonarSensor sonarSensor;
};

/**
 * ModeSelect クラス
 */
class ModeSelectClass {
    public:
        // コンストラクタ
        ModeSelectClass() {
        }

        // 操作
        // 選択したモードを取得する
        int getMode() {
            // サンタ帽子の有無を判定する
            if (!hatChecker.hatChecker()) {
                // 帽子がないとき
                backlight.setGreenLED(); // 緑色に変更
                return NORMAL_MODE; // 通常モード
            } else {
                // 帽子があるとき
                backlight.setOrangeLED(); // オレンジ色に変更
                return RAPID_MODE; // 爆速モード
            }
        }

        // タッチセンサが押されたことを確認する
        bool isButtonPressed() {
            return button.isPressed();
        }
    // 関連
    HatCheckerClass hatChecker;
    Backlight       backlight;
    Button          button;
};

/**
 * ColorSensor クラス
 */
class ColorSensor {
    public:
        // コンストラクタ
        ColorSensor() {
        }

        // 操作
        // 反射光
        void reflectValue() {
            uint8_t value=ev3_color_sensor_get_reflect(color_sensor);
            char data[10];
            sprintf(data,"%d",value);

            ev3_lcd_draw_string("reflectValue.",0,36);
            ev3_lcd_draw_string(data,0,46);
        }

        // RGB
        void RGBValue() {
            rgb_raw_t rgb;
            ev3_color_sensor_get_rgb_raw(color_sensor,&rgb);
            char dataR[10];
            char dataG[10];
            char dataB[10];
            sprintf(dataR,"%d",rgb.r);
            sprintf(dataG,"%d",rgb.g);
            sprintf(dataB,"%d",rgb.b);

            ev3_lcd_draw_string("RGBValue.",0,36);
            ev3_lcd_draw_string(dataR,0,46);
            ev3_lcd_draw_string(dataG,0,56);
            ev3_lcd_draw_string(dataB,0,66);
        }

        // 輝度値を取得する
        int getBrightness() {
            return ev3_color_sensor_get_reflect(color_sensor);
        }

        // 色に対応する値を取得する
        int getColorValue() {
            return static_cast<int>(ev3_color_sensor_get_color(color_sensor));
        }
};

/**
 * GiftDrop クラス
 */
class GiftDrop {
    public:
        // コンストラクタ
        GiftDrop() {
        }

        // 操作
        // アームを2回上下運動させる
        void dropAction() {
            /* TODO */
            int degrees = 30;
            uint32_t speed_abs = 4;
            ev3_motor_rotate(arm_motor, degrees, speed_abs, true);
            tslp_tsk(70 * 1000U);
            ev3_motor_rotate(arm_motor, -degrees, speed_abs, true);
            tslp_tsk(70 * 1000U);
            ev3_motor_rotate(arm_motor, degrees, speed_abs, true);
            tslp_tsk(70 * 1000U);
            ev3_motor_rotate(arm_motor, -degrees, speed_abs, true);
        }
};

/**
 * RunControl
 */
class RunControl {
    public:
        // コンストラクタ
        RunControl() {
        }

        // 操作
        // 停止する
        void stop() {
            ev3_motor_stop(right_motor, true);
            ev3_motor_stop(left_motor, true);
        }

        // 前進する
        void forward(int leftSpeed, int rightSpeed) {
            ev3_motor_set_power(right_motor, leftSpeed);
            ev3_motor_set_power(left_motor, rightSpeed);
        }

        // 回転する
        void rotate(motor_port_t motor_port, int degrees, uint32_t speed_abs) {
            ev3_motor_rotate(motor_port, degrees, speed_abs, true);
        }

        // ステアリングする
        void steer(int power, int turn_ratio) {
            ev3_motor_steer(left_motor, right_motor, power, turn_ratio);
        }

        // 走行距離をリセットする
        void resetDistance() {
            ev3_motor_reset_counts(right_motor);
        }

        // 走行距離を取得する
        long long getDistance() {
            return static_cast<long long>(ev3_motor_get_counts(right_motor) * 31.4 / 360);
        }
};

/**
 * LineTrace クラス
 */
class LineTraceClass {
    public:
        // コンストラクタ
        LineTraceClass() {
            prevError = 0; // 誤差の合計値
        }

        // 操作
        // ライントレースして走行する
        void lineTraceAction(int brightness, double Kp, double Kd, int target = 20) {
            // PD制御の計算
            int error = target - brightness;
            int derivative = error - prevError; // 偏差の変化量（微分）
            prevError = error;

            // モーター制御
            int forwardSpeed = 20; // スピード
            int power = static_cast<int>((Kp * error) + (Kd * derivative));
            power = min(max(power, -100), 100);
            ev3_motor_set_power(left_motor, forwardSpeed + power);
            ev3_motor_set_power(right_motor, forwardSpeed - power);
        }

        // 走行距離をリセットする
        void resetDistance() {
            ev3_motor_reset_counts(right_motor);
        }

        // 走行距離を取得する
        long long getDistance() {
            return static_cast<long long>(ev3_motor_get_counts(right_motor) * 31.4 / 360);
        }
    private:
        // 属性
        int prevError; // 誤差の合計値
};

/**
 * NyoroSanta クラス
 */
class NyoroSantaClass {
    public:
        // コンストラクタ
        NyoroSantaClass() {
            ev3_lcd_draw_string("Santa Class is created.", 0, 16);

            // センサー入力ポートの設定
            ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
            ev3_sensor_config(color_sensor, COLOR_SENSOR);
            ev3_color_sensor_get_reflect(color_sensor); // 反射率モード
            ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

            // モーター出力ポートの設定
            ev3_motor_config(arm_motor, LARGE_MOTOR);
            ev3_motor_config(right_motor, LARGE_MOTOR);
            ev3_motor_config(left_motor, LARGE_MOTOR);

            mode = NORMAL_MODE; // 初期状態は通常モード
        }

        // 操作
        // 動作を実行する
        void start() {
            // 走行モード選択
            modeSelectAction();
            // 現在の走行モードの内容ごとに分岐する
            if (mode == NORMAL_MODE) {
                // 通常モード
                normalModeAction();
            } else {
                // 爆速モード
                rapidModeAction();
            }

            // サンタ走行（メソッドにする）
            ; /* TODO */

            // プレゼント投下（メソッドにはしない）
            // giftDrop.dropAction();

            // ここからは実験用
            // runControl.rotate(left_motor, 272, 100); // 90度回転
            // runControl.steer(20, 30); // 片方のモータのパワーを30%減らしてステアリング
            // tslp_tsk(1000 * 1000U);

        }

        // 走行体の走行モードを選択する
        void modeSelectAction() {
            while(1) {
                // 選択したモードを取得する
                mode = modeSelect.getMode();

                // LCDへの表示によるチェック用のコード
                if (mode == NORMAL_MODE) {
                    ev3_lcd_draw_string("Normal_mode.", 0, 36);
                } else {
                    ev3_lcd_draw_string("Rapid_mode.", 0, 36);
                }

                // タッチセンサが押されたことを確認する
                if (modeSelect.isButtonPressed()) {
                    tslp_tsk(500 * 1000U);
                    break;
                }
                tslp_tsk(100 * 1000U);
            }
        }

        // 通常モードで走行する
        void normalModeAction() {
            while(1) {
                /* TODO */
                if (button.isPressed() || colorSensor.getColorValue() == COLOR_GREEN) {
                    break; // ここでは検証のため、ボタンを押してbreakしているが、本番では青色を検知したらbreakするようにする。
                };
                // ライントレースして走行する
                linetrace.lineTraceAction(colorSensor.getBrightness(), 1.1, 0.8, 17);
                // linetrace.lineTraceAction(colorSensor.getBrightness(), 0.9, 0.4, 15);
                tslp_tsk(30 * 1000U);
            }

            while(1) {
                // 停止（本番ではこの行にあるコードは使わない。検証のためのコード。）
                runControl.stop();
            }
        }

        // 爆速モードで走行する
        void rapidModeAction() {
            ; /* TODO */
        }
    private:
        // 属性
        int mode; // モード
    // 関連
    ModeSelectClass modeSelect;
    LineTraceClass  linetrace;
    ColorSensor     colorSensor;
    Button          button;
    RunControl      runControl;
    GiftDrop        giftDrop;
};



// Santaオブジェクト生成
NyoroSantaClass nyoroSanta;

void main_task(intptr_t unused) {
    // 動作を実行する
    nyoroSanta.start();
}
