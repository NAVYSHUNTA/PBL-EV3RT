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

/**
 * 走行するエッジを指定する
 */
const int LEFT_EDGE  = false; // 左エッジ
const int RIGHT_EDGE = true;  // 右エッジ

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
        // 輝度値を取得する
        int getBrightness() {
            return ev3_color_sensor_get_reflect(color_sensor);
        }

        // 色に対応する値を取得する
        int getColorValue() {
            return static_cast<int>(ev3_color_sensor_get_color(color_sensor));
        }

        // 青色かどうかを判定する
        bool isBlue() {
            rgb_raw_t rgb;
            ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);
            if ((rgb.r >= 0 && rgb.r <= 30) && (rgb.g >= 20 && rgb.g <= 50) && (rgb.b >= 50 && rgb.b <= 100)) {
                return true;
            } else {
                return false;
            }
        }

        // 黄色かどうかを判定する
        bool isYellow() {
            rgb_raw_t rgb;
            ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);
            if ((rgb.r >= 80 && rgb.r <= 120) && (rgb.g >= 80 && rgb.g <= 120) && (rgb.b >= 0 && rgb.b <= 30)) {
                return true;
            } else {
                return false;
            }
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
            stop();
        }

        // 走行距離をリセットする
        void resetDistance() {
            ev3_motor_config(right_motor, LARGE_MOTOR);
            ev3_motor_reset_counts(right_motor);
        }

        // 走行距離を取得する
        long long getDistance() {
            return static_cast<long long>(ev3_motor_get_counts(right_motor) * 31.4 / 360);
        }

        // 一定距離の直線走行を行う
        void forwardDistance(int distance, int power = 20) {
            resetDistance(); // 走行距離をリセット
            forward(power, power);
            while (getDistance() <= distance) {
                tslp_tsk(30 * 1000U);
            }
            stop();
            tslp_tsk(30 * 1000U);
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
        void lineTraceAction(int brightness, double Kp, double Kd, int forwardSpeed = 20, int target = 20, bool edge = RIGHT_EDGE) {
            // PD制御の計算
            int error = target - brightness;
            int derivative = error - prevError; // 偏差の変化量（微分）
            prevError = error;

            // モーター制御
            int power = static_cast<int>((Kp * error) + (Kd * derivative));
            power = min(max(power, -100), 100);

            if (RIGHT_EDGE) {
                // 右エッジのとき
                ev3_motor_set_power(left_motor, forwardSpeed + power);
                ev3_motor_set_power(right_motor, forwardSpeed - power);
            } else {
                // 左エッジのとき
                ev3_motor_set_power(left_motor, forwardSpeed - power);
                ev3_motor_set_power(right_motor, forwardSpeed + power);
            }
        }

        // 走行距離をリセットする
        void resetDistance() {
            ev3_motor_config(right_motor, LARGE_MOTOR);
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
                normalModeAction(); // 通常モード
            } else {
                rapidModeAction();  // 爆速モード
            }

            // サンタ走行
            // 緑色サークルから補助線の終端（灰色サークル）までの走行
            goToGrayCircleAction();

            // フリーエリア内の走行（時計回りで走行）
            moveClockwiseAction();

            // フリーエリア内の走行（牛耕式で走行）
            moveZigzagAction();

            // プレゼント投下
            giftDrop.dropAction();

            // 停止
            while (1) {
                runControl.stop();
                tslp_tsk(30 * 1000U);
            }
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
            // ライントレース（カーブに強い）
            linetrace.resetDistance();
            tslp_tsk(30 * 1000U);
            while(1) {
                // 一定距離、色の識別を行わない
                if (linetrace.getDistance() > 500) {
                    break;
                }
                // ライントレースして走行する
                linetrace.lineTraceAction(colorSensor.getBrightness(), 1.1, 0.8, 17);
                tslp_tsk(30 * 1000U);
            }

            // ライントレース（カーブに弱いがまっすぐ走る）
            linetrace.resetDistance();
            runControl.stop();
            tslp_tsk(30 * 1000U);
            while(1) {
                // 一定距離、色の識別を行わない
                if (linetrace.getDistance() > 80) {
                    break;
                }
                // ライントレースして走行する
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.8, 0.5, 15);
                tslp_tsk(30 * 1000U);
            }

            // ライントレース（色の識別を開始する）
            runControl.stop();
            tslp_tsk(30 * 1000U);
            while(1) {
                if (colorSensor.getColorValue() == COLOR_GREEN) {
                    break;
                }
                // ライントレースして走行する
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.8, 0.5, 15);
                tslp_tsk(30 * 1000U);
            }
        }

        // 爆速モードで走行する
        void rapidModeAction() {
            // 一定距離の直線走行を行う
            while(1) {
                // 黒線を見つけるまで直線走行する
                if (runControl.getDistance() > 20) {
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 近道するために回転し、直線走行する
            linetrace.resetDistance();
            runControl.stop();
            runControl.rotate(right_motor, 195, 100); // 回転
            tslp_tsk(30 * 1000U);

            // 近道（高速で）
            runControl.forwardDistance(250, 40);

            // 近道（低速で）
            runControl.stop();
            tslp_tsk(100 * 1000U);
            runControl.forward(5, 5);
            while(1) {
                // 黒線を見つけるまで直線走行する
                if (colorSensor.getColorValue() == COLOR_BLACK) {
                    break;
                }
                tslp_tsk(30 * 1000U);
            }

            // 黒線上をライントレースするために回転し、再度ライントレースする
            runControl.rotate(left_motor, 210, 100);
            linetrace.resetDistance();
            tslp_tsk(30 * 1000U);

            while(1) {
                if (linetrace.getDistance() > 150) {
                    break;
                }
                // ライントレースして走行する
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.8, 0.5, 15);
                tslp_tsk(30 * 1000U);
            }

            // ライントレース（色の識別を開始する）
            runControl.stop();
            tslp_tsk(30 * 1000U);
            while(1) {
                if (colorSensor.getColorValue() == COLOR_GREEN) {
                    break;
                }
                // ライントレースして走行する
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.8, 0.5, 15);
                tslp_tsk(30 * 1000U);
            }
        }

        // 緑色サークルから補助線の終端（灰色サークル）までの走行
        void goToGrayCircleAction() {
            // 緑色サークル上での90度左回転
            runControl.stop();
            tslp_tsk(300 * 1000U);
            runControl.rotate(left_motor, 310, 40);
            tslp_tsk(300 * 1000U);

            // 赤色サークルまで走行する
            while (1) {
                if (colorSensor.getColorValue() == COLOR_RED) {
                    runControl.stop();
                    runControl.forwardDistance(10);
                    tslp_tsk(300 * 1000U);
                    runControl.rotate(left_motor, 30, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 補助線を通ってフリーエリア内へ侵入する
            while (1) {
                if (linetrace.getDistance() >= 40) {
                    runControl.stop();
                    runControl.rotate(right_motor, 20, 30);
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(12, 10);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }
        }

        // 時計回りで走行
        void moveClockwiseAction() {
            // 青色（左上）へ向かう
            while (1) {
                if (colorSensor.isBlue()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(3, 10);
                    runControl.rotate(left_motor, 63, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 灰色（上）へ向かう
            linetrace.resetDistance();
            while (1) {
                if (linetrace.getDistance() >= 35) {
                    runControl.stop();
                    runControl.forwardDistance(9, 10);
                    runControl.rotate(left_motor, 65, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 赤色（右上）へ向かう
            while (1) {
                if (colorSensor.getColorValue() == COLOR_RED) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(3);
                    runControl.rotate(left_motor, 56, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 灰色（右）へ向かう
            linetrace.resetDistance();
            while (1) {
                if (linetrace.getDistance() >= 35) {
                    runControl.stop();
                    runControl.forwardDistance(9);
                    tslp_tsk(300 * 1000U);
                    runControl.rotate(left_motor, 64, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 黄色（右下）へ向かう
            while (1) {
                if (colorSensor.isYellow()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(3, 15);
                    runControl.rotate(left_motor, 63, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 灰色（下）へ向かう
            linetrace.resetDistance();
            while (1) {
                if (linetrace.getDistance() >= 35) {
                    runControl.stop();
                    runControl.forwardDistance(9);
                    tslp_tsk(300 * 1000U);
                    runControl.rotate(left_motor, 64, 30);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }
        }

        // 牛耕式で走行
        void moveZigzagAction() {
            // 緑色（左下）で回転する
            while (1) {
                if (colorSensor.getColorValue() == COLOR_GREEN) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(15);
                    tslp_tsk(300 * 1000U);
                    runControl.rotate(left_motor, 790, 40);
                    tslp_tsk(300 * 1000U);
                    runControl.rotate(right_motor, 320, 40);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 緑色（右下）へ向かう
            while (1){
                if (colorSensor.getColorValue() == COLOR_GREEN) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 黄色（左下）へ向かう
            while (1) {
                if (colorSensor.isYellow()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 黄色（右下）へ向かう
            while (1) {
                if (colorSensor.isYellow()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 黄色（右下）で回転する
            runControl.rotate(right_motor, 300, 40);
            tslp_tsk(300 * 1000U);

            // 黄色（右上）へ向かう
            while (1) {
                if (colorSensor.isYellow()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 黄色（右上）で回転する
            runControl.rotate(right_motor, 300, 30);
            tslp_tsk(500 * 1000U);

            // 黄色（左上）へ向かう
            while (1) {
                if (colorSensor.isYellow()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(5, 15);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 緑色（右上）へ向かう
            while (1) {
                if (colorSensor.getColorValue() == COLOR_GREEN) {
                    runControl.stop();
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 緑色（左上）へ向かう
            while (1) {
                if (colorSensor.getColorValue() == COLOR_GREEN) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 緑色（左上）で回転する
            runControl.rotate(left_motor, 290, 30);
            tslp_tsk(500 * 1000U);

            // 青色（左下）へ向かう
            while (1) {
                if (colorSensor.isBlue()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 青色（左下）で回転する
            runControl.rotate(left_motor, 270, 30);
            tslp_tsk(500 * 1000U);

            // 青色（右下）へ向かう
            while (1) {
                if (colorSensor.isBlue()) {
                    runControl.stop();
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 赤色（左下）へ向かう
            while (1) {
                if (colorSensor.getColorValue() == COLOR_RED) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 赤色（右下）へ向かう
            while (1) {
                if (colorSensor.getColorValue() == COLOR_RED) {
                    runControl.stop();
                    tslp_tsk(500 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15, LEFT_EDGE);
                tslp_tsk(30 * 1000U);
            }

            // 赤色（右下）で回転する
            runControl.rotate(right_motor, 310, 30);
            tslp_tsk(500 * 1000U);

            // 赤色（右上）へ向かう
            linetrace.resetDistance();
            while (1) {
                if (linetrace.getDistance() >= 12) {
                    runControl.stop();
                    tslp_tsk(500 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }
            while (1) {
                runControl.forward(10, 10);
                if (colorSensor.getColorValue() == COLOR_RED) {
                    runControl.stop();
                    tslp_tsk(500 * 1000U);
                    break;
                }
                tslp_tsk(30 * 1000U);
            }

            // 赤色（右上）で回転する
            runControl.rotate(right_motor, 240, 30);
            tslp_tsk(500 * 1000U);

            // 赤色（左上）へ向かう
            while (1) {
                if (colorSensor.getColorValue() == COLOR_RED) {
                    runControl.stop();
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // 青色（右上）へ向かう
            while (1) {
                if (colorSensor.isBlue()) {
                    runControl.stop();
                    runControl.forwardDistance(5, 15);
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }

            // GOAL2
            // 青色（左上）へ向かう
            while (1) {
                if (colorSensor.isBlue()) {
                    runControl.stop();
                    tslp_tsk(300 * 1000U);
                    break;
                }
                linetrace.lineTraceAction(colorSensor.getBrightness(), 0.6, 0.1, 10, 15);
                tslp_tsk(30 * 1000U);
            }
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
