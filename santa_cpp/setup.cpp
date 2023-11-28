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
const int RAPID_MODE = 1; // 爆速モード

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    color_sensor    = EV3_PORT_2,
    sonar_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B;

/**
* Button クラス
*/
class Button {
public:
    /**
    * コンストラクタ
    */
    Button() {
    }

    // 押下状態を取得する
    bool isPressed() {
        return ( ev3_touch_sensor_is_pressed( touch_sensor ) );
    }
};

/**
 * ColorSenSensor クラス
 */
class ColorSensor{
public:
    /*コンストラクタ*/
    ColorSensor() {
    }
    /*反射光*/
    void reflectValue() {
        uint8_t value=ev3_color_sensor_get_reflect(color_sensor);
        char data[10];
        sprintf(data,"%d",value);

        ev3_lcd_draw_string("reflectValue.",0,36);
        ev3_lcd_draw_string(data,0,46);
    }
    /*RGB*/
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
};

/*超音波センサ*/
class SonarSensor {
    public:
        /*コンストラクタ*/
        SonarSensor() {
        }

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
        // サンタ帽子の有無を判定する
        bool hatChecker() {
            if (sonarSensor.sonardetection() <= hatCheckDistance) {
                return RAPID_MODE; // 爆速モード
            } else {
                return NORMAL_MODE; // 通常モード
            };
        }

    private:
        /**
        * 属性
        */
        int hatCheckDistance = 5; // 被っているとみなす距離

    /**
    * 関連
    */
    SonarSensor sonarSensor;
};

/**
 * ModeSelect クラス
 */
class ModeSelectClass {
    public:
        // 選択したモードを取得する
        int getMode() {
            return hatChecker.hatChecker();
        }
        // タッチセンサが押されたことを確認する
        bool isButtonPressed() {
            Button button; // インスタンス生成
            return button.isPressed();
        }

    /**
    * 関連
    */
    HatCheckerClass hatChecker;
};

/**
 * Santa クラス
 */
class SantaClass {
    public:
        /**
        * コンストラクタ
        */
        SantaClass() {
            ev3_lcd_draw_string("Santa Class is created.", 0, 16);

            /* センサー入力ポートの設定 */
            ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
            ev3_sensor_config(color_sensor, COLOR_SENSOR);
            ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
            ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

            /* モーター出力ポートの設定 */
            ev3_motor_config(left_motor, LARGE_MOTOR);
            ev3_motor_config(right_motor, LARGE_MOTOR);

            mode = NORMAL_MODE; // 最初は通常モード
        }
        void start() {
            while(1) {
                mode = modeSelect.getMode();

                if (mode == NORMAL_MODE) {
                    ev3_lcd_draw_string("Normal_mode.", 0, 36);
                } else {
                    ev3_lcd_draw_string("Rapid_mode.", 0, 36);
                }

                // タッチセンサが押されたらループを中断
                if (modeSelect.isButtonPressed()) {
                    break;
                }
                tslp_tsk(100 * 1000U); /* 100 msec周期起動 */
            };
        }

    private:
        /**
        * 属性
        */
        int mode; // モード

    /**
    * 関連
    */
    ModeSelectClass modeSelect;
    //////////////////// あとでクラス図をもとに書く。/////////////////////////////////////////

    // 例
    // Button            runButton;          /* Buttonクラス */
    // RunControl        runControl;         /* RunControlクラス */
    // ObstacleDetection obstacleDetection;  /* ObstacleDetectionクラス */
    // SonarSensor       sonrsensor;         /* SonarSesorクラス */
};

/* Santaオブジェクト生成 */
SantaClass santa;

void main_task(intptr_t unused) {
    /* Santa start */
    santa.start();
}
