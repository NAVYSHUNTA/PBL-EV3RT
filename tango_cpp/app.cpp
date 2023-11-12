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

    /**
    * 操作
    */
    bool isPressed() {
        return( ev3_touch_sensor_is_pressed( touch_sensor ) );
    }
};

/**
* RunControl クラス
*/
class RunControl {
public:
    /**
    * コンストラクタ
    */
    RunControl() {
    }

    /**
    * 操作
    */
    void stop() {
        ev3_motor_stop(right_motor, true);
        ev3_motor_stop(left_motor, true);
    }

    void turn(int speed) {
        ev3_motor_set_power(right_motor, speed);
        ev3_motor_set_power(left_motor, -speed);
    }

    void forward(int speed) {
        ev3_motor_set_power(right_motor, speed);
        ev3_motor_set_power(left_motor,  speed);
    }
};

/**
* ObstacleDetection クラス
*/
class ObstacleDetection {
public:
    /**
    * 属性
    */
    int obsDistance;
    
    /**
    * 関連
    */
    
    /**
    * コンストラクタ
    */
    ObstacleDetection() {
        /* 10[cm] */
        obsDistance = 10;
    }

    /**
    * 操作
    */
    bool isDetected() {
        return( ev3_ultrasonic_sensor_get_distance(sonar_sensor) < obsDistance );
    }
};

/**
 * Tangoクラス
 */
class TangoClass {
public:

    /**
    * 状態
    */
    enum RunState {
        STATE_STOP    = 0,
        STATE_TURN    = 1,
        STATE_FORWARD = 2,
    };

    /**
    * コンストラクタ
    */
    TangoClass() {
        ev3_lcd_draw_string("Tango Class is created.", 0, 16);

        /* センサー入力ポートの設定 */
        ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
        ev3_sensor_config(color_sensor, COLOR_SENSOR);
        ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
        ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

        /* モーター出力ポートの設定 */
        ev3_motor_config(left_motor, LARGE_MOTOR);
        ev3_motor_config(right_motor, LARGE_MOTOR);

        state = STATE_STOP;
        turnSpeed = 20;
        forwardSpeed = 30;
    }

    /**
    * 操作
    */
    /* 停止状態 タッチセンサ押下待ち */
    void procStopState() {
        runControl.stop();
    
        while(1) {
            if( runButton.isPressed() ) {
                state = STATE_TURN;
                break;
            };
            
            tslp_tsk(200 * 1000U); /* 200 msec周期起動 */
        };
    }

    /* 回転状態 */
    void procTurnState() {
        while(1) {
            if( runButton.isPressed() ) {
                state = STATE_STOP;
                break;
            };
            
            if( obstacleDetection.isDetected() == false ) {
                state = STATE_FORWARD;
                break;
            };
            
            runControl.turn( turnSpeed );
            
            tslp_tsk(200 * 1000U); /* 200 msec周期起動 */
        };
        
        runControl.stop();
    }

    /* 前進状態 */
    void procForwardState() {
        while(1) {
            if( runButton.isPressed() ) {
                state = STATE_STOP;
                break;
            };
            
            if( obstacleDetection.isDetected() ) {
                state = STATE_TURN;
                break;
            };
        
            runControl.forward( forwardSpeed );
        
            tslp_tsk(200 * 1000U); /* 200 msec周期起動 */
        };
        
        runControl.stop();
    }

    void start() {
        while(1) {
            ev3_lcd_draw_string("                ", 0, 26);
            if( state == STATE_STOP ) {
                ev3_lcd_draw_string("Stop.", 0, 26);
                procStopState();
            } else if( state == STATE_TURN ) {
                ev3_lcd_draw_string("Turn.", 0, 26);
                procTurnState();
            } else if( state == STATE_FORWARD ) {
                ev3_lcd_draw_string("Forward.", 0, 26);
                procForwardState();
            } else {
                ev3_lcd_draw_string("State Invalid.", 0, 26);
            };
            
            tslp_tsk(100 * 1000U); /* 100 msec周期起動 */
        };
    }

private:
    /**
    * 属性
    */
    int state;
    int turnSpeed;
    int forwardSpeed;
  
    /**
    * 関連
    */
    Button            runButton;          /* Buttonクラス */
    RunControl        runControl;         /* RunControlクラス */
    ObstacleDetection obstacleDetection;  /* ObstacleDetectionクラス */
};


/* Tangoオブジェクト生成 */
TangoClass tango;

void main_task(intptr_t unused) {
    /* Tango start */
    tango.start();
}
