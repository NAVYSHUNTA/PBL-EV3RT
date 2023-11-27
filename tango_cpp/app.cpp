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

/*** ColorSensor*/
class ColorSensor {
public:
/*** コンストラクタ*/
    ColorSensor() {
    }
/** 反射光*/
    void reflectValue() {
        uint8_t value=ev3_color_sensor_get_reflect(color_sensor);
        char data[10];
        sprintf(data,"%d",value);

        ev3_lcd_draw_string("reflectValue",0,86);
        ev3_lcd_draw_string("reflectValue",0,96);
    }
/* RGB*/
void RGBValue() {
    rgb_raw_t rgb;
    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb);
    char dataR[10];
    char dataG[10];
    char dataB[10];
    sprintf(dataR, "%d", rgb.r);
    sprintf(dataG, "%d", rgb.g);
    sprintf(dataB, "%d", rgb.b);
    ev3_lcd_draw_string("RGBValue.", 0, 36);
    ev3_lcd_draw_string(dataR, 0, 46);
    ev3_lcd_draw_string(dataG, 0, 56);
    ev3_lcd_draw_string(dataB, 0, 66);
}

    int getBrightness() {
        return ev3_color_sensor_get_reflect(color_sensor);
    }
};


/**
* LineTrace クラス
*/

class LineTrace {
public:
    int brightness;
    int prevError;
    int integral;
    int forwardSpeed;
    int target;

    LineTrace() {
        brightness = 0;
        prevError = 0;
        integral = 0;
        target=17;
        forwardSpeed=30;
    }

    void lineTraceAction(double Kp, double Kd) {
        ColorSensor colorSensor; 
        brightness = colorSensor.getBrightness(); 

        int error = target - brightness;

        // PD制御のパラメータ
        //double Kp = 1.2; // 比例制御のゲイン
        //double Kd = 0.8; // 微分制御のゲイン

        // 偏差の変化量（微分）
        int derivative = error - prevError;
        prevError = error;

        // 偏差の積分
        integral += error;

        // PD制御に基づいてモーターのパワーを計算
        int power = static_cast<int>((Kp * error) + (Kd * derivative));

        // モーター制御
        if (power < -100) power = -100;
        if (power > 100) power = +100;

        ev3_motor_set_power(right_motor, forwardSpeed - power);
        ev3_motor_set_power(left_motor, forwardSpeed + power);
    }

    private:
    /**
    * 属性
    */
    /**
    * 関連
    */
    //ColorSensor colorSensor;
    
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
 * Nyoroクラス
 */
class NyoroClass {
public:

    /**
    * 状態
    */
    enum RunState {
        STATE_STOP    = 0,
        STATE_LINE    = 1,
        STATE_TURN    = 3,
        STATE_FORWARD = 2,
    };

    /**
    * コンストラクタ
    */
    NyoroClass() {
        ev3_lcd_draw_string("Nyoro Class is created.", 0, 16);

        /* センサー入力ポートの設定 */
        ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
        ev3_sensor_config(color_sensor, COLOR_SENSOR);
        ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
        ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

        /* モーター出力ポートの設定 */
        ev3_motor_config(left_motor, LARGE_MOTOR);
        ev3_motor_config(right_motor, LARGE_MOTOR);

        state = STATE_STOP;
        turnSpeed = 20;//20;
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
                state = STATE_LINE;
                break;
            };
            
            tslp_tsk(200 * 1000U); /* 200 msec周期起動 */
        };
    }

    /* ライントレース */
    void procLineState() {
        while(1) {
            lineTrace.lineTraceAction(0.5,2.0);
            if( runButton.isPressed() ) {
                state = STATE_STOP;
                break;
            };
            
            //if( obstacleDetection.isDetected() == false ) {
                //state = STATE_FORWARD;
                //break;
            //};
            
            //runControl.turn( turnSpeed );
            
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
            } else if( state == STATE_LINE ) {
                ev3_lcd_draw_string("Line.", 0, 26);
                procLineState();
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
    LineTrace         lineTrace;
    //ColorSensor       colorSensor;
    Button            runButton;          /* Buttonクラス */
    RunControl        runControl;         /* RunControlクラス */
    ObstacleDetection obstacleDetection;  /* ObstacleDetectionクラス */
};


/* Nyoroオブジェクト生成 */
NyoroClass nyoro;

void main_task(intptr_t unused) {
    /* Nyoro start */
    nyoro.start();
}
