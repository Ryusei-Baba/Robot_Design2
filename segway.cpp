// 台車の倒立制御（PID制御/状態フィードバック）
// PID control / State Feedback control

#include "mbed.h"
#include "adrobo.h"
#include "Motor.h"
#include "QEI.h"

#define THETA_REF    0             //THETA_REF: 目標となる角度,振子の目標値(rad表記)
#define ZERO_ADV    374           //ZERO_ADV:棒の角度が0になる時のAD値（機体により異なります）
#define ADV_TO_RAD      0.00448857  // ADV_TO_RAD: AD値を実際の角度radに変換する係数
#define PULSE_TO_METER  0.0005063  // PULSE_TO_METER: エンコーダパルス数を台車の位置に変換する係数
#define MAX_V   7.2                //MAX_V :駆動系の最大電圧
#define T  0.1                     // T: サンプリング時間
#define KP  100                   // KP: Pゲイン
#define KI  200                 // KI: Iゲイン
#define KD  0.3                    // KD: Dゲイン
#define K1   -0.0469             // K1-K4: 状態フィードバックゲイン（1x4のゲイン）
#define K2   -0.5970
#define K3  -1.6444 
#define K4  -0.2112







BusOut led(D2,D4,D5,D7,D8);     //基板LED用IO設定
AnalogIn pen(A0);               //ポテンショメータ用IO設定
Ticker pen_control;             //台車の制御用タイマー割り込み
Serial pc(USBTX, USBRX);        //デバッグ用シリアル通信

//モータ制御用オブジェクト
Motor motor_left(MOTOR11, MOTOR12);     //左モータ
Motor motor_right(MOTOR21, MOTOR22);    //右モータ

//***************　台車の制御　ここから　***************//
int theta_adv, adv;     
int left, right;                                        //左車輪　,　右車輪
double theta, e, e0, ed, ei, x, x0, dx, dtheta, theta0;    
double v_ref, duty_ratio;                               //電圧指令値　，　デューティー比

void pen_control_handler(){
    theta_adv = pen.read_u16()>>6;                      //ADCを通してポテンショメータのAD値を取得
    adv = theta_adv - ZERO_ADV;                         //AD値（測定値ー角度0のAD値）の宣言
                                                        //搭載されているLPC1114のADCは10bitのため6bit右にシフト
    
    theta = (double)adv * ADV_TO_RAD;                   // AD値を実際の角度radで表したもの。振子が地面と垂直の時からの傾きの角度。
    e = THETA_REF - theta;                              //目標となる角度と現在の角度のズレを表している。
    ed = (e - e0) / T;                                  //角度の変位をTで微分しているので振子の角速度を表している。
    ei += e * T;                                        //振子が動いた角度の和を表している。
    e0 = e;                                             //eをここで保存している。edを求めるときに使う。
    
//  頭打ち処理
    if(ei > 10000) ei = 10000;
    if(ei < -10000) ei = -10000;
    
//  Calculate PID control
    v_ref = (e * KP + ei * KI + ed * KD);               //電圧指令値を表している。
    
//  Introduce x, dx, theta, dtheta
    x = (double)(left + right) / 2 * PULSE_TO_METER;     //左と右のエンコーダパルスの平均を表している。台車の位置。
    dx = (x - x0) / T;                                  //台車の位置の変位をTで微分しているので台車の速度を表している。
    x0 = x;                                             //xをここで保存している。dxを求めるときに使う。
    theta = e;                                          //目標となる角度と現在の角度のズレを表している。
    dtheta = ed;                                        //振子の角速度を表している。
    theta0 = theta;                                     //目標となる角度と現在の角度のズレを表している。

//  Calculate state feedback control
//  v_ref = -(x*K1 + dx*K2 + theta*K3 + dtheta*K4);     //電圧指令値を表している。
    
    duty_ratio = v_ref / MAX_V;                         //デューティ比を表している。
    if (duty_ratio > 1) duty_ratio = 1;
    else if (duty_ratio < -1) duty_ratio = -1;
    
    
    theta = (double)(theta_adv - ZERO_ADV) * ADV_TO_RAD;
    e = THETA_REF - theta;                    
    v_ref = e * KP;
    duty_ratio = v_ref / MAX_V;
    
//  指令値の頭打ち処理
    if(duty_ratio > 1.0) duty_ratio = 1.0;
    if(duty_ratio < -1.0) duty_ratio = -1.0;
    
    //**** 指令値によって発光するLEDを変える ****//
        if(duty_ratio > 0.8 ){
            led = 8;
        }else if(duty_ratio <= 0.8 && duty_ratio >= 0){
            led = 4;
        }else if(duty_ratio < 0 && duty_ratio >= -0.8){
            led = 2;
        }else if(duty_ratio < -0.8){
            led = 1;
        }
    //**** 指令値によって発光するLEDを変える　ここまで ****//
    
    //計算結果をモータの速度指令に反映
    motor_left =  duty_ratio*0.8;      //左モータの回転する方向を変えたいならここ
    motor_right = duty_ratio;      //右モータの回転する方向を変えたいならここ

}
//***************　台車の制御　ここまで　***************//


//***************　main関数　ここから　***************//
int main() {

//  モータの最大電圧範囲を設定
    motor_left.setMaxRatio(0.6);
    motor_right.setMaxRatio(0.6);
    
    pen_control.attach(&pen_control_handler, 0.001);        //台車の制御用のタイマー関数を設定
    
    led = 1;        //LEDの値を設定　動作確認用
    
    wait(1.0);      //なんとなく1秒待つ
    
    while(1) {      //無限ループ
        printf("theta_adv:%d duty_ratio:%2.2f \r\n", theta_adv ,duty_ratio);
        wait(0.08);
    }
}
