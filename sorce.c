// Sample Code for PID control / State Feedback control
// ADV_TO_RAD: AD値を実際の角度radに変換する係数
// THETA_REF: 目標となる角度
// ZERO_ADV: 棒の角度が0になる時のAD値
// T: サンプリング時間
// KP: Pゲイン
// KI: Iゲイン
// KD: Dゲイン
// PULSE_TO_METER: エンコーダパルス数を台車の位置に変換する係数
// K1-K4: 状態フィードバックゲイン（1x4のゲイン）
// MAX_V : 駆動系の最大電圧


adv =    ;    

    theta = (float)adv * ADV_TO_RAD;
	e = THETA_REF - theta;
	ed = (e - e0) / T;
	ei += e * T;
	e0 = e;
	
	if(ei > 10000) ei = 10000;
	if(ei > -10000) ei = -10000;

//  Calculate PID control
	v_ref = (e * KP + ei * KI + ed * KD);

//  Introduce x, dx, theta, dtheta
	x = (float)(left + right) / 2 * PULSE_TO_METER;
	dx = (x - x0) / T;
	x0 = x;
	theta = e;
	dtheta = ed;
	theta0 = theta;

//  Calculate state feedback control
//	v_ref = -(x*K1 + dx*K2 + theta*K3 + dtheta*K4);
	
	duty_ratio = v_ref / MAX_V;
	if (duty_ratio > 1) duty_ratio = 1;
	else if (duty_ratio < -1) duty_ratio = -1;
