
/***********************************************************************/
/*                                                                     */
/*  ファイル名			: move_motor.c                                 */
/*  ファイルの種類		: program file.                                */
/*                                                                     */
/***********************************************************************/

/***********************************************************************/
/*  インクルードファイル                                               */
/***********************************************************************/
#include "../Header_File/sfr_r8c2c2d.h"	// R8C/2Cのレジスタが定義されているヘッダファイル
#include "../Header_File/module.h"		// R8C/2Cのモジュールに深くかかわる関数等が宣言されているヘッダファイル
#include "../Header_File/main.h"		// 超音波センサの閾値等が定義されているヘッダファイル

/***********************************************************************/
/*  プロトタイプ宣言                                                   */
/***********************************************************************/
void move_motor( void );				//AGV3

/***********************************************************************/
/*	前進・後退するプログラム                                   */
/*		① 1秒程度、素早く前進する                                    */
/*		② 5cm程度、ゆっくり前進する                                   */
/*		③ 一時停止する                                                */
/*		④ 1秒程度、素早く後退する                                    */
/*		⑤ 5cm程度、ゆっくり後退する                                   */
/*		⑥ 一時停止する                                                */
/*		⑦ ①～⑥を繰り返し続ける                                      */
/***********************************************************************/
void move_motor( void )
{
/*****ローカル変数宣言*****/
	unsigned short distance_sensor_value[4];	// AD変換結果を格納する配列
												// 関数ad_readの注意通り、要素が4つ以上の配列にする。
	unsigned char obstacle1;					// 右前に障害物がある場合セットされるフラグ
	unsigned char obstacle2;					// 左前に障害物がある場合セットされるフラグ
	unsigned char obstacle_l;					// 左方近くに障害物がある場合セットされるフラグ
	unsigned char obstacle_r;					// 右方近くに障害物がある場合セットされるフラグ
	unsigned char course;						// 走行順序
	unsigned short sonic_th;					// 超音波閾値
	unsigned short ur_th;						// 赤外線閾値(3cm)
	unsigned short ur_tha;						// 赤外線閾値(1.5cm)
	unsigned long wait_time;					// 待機時間をカウントする変数
/******プログラム内容******/

	ad_init();											// A/Dコンバータモジュールの初期設定を行う
	ad_select( DISTANCE );								// A/Dコンバータモジュールの機能として距離センサを選択
	int_init();											// 外部割り込みモジュール(超音波受信)の初期設定を行う
	trb_init();											// タイマRBモジュール(超音波送信)の初期設定を行う
	trc_init();											// タイマRC(エンコーダ制御)の初期設定を行う
	trd_init();											// タイマRD(モータ制御)の初期設定を行う

	DIS_SEN_S = SEN_ON;									// 横方向の赤外距離センサをONする
	
	asm("FSET I");										// 割り込みを許可する
	sonic_th = 50;
	ur_th = 0x0035;
	course = 101;
while(1){
		
	if(course == 100){
		course = 101;
		for( wait_time=0; wait_time<610000*2; wait_time++ );	// 1秒間停止する
		goto LOOP1;
	}else if(course == 101){
		course = 102;
		rotate_tire( CW, CW, 1, 1, 30, 150*0.086*ONE_ROT);	
		goto LOOP1;
	}else if(course == 102){
		course = 103;
		rotate_tire( CW, CW, 0, 2, 60, ROT_90);	// 左へ90度回転する
		goto LOOP1;
	}else if(course == 103){
		course = 104;
		rotate_tire( CW, CW, 1, 1, 30, 105*0.086*ONE_ROT);
		goto LOOP1;
	}else if(course == 104){
		course = 105;
		for( wait_time=0; wait_time<610000*5; wait_time++ );	// 10秒間停止する
		goto LOOP1;
	}else if(course == 105){
		course = 106;
		rotate_tire( CW, CW, 1, 1, 30, 35*0.086*ONE_ROT);
		goto LOOP1;
	}else if(course == 106){
		course = 107;
		rotate_tire( CW, CW, 0, 2, 60, ROT_90);	// 左へ90度回転する
		goto LOOP1;
	}else if(course == 107){
		course = 108;
		rotate_tire( CW, CW, 1, 1, 30, 105*0.086*ONE_ROT);
		goto LOOP1;
	}else if(course == 108){
		course = 109;
		for( wait_time=0; wait_time<610000*5; wait_time++ );	// 10秒間停止する
		goto LOOP1;
	}else if(course == 109){
		course = 110;
		rotate_tire( CW, CW, 1, 1, 30, 35*0.086*ONE_ROT);
		goto LOOP1;
	}else if(course == 110){
		course = 111;
		rotate_tire( CW, CW, 0, 2, 60, ROT_90);	// 左へ90度回転する
		goto LOOP1;
	}else if(course == 111){
		course = 112;
		rotate_tire( CW, CW, 1, 1, 30, 150*0.086*ONE_ROT);
		goto LOOP1;
	}

LOOP1:;
// 超音波を用いた距離計測には5ms程度かかり、赤外線距離センサを用いた距離計測は11us程度かかる。

//----赤外線センサ------//
		// 赤外線距離センサで障害物判定をする。
		AD_STATUS = AD_RUN;											// AD変換を開始する
		while( AD_STATUS == AD_RUN );								// AD変換終了まで待機する
		ad_read( distance_sensor_value );							// AD変換結果を配列に格納する
		obstacle_l = 0;												// 左方の障害物フラグを全てクリアする
		obstacle_r = 0;												// 右方の障害物フラグを全てクリアする




		// 左横センサが3cm以内に障害物を検知したら
		if( distance_sensor_value[0] > ur_th ){
			obstacle_l = 1;											// 左障害物フラグをセットする

			LED1 = LED_ON;											// LED1(左)を点灯する。
		// 左横センサが3cm以内には障害物を検知しなかったら
		}else{
			LED1 = LED_OFF;											// LED1(左)を消灯する。
		}


//----赤外線センサ------//
		// 右横センサが3cm以内に障害物を検知したら
		if( distance_sensor_value[1] > ur_th ){
			obstacle_r= 1;											// 右障害物フラグをセットする

			LED3 = LED_ON;											// LED3(右)を点灯する。
		// 右横センサが3cm以内には障害物を検知しなかったら
		}else{
			LED3 = LED_OFF;											// LED3(右)を消灯する。
		}

		if(g_Motor_Distance != 0 ){
				// 左も右も障害物がなかった場合
				if( obstacle_l == 0 && obstacle_r == 0 ){
				// 左か右に障害物があった場合
				//右横に障害物有り	
				}else if( obstacle_l == 0 && obstacle_r == 1){
				change_motor_l_speed( CW, 10 );			// 左モータをDuty20%で前転させる
				change_motor_r_speed( CW, 25 );			// 右モータをDuty25%で前転させる
				//左横に障害物有り
				}else if( obstacle_l == 1 && obstacle_r == 0 ){
				change_motor_l_speed( CW, 25 );			// 左モータをDuty25%で前転させる
				change_motor_r_speed( CW, 10 );			// 右モータをDuty20%で前転させる
				//右横、左横に障害物有り	
				}else if( obstacle_l == 1 && obstacle_r == 1 ){
					rotate_tire( STOP, STOP, 0, 0, 0, 0);		// 停止する
				}// if( obstacle_l == 0 && obstacle_r == 0 )の終了括弧

				}//if(g_Motor_Distance != 0 )の終了括弧
		if(g_Motor_Distance == 0 ){ goto OUT1;}
			
	goto LOOP1;
	OUT1:;
	
	}

}	// void line_sensor( void )の終了括弧