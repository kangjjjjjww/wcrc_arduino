#include "AIRO_XARM.h"
#include <TimerFive.h>
#include <Pixy2.h>
#include <string.h>
int pixy_x=0;
Pixy2   pixy;

//Robot parameter
#define L       45      //45mm
#define D       85      //85mm
#define LPLUSD  0.13    //L+D=130mm/1000 = 0.13m
#define R       30      //30mm

#define NUM_OBJECT  6
#define ITER_NUM 10

//for Encoder pin define
#define ENC1_CHA    18  //INT.5
#define ENC1_CHB    31
#define ENC2_CHA    19  //INT.4
#define ENC2_CHB    38
#define ENC3_CHA    3   //INT.1
#define ENC3_CHB    49
#define ENC4_CHA    2   //INT.0
#define ENC4_CHB    A1

//for Motor I/O pin define
#define M1_I1       34
#define M1_I2       35
#define M1_PWM      12
#define M2_I1       37
#define M2_I2       36
#define M2_PWM      8
#define M3_I1       42
#define M3_I2       43
#define M3_PWM      9
#define M4_I1       A5
#define M4_I2       A4
#define M4_PWM      5
#define LED_R       A13
#define LED_G       A14
#define LED_B       A15
//
#define PSD_R       A7
#define PSD_L       A6

//for PID control
//motor1 gain
#define Kp1         0.65
#define Ki1         0.2
#define Kd1         0.15
//motor2 gain
#define Kp2         0.6
#define Ki2         0.2
#define Kd2         0.15
//motor3 gain
#define Kp3         0.6
#define Ki3         0.2
#define Kd3         0.16
//motor4 gain
#define Kp4         0.6
#define Ki4         0.2
#define Kd4         0.15

bool ul_rcv_flag= false;
byte ul_rdata ='0';

//Timer control
bool t5_flag=0;

//pixy Variables
int object_position[2] = {};
int x_value[NUM_OBJECT];
int block_check[2];
int pixy_err_count = 0;
int pixy_err_count_1 = 0;
int block_p = 0;
int dir_flag = 0;
float lr_time = 0, lr_value = 0;
int iter_count = 0;

int signiture_xvalue = 0; // 미세 조정을 위한 블록의 x축 값
int detect_blocks = 0; //================================================변경============================================
int xvalue_avg_value = 0;
int xvalue_avg[NUM_OBJECT] = {};
int yvalue_avg[NUM_OBJECT] = {};

int mission[NUM_OBJECT] = {}; //미션 인식 존에서 인식한 물체 저장
int frameposition[NUM_OBJECT] = {}; //

int object_signiture[ITER_NUM][NUM_OBJECT] = {}; //적재함에서 인식한 물체 저장
int object_raw_xvalue[ITER_NUM][NUM_OBJECT] = {};
int object_raw_yvalue[ITER_NUM][NUM_OBJECT] = {};
int object_sorted_xvalue[ITER_NUM][NUM_OBJECT] = {};
int object_sorted_yvalue[ITER_NUM][NUM_OBJECT] = {};


//Encoder value
long    e1cnt = 0;
long    e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
long    e2cnt = 0;
long    e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;
long    e3cnt = 0;
long    e3cnt_k = 0, e3cnt_k_1 = 0, d_e3cnt = 0;
long    e4cnt = 0;
long    e4cnt_k = 0, e4cnt_k_1 = 0, d_e4cnt = 0;

//motor value
float   m1speed = 0;
float   m2speed = 0;
float   m3speed = 0;
float   m4speed = 0;
float x_val = 0, y_val = 0, w_val = 0;

//for motor control variable
//motor1
float   m1_ref_spd = 0;
float   m1_err_spd = 0;
float   m1_err_spd_k_1 = 0;
float   m1_derr_spd = 0;
float   m1_err_sum = 0;
float   m1_ctrl_up = 0;
float   m1_ctrl_ui = 0;
float   m1_ctrl_ud = 0;
int     m1_ctrl_u = 0;
int     m1_ipwm_u = 0;

//motor2
float   m2_ref_spd = 0;
float   m2_err_spd = 0;
float   m2_err_spd_k_1 = 0;
float   m2_derr_spd = 0;
float   m2_err_sum = 0;
float   m2_ctrl_up = 0;
float   m2_ctrl_ui = 0;
float   m2_ctrl_ud = 0;
int     m2_ctrl_u = 0;
int     m2_ipwm_u = 0;
//motor3
float   m3_ref_spd = 0;
float   m3_err_spd = 0;
float   m3_err_spd_k_1 = 0;
float   m3_derr_spd = 0;
float   m3_err_sum = 0;
float   m3_ctrl_up = 0;
float   m3_ctrl_ui = 0;
float   m3_ctrl_ud = 0;
int     m3_ctrl_u = 0;
int     m3_ipwm_u = 0;
//motor4
float   m4_ref_spd = 0;
float   m4_err_spd = 0;
float   m4_err_spd_k_1 = 0;
float   m4_derr_spd = 0;
float   m4_err_sum = 0;
float   m4_ctrl_up = 0;
float   m4_ctrl_ui = 0;
float   m4_ctrl_ud = 0;
int     m4_ctrl_u = 0;
int     m4_ipwm_u = 0;

unsigned int dis_l, dis_r = 0;
unsigned int dis_avg_l = 0, dis_avg_r = 0, dis_avg = 0;
unsigned int m_seq = 9999, wait_flag = 0, set_cycle = 0;
unsigned long time = millis() ;

void setup() {
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(ENC3_CHA, INPUT_PULLUP);
  pinMode(ENC3_CHB, INPUT_PULLUP);
  pinMode(ENC4_CHA, INPUT_PULLUP);
  pinMode(ENC4_CHB, INPUT_PULLUP);
  pinMode(M1_I1, OUTPUT);
  pinMode(M1_I2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_I1, OUTPUT);
  pinMode(M2_I2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_I1, OUTPUT);
  pinMode(M3_I2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M4_I1, OUTPUT);
  pinMode(M4_I2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), Enc2chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_CHA), Enc3chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_CHA), Enc4chA_ISR, CHANGE);
  //Motor initialize
  digitalWrite(M1_I1, HIGH);  //M1: I1(H), I2(L) -> Forward
  digitalWrite(M1_I2, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_I1, HIGH);  //M2: I1(H), I2(L) -> Forward
  digitalWrite(M2_I2, LOW);
  analogWrite(M2_PWM, 0);
  digitalWrite(M3_I1, HIGH);  //M3: I1(H), I2(L) -> Forward
  digitalWrite(M3_I2, LOW);
  analogWrite(M3_PWM, 0);
  digitalWrite(M4_I1, HIGH);  //M4: I1(H), I2(L) -> Forward
  digitalWrite(M4_I2, LOW);
  analogWrite(M4_PWM, 0);
  delay(500);

  //pixy 2 -> init
  pixy.init();
  delay(100);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(500);

  //led setup before mission start
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_R, HIGH);
    delay(100);
    digitalWrite(LED_R, LOW);
    delay(100);
  }
  digitalWrite(LED_G, HIGH);
  delay(100);
  digitalWrite(LED_G, LOW);
  delay(100);

  //Timer5 setup
  Timer5.initialize(10000);       //10msec,
  Timer5.attachInterrupt(T5ISR);  //T5ISR
}

void loop() {

switch (m_seq) {

    case 9999:// 로봇 팔 회전... 
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 0;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 0: // step 0->장애물까지 전진
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 1234; //case 1로 이동a7
          set_cycle = 0;
          wait_flag = 0;
          dis_avg = 0;
        }
      }
      else {
        if (dis_avg > 630) {  //about 10cm (가까워질수록 값이 커진다)
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          psd();
          x_val = 15;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

    case 1234: // 좌측으로 이동...
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 130) { //wait for 1sec(100*10msec)
          m_seq = 12345;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 100) {  //run for 2.0sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 15; // 20cm/s /////////////aaaa
          w_val = 1;
        }
      }
      break;

      case 12345:// pixy인식을 위해 후진
        if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 1;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 60) {  //run for sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -8;
          y_val = 0; // 20cm/s
          w_val = 1; ///보정
        }
      }
    break;

    case 1: //미션 인식 존...pixy2이용해서 미션 인식하기
      if (wait_flag) {
        if (set_cycle > 100) { //wait for
          m_seq = 99991;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 101) {
          //To use Pixy2, turn off timer 5
          Timer5.stop();
          delay(200);
          dae_Servomove(910,370,50,300,300, 2000);
          delay(2000);
          Serial.println("up");
          //parameter_clr();

          //------------------------------------------------------- 픽시 캠 사용 ---------------------------------------------------------
          pixy.ccc.getBlocks(); //인식하는 것을 평균...내거나 해서 필터링해야하는데...

          detect_blocks = pixy.ccc.numBlocks;
          delay(50);

          while (NUM_OBJECT != detect_blocks) //원하는 개수만큼 인식이 되지 않는다면 인식될 때까지 대기
          {Serial.print(detect_blocks);
            if (pixy_err_count == 0)
            
            {Serial.print(detect_blocks);
              ServoMove(Serial2, 11 , 930 , 1000);
              Serial.print("down1");
              delay(2000);


              pixy_err_count = 1;
              delay(500);
              

            }
            else if (pixy_err_count == 1)
            {
               ServoMove(Serial2, 11 , 875 , 1000);
               Serial.print("down2");
              delay(2000);
              pixy_err_count = 2;
              delay(500);
            }
            else
            {
              ServoMove(Serial2, 11 , 945 , 1000);
              Serial.print("down3");
              delay(2000);
              pixy_err_count = 0;
              delay(500);
            }
            //repeat printing red led

            digitalWrite(LED_R, HIGH);
            delay(200);
            digitalWrite(LED_R, LOW);
            delay(500);
            pixy.ccc.getBlocks();
            detect_blocks = pixy.ccc.numBlocks;
          }
          pixy_err_count = 0;

          for (int i = 0; i < detect_blocks; i++) //지정된 개수만큼 저장
          {
            mission[i] = pixy.ccc.blocks[i].m_signature;
            x_value[i] = pixy.ccc.blocks[i].m_x;
          }
          SortBlocks();//x좌표 정보에 따른 버블정렬을 수행한다
          detect_blocks = 0;

          //정렬이 완료되면 여기 시퀀스는 끝
          //시퀀스 ++, 타이머인터럽트 restart();
          /**/
          
          //Change Arm pos for detect 2
          //angle_mission_detect1[3] = 1086;
           dae_Servomove(500,500,500,500,500,2000);
          //angle_mission_detect1[3] = 2779;
//          Serial.print(mission[0]);
//          Serial.print(' ');
//          Serial.print(mission[1]);
//          Serial.print(' ');
//          Serial.print(mission[2]);
//          Serial.print(' ');
//          Serial.print(mission[3]);
//          Serial.println(' ');
          //delay(1500);
          //Sync_Pos(init_pos);
          //printing led for checking if values are saved correctly
//          printled(pixy.ccc.numBlocks); // 인식한 개수만큼 led 깜박이는 명령어. 내부에서 mission 배열 불러옴.

          //timer5 restart
          Timer5.restart();
          wait_flag=1;

//          if (wait_flag == 0) {
//            set_cycle = 0;
//            wait_flag = 1;
//          }

        }
//        else {
//          if (set_cycle == 0) {
//            parameter_clr();
//            set_cycle = 2;
//          }
////          else t5_index = 10; // t5_index는 계속 default 가 되게 큰 임의 값으로 유지
//        }
      }
      break;

      

      case 99991:// 팔 초기값
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 2;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


    case 2: // 좌측으로 이동...
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 2111;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 90) {  //run for 2.0sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 15; // 20cm/s ///////////////aaaa
          w_val = 0;
        }
      }
      break;

      case 2111: // 약간 직진
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 2112;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 260) {  //run for 2.0sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 15;
          y_val = 0; // 20cm/s
          w_val = 1;
        }
      }
      break;

      case 2112: // right         ///////////////////////aaaa
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 120) { //wait for 1sec(100*10msec)
          m_seq = 4;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 110) {  //run for 2.0sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -15; // 20cm/s
          w_val = 0;
        }
      }
      break;


      case 4: // 로봇 팔 들기...(픽시 캠 인식을 위한 각도로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 60) { //wait for 1sec(100*10msec)
          m_seq = 5;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        dae_Servomove(490,550,500,500,500,2000);
        delay(2000);

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 5: // 미션 그립 구역에서 픽시캠 인식 단계
      if (wait_flag) {
        if (set_cycle > 100) { //wait for
          m_seq = 500;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 101) {  //wait for 1.0s before turn off timer5
          ////To use Pixy2, turn off timer 5
          Timer5.stop();
          delay(200);
          //parameter_clr();

          //------------------------------------------------------------ 픽시 캠 사용 ----------------------------------------
          detect_blocks = 0;

          pixy.ccc.getBlocks();
          detect_blocks = pixy.ccc.numBlocks;
          delay(50);

          while (NUM_OBJECT != detect_blocks) //원하는 개수만큼 인식이 되지 않는다면 인식될 때까지 대기
          {
           
              if(pixy_err_count == 0)
              {
              dae_Servomove(510,550,500,500,500,2000);
              pixy_err_count = 1;
              delay(500);

              }
              else if(pixy_err_count == 1)
              {
              dae_Servomove(470,550,500,500,500,2000);
              pixy_err_count = 2;
              delay(500);
              }
              else
              {
              dae_Servomove(490,550,500,500,500,2000);
              pixy_err_count = 0;
              delay(500);
              }
            
            //repeat printing red led
            digitalWrite(LED_R, HIGH);
            delay(200);
            digitalWrite(LED_R, LOW);
            delay(500);

            pixy.ccc.getBlocks();
            detect_blocks = pixy.ccc.numBlocks;
          }
          //pixy_err_count = 0;

          //NUM_OBJECT 개수만큼 각각 배열에 저장하기
          for (int iter = 0; iter < ITER_NUM; iter++)
          {
            pixy.ccc.getBlocks();
            detect_blocks = pixy.ccc.numBlocks;
            for (int onum = 0; onum < NUM_OBJECT; onum++)
            {
              //미션 수행 블럭들의 빨주노초 숫자와 x, y 값을 각각 저장
              //행은 반복 횟수를 의미한다. 각 행의 열 순서는 랜덤으로 저장되는 값들이다.
              object_signiture[iter][onum] = pixy.ccc.blocks[onum].m_signature;
              object_raw_xvalue[iter][onum] = pixy.ccc.blocks[onum].m_x;
              object_raw_yvalue[iter][onum] = pixy.ccc.blocks[onum].m_y;
            }
            delay(20);
          }

          //이제 이렇게 각 block들 *10번 저장한 값들을,,, 미션 인식 존에서 인식한 순서로 정렬
          //mission[]에 저장되어있다.
          for (int iter = 0; iter < ITER_NUM; iter++)
          {
            for (int i = 0; i < NUM_OBJECT; i++)
            {
              for (int j = 0; j < NUM_OBJECT; j++)
              {
                if (mission[i] == object_signiture[iter][j])
                {
                  object_sorted_xvalue[iter][i] = object_raw_xvalue[iter][j];
                  object_sorted_yvalue[iter][i] = object_raw_yvalue[iter][j];
                }
              }
            }
          }

          //이제 mission 인식한 순서대로 x, y value가 저장되어있으므로, 평균내기
          //평균을 내기 전에 극단적인 값들을 빼주려면 정렬이 필요함.
          //행에 대해서 정렬하는 알고리즘이 필요,,,
          SortBlocks_row(object_sorted_xvalue, ITER_NUM, NUM_OBJECT);
          SortBlocks_row(object_sorted_yvalue, ITER_NUM, NUM_OBJECT);

          //각각의 object의 x값과 y값에 대해 평균내기(극값 각각 2개씩 제외)
          for (int col = 0; col < NUM_OBJECT; col ++)
          {
            for (int row = 2; row < ITER_NUM - 2; row++)
            {
              xvalue_avg[col] = xvalue_avg[col] + object_sorted_xvalue[row][col];
              yvalue_avg[col] = yvalue_avg[col] + object_sorted_yvalue[row][col];
            }
            xvalue_avg[col] = xvalue_avg[col] / 6;
            yvalue_avg[col] = yvalue_avg[col] / 6;
          }

          //반복하고 평균내서 구한 x y값에 따라 구역을 나눠보기
          positioning(xvalue_avg, yvalue_avg); // object_position라는 배열에 저장됨
          //object_position에 저장되는 순서는 이미 mission에 의해 정렬해 놓은 x y 값들


          //----------------------------------------------------------여기까지는 추가한 것
          Timer5.restart();

          if (wait_flag == 0) {
//            t5_index = 0;
            set_cycle = 0;
            wait_flag = 1;
          }

        }
        else {
          if (set_cycle == 0) {
            parameter_clr();
            set_cycle = 1;
          }
//          else t5_index = 10; // t5_index는 계속 default 가 되게 큰 임의 값으로 유지
        }
      }
      break;

      case 3: // 미션 그립존까지
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 6; //case 1로 이동a7
          set_cycle = 0;
          wait_flag = 0;
          dis_avg = 0;
        }
      }
      else {
        if (set_cycle > 90) {  //run for 2.0sec
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 15;
          y_val = 0; // 20cm/s
          w_val = -1;
        }
      }
      break;


      

      case 500:// y축으로 얼마나 이동할지 결정
      switch (object_position[0]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
          m_seq = 501;
          break;

        case 2:
        case 6:
          m_seq = 502;
          break;

        case 3:
        case 7:
          m_seq = 503;
          break;

        case 4:
        case 8:
          m_seq = 504;
          break;

        default: break;
      }
      break;

    case 501: // 로봇 y축 이동
      // 1번째 열에 있는 블럭을 잡으러 가는 경우
      go_1(52);
      break;
    case 503: // 로봇 y축 이동 /bbbb
      // 3번째 열에 있는 블럭을 잡으러 가는 경우, 움직이지 않음
      m_seq = 52;
      break;

    case 502: // 로봇 y축 이동
      // 2번째 열에 있는 블럭을 잡으러 가는 경우
      go_2(52);
      break;
    case 504: // 로봇 y축 이동
      // 4번째 열에 있는 블럭을 잡으러 가는 경우
      go_4(52);
      break;

    case 52: // 로봇 팔 회전...(픽시 캠 인식 후, 1번째  블럭을 잡기 좋은 위치로)
      //
      if (wait_flag) {
        if (set_cycle > 250) { //wait for 1sec(100*10msec)
          m_seq = 3;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[0]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            no_grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            no_grip_1();
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
      //=====================================================================================================
       case 53: // pixy를 이용해 미세조정
        if (wait_flag) {
        if (set_cycle > 100) { //wait for
          m_seq = 61;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        
        if (set_cycle > 101) {  //wait for 1.0s before turn off timer5
          ////To use Pixy2, turn off timer 5
          Timer5.stop();
          delay(200);
//------------------------------------------------ 픽시 캠 사용해서 열 미세조정 ----------------------------------------
          pixy.ccc.getBlocks();
          delay(50);
      while(pixy_x=0)
        {
          if (pixy.ccc.numBlocks)
          { 
            for (int i=0; i<pixy.ccc.numBlocks; i++)
            {
              if(pixy.ccc.blocks[i].m_signature == mission[0])
              {
                pixy_x = pixy.ccc.blocks[i].m_x;
              }
            else
              {         //물체를 인식했는데 원하는게 없다면 파란불
              digitalWrite(LED_G, HIGH);
              delay(200);
              digitalWrite(LED_G, LOW);
              delay(200);
              }
            }
          }
        else //물체를 아무것도 인식 못했다면 빨간불
        {          
            digitalWrite(LED_R, HIGH);
            delay(200);
            digitalWrite(LED_R, LOW);
            delay(200);
        }
      }
          if (pixy_x>>158)// 픽시캠 중간 값 158
          {   
            while(pixy_x>>158)
            {
            right(); //or left
            }
          }
          else if(pixy_x<<158)
          {
            while(pixy_x<<158)
            {
              left(); //or right
            }
          }
//          else{
//            stop_();
//            Timer5.restart();
//            set_cycle=0;
//            wait_flag=1;
//          }
      }
   }
break;
      //=====================================================================================================
     case 61: // 
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 6;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 70) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

    
    case 6: // 전방으로 약간 직진...(블럭 잡는 지점 바로앞으로)
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 7;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 100) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

     


    case 7: // 첫 번째물건 그립(음,,,)
      if (wait_flag) {
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 8;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[0]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층에서 그립
            dae_Servomove(500,280,100,320,620,2000); //////2 층 완료
            delay(2000);
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 1층에서 그립
            dae_Servomove(500,380,70,200,620,2000);
            delay(2000);
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 8: //후진하기 (블록 goal 지점방향으로 가기 위해)
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 81;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 100) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;
      
    case 81: // 로봇 팔 회전...(회전하는데 걸리적 안하게)
      if (wait_flag) {
        if (set_cycle > 350) { //wait for 1sec(100*10msec)
          m_seq = 810;  //next step            ////////////////810으로 수정
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[0]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 위로 팔을 올려도 되니까, detect1 포지션으로 돌리고
          good_move();
          delay(2000);
 
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 2층을 치면 안되니까.
          good_move();
          delay(2000);
  
            break;
        }

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 810:// (3열 앞으로)
      Serial.println(object_position[0]);
      switch (object_position[0]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
          m_seq = 811;
          break;

        case 2:
        case 6:
          m_seq = 812;
          break;

        case 3:
        case 7:
          m_seq = 813;
          break;

        case 4:
        case 8:
          m_seq = 814;
          break;

        default: break;
      }
      break;

    case 811: // 로봇 y축 이동
      // 1번째 열에 있는 블럭을 잡은 경우
      back_1(10);
      break;
    case 813: // 로봇 y축 이동
      // 3번째 열에 있는 블럭을 잡은 경우, 움직이지 않음
      m_seq = 10;
      break;

    case 812: // 로봇 y축 이동
      // 2번째 열에 있는 블럭을 잡은 경우
      back_2(10);
      break;

    case 814: // 로봇 y축 이동
      // 4번째 열에 있는 블럭을 잡은 경우
      back_4(10);
      break;
   
    case 10://3열에 정렬 후 블록 goal 지점까지
      go_goal(101);
      break;
      
    case 101: // 로봇 팔 회전... (임의로 블록 goal지점 1번 위치로)
      if (wait_flag) {
        if (set_cycle > 200) { //wait for 1sec(100*10msec)
          m_seq = 102;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[0])
        {
          case 1: // 원소가 빨간색이면       /////// 미션 받고 변경/////
            drop_1();
            break;

          case 2:
          drop_1();
            break;
          case 3:
            drop_1();
            break;

          case 4:
            drop_1();
            break;

          case 5:
          drop_4();
            break;

          case 6:
          drop_1();
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 102: // 그리퍼 오픈...(mission goal 1지점)
      if (wait_flag) {
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 109;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[0])  ///////==================미션받고 그리퍼 폈을 때 모터값 000에 대입, 나머지 제거============
        {
          case 1: // 원소가 빨간색이면
          ServoMove(Serial2, 5,300, 2000);
          delay(2000);
            break;

          case 2:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
            break;
          case 3:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 4:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 5:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
            break;

          case 6:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
      //==========================================1번째 물체 이동 완료=========================================
     //----------------------------------------- 2 번째 물체 목표 -----------------------------------------------------------------------------
    case 109: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 103;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 103:// 되돌아가기
      back_goal(104);
      break;
      

    case 104: // 로봇 팔 회전...
      // 두 번째 블록 위치로 회전(1층 or 2층)
      if (wait_flag) {
        if (set_cycle > 190) { //wait for 1sec(100*10msec)
          m_seq = 110;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[1]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지///////////미션 층수 수정//
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            no_grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            no_grip_1(); //// grip_1()측정해서 수정
            break;
          default: break;

        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


      

    case 110:// 
      switch (object_position[1]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        go_1(12);
        break;

        case 2:
        case 6:
        go_2(12);
        break;

        case 3:
        case 7:
        m_seq=12;
        break;

        case 4:
        case 8:
        go_4(12);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;

      case 12:// 전방으로 약간 직진...(블럭 잡는 지점 바로앞으로)
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 121;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 121://2번째 물체 그립 수행
      //임의로 1층 물체
      if (wait_flag) {
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 122;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[1]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층에서 그립
            grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 1층에서 그립
            grip_1();
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 122:  //후진...(그립 수행 후 후진)
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 139;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -7;
          y_val = 0;
          w_val = -2;
        }
      }
      break;

    case 139: // 로봇 팔 회전...(회전하는데 걸리적 안하게)
      if (wait_flag) {
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 1399;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[1]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 위로 팔을 올려도 되니까, detect1 포지션으로 돌리고
          good_move();
          break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 2층을 치면 안되니까.
          good_move();
          break;
        }

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 1399:// 3열로 정렬
      switch (object_position[1]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        back_1(13);
        break;

        case 2:
        case 6:
        back_2(13);
        break;

        case 3:
        case 7:
        m_seq=13;
        break;

        case 4:
        case 8:
        back_4(13);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;
      
      case 13://y축 직진...(미션 goal 지점방향으로)
      go_goal(14);
      break;
      
      case 14: //로봇 팔 회전... (2번째 미션 goal지점으로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 141;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[1])////////////////////////////////////미션받고 drop_n추가////////////////////
        {
          case 1: // 원소가 빨간색이면
            drop_1();
            break;

          case 2:
            drop_1();
            break;
          case 3:
            drop_1();
            break;

          case 4:
            drop_1();
            break;

          case 5:
            drop_1();
            break;

          case 6:
            drop_1();
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 141: //그리퍼 오픈... (2번째 미션 goal지점)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 159;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[1])  
        {
          case 1: // 원소가 빨간색이면
            ServoMove(Serial2, 5, 300, 2000); ///////////////////그리퍼 open값//////////////
            delay(2000);
            break;

          case 2:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;
          case 3:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 4:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 5:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 6:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
          break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 2 번째 물체 이동 완료 --------------------------------------------------------------------------

 //----------------------------------------- 3 번째 물체 목표 -----------------------------------------------------------------------------
    case 159: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 150;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 150:// 되돌아가기
      back_goal(1511);
      break;
      

    case 1511: // 로봇 팔 회전...
      // 두 번째 블록 위치로 회전(1층 or 2층)
      if (wait_flag) {
        if (set_cycle > 190) { //wait for 1sec(100*10msec)
          m_seq = 15;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[2]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지///////////미션 층수 수정//
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            no_grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            no_grip_1(); //// grip_1()측정해서 수정
            break;
          default: break;

        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


      

    case 15:// 
      switch (object_position[2]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        go_1(151);
        break;

        case 2:
        case 6:
        go_2(151);
        break;

        case 3:
        case 7:
        m_seq=151;
        break;

        case 4:
        case 8:
        go_4(151);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;

      case 151:// 전방으로 약간 직진...(블럭 잡는 지점 바로앞으로)
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 1522;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 1522://2번째 물체 그립 수행
      //임의로 1층 물체
      if (wait_flag) {
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 153;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[2]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층에서 그립
            grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 1층에서 그립
            grip_1();
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 153:  //후진...(그립 수행 후 후진)
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 154;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -7;
          y_val = 0;
          w_val = -2;
        }
      }
      break;

    case 154: // 로봇 팔 회전...(회전하는데 걸리적 안하게)
      if (wait_flag) {
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 155;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[2]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 위로 팔을 올려도 되니까, detect1 포지션으로 돌리고
          good_move();
          break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 2층을 치면 안되니까.
          good_move();
          break;
        }

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 155:// 3열로 정렬
      switch (object_position[2]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        back_1(16);
        break;

        case 2:
        case 6:
        back_2(16);
        break;

        case 3:
        case 7:
        m_seq=16;
        break;

        case 4:
        case 8:
        back_4(16);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;
      
      case 16://y축 직진...(미션 goal 지점방향으로)
      go_goal(161);
      break;
      
      case 161: //로봇 팔 회전... (2번째 미션 goal지점으로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 162;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[2])////////////////////////////////////미션받고 drop_n추가////////////////////
        {
          case 1: // 원소가 빨간색이면
            drop_1();
            break;

          case 2:
            drop_1();
            break;
          case 3:
            drop_1();
            break;

          case 4:
            drop_1();
            break;

          case 5:
            drop_1();
            break;

          case 6:
            drop_1();
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 162: //그리퍼 오픈... (2번째 미션 goal지점)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 700;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[2])  
        {
          case 1: // 원소가 빨간색이면
            ServoMove(Serial2, 5, 300, 2000); ///////////////////그리퍼 open값//////////////
            delay(2000);
            break;

          case 2:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;
          case 3:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 4:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 5:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 6:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
          break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 3 번째 물체 이동 완료 --------------------------------------------------------------------------


//----------------------------------------- 4 번째 물체 목표 -----------------------------------------------------------------------------
    case 700: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 701;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 701:// 되돌아가기
      back_goal(702);
      break;
      

    case 702: // 로봇 팔 회전...
      // 두 번째 블록 위치로 회전(1층 or 2층)
      if (wait_flag) {
        if (set_cycle > 190) { //wait for 1sec(100*10msec)
          m_seq = 703;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[3]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지///////////미션 층수 수정//
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            no_grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            no_grip_1(); //// grip_1()측정해서 수정
            break;
          default: break;

        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


      

    case 703:// 
      switch (object_position[3]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        go_1(704);
        break;

        case 2:
        case 6:
        go_2(704);
        break;

        case 3:
        case 7:
        m_seq=704;
        break;

        case 4:
        case 8:
        go_4(704);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;

      case 704:// 전방으로 약간 직진...(블럭 잡는 지점 바로앞으로)
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 705;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 705://2번째 물체 그립 수행
      //임의로 1층 물체
      if (wait_flag) {
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 706;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[3]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층에서 그립
            grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 1층에서 그립
            grip_1();
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 706:  //후진...(그립 수행 후 후진)
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 707;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

    case 707: // 로봇 팔 회전...(회전하는데 걸리적 안하게)
      if (wait_flag) {
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 708;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[3]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 위로 팔을 올려도 되니까, detect1 포지션으로 돌리고
          good_move();
          break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 2층을 치면 안되니까.
          good_move();
          break;
        }

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 708:// 3열로 정렬
      switch (object_position[3]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        back_1(709);
        break;

        case 2:
        case 6:
        back_2(709);
        break;

        case 3:
        case 7:
        m_seq=709;
        break;

        case 4:
        case 8:
        back_4(709);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;
      
      case 709://y축 직진...(미션 goal 지점방향으로)
      go_goal(710);
      break;
      
      case 710: //로봇 팔 회전... (2번째 미션 goal지점으로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 711;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[3])////////////////////////////////////미션받고 drop_n추가////////////////////
        {
          case 1: // 원소가 빨간색이면
            drop_1();
            break;

          case 2:
            drop_1();
            break;
          case 3:
            drop_1();
            break;

          case 4:
            drop_1();
            break;

          case 5:
            drop_1();
            break;

          case 6:
            drop_1();
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 711: //그리퍼 오픈... (2번째 미션 goal지점)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 800;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[3])  
        {
          case 1: // 원소가 빨간색이면
            ServoMove(Serial2, 5, 300, 2000); ///////////////////그리퍼 open값//////////////
            delay(2000);
            break;

          case 2:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;
          case 3:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 4:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 5:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 6:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
          break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 4 번째 물체 이동 완료 --------------------------------------------------------------------------


//----------------------------------------- 5 번째 물체 목표 -----------------------------------------------------------------------------
    case 800: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 801;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 801:// 되돌아가기
      back_goal(802);
      break;
      

    case 802: // 로봇 팔 회전...
      // 두 번째 블록 위치로 회전(1층 or 2층)
      if (wait_flag) {
        if (set_cycle > 190) { //wait for 1sec(100*10msec)
          m_seq = 803;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[4]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지///////////미션 층수 수정//
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            no_grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            no_grip_1(); //// grip_1()측정해서 수정
            break;
          default: break;

        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


      

    case 803:// 
      switch (object_position[4]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        go_1(804);
        break;

        case 2:
        case 6:
        go_2(804);
        break;

        case 3:
        case 7:
        m_seq=804;
        break;

        case 4:
        case 8:
        go_4(804);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;

      case 804:// 전방으로 약간 직진...(블럭 잡는 지점 바로앞으로)
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 805;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 805://2번째 물체 그립 수행
      //임의로 1층 물체
      if (wait_flag) {
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 806;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[4]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층에서 그립
            grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 1층에서 그립
            grip_1();
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 806:  //후진...(그립 수행 후 후진)
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 807;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

    case 807: // 로봇 팔 회전...(회전하는데 걸리적 안하게)
      if (wait_flag) {
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 808;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[4]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 위로 팔을 올려도 되니까, detect1 포지션으로 돌리고
          good_move();
          break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 2층을 치면 안되니까.
          good_move();
          break;
        }

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 808:// 3열로 정렬
      switch (object_position[4]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        back_1(809);
        break;

        case 2:
        case 6:
        back_2(809);
        break;

        case 3:
        case 7:
        m_seq=809;
        break;

        case 4:
        case 8:
        back_4(809);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;
      
      case 809://y축 직진...(미션 goal 지점방향으로)
      go_goal(8100);
      break;
      
      case 8100: //로봇 팔 회전... (2번째 미션 goal지점으로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 8111;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[4])////////////////////////////////////미션받고 drop_n추가////////////////////
        {
          case 1: // 원소가 빨간색이면
            drop_1();
            break;

          case 2:
            drop_1();
            break;
          case 3:
            drop_1();
            break;

          case 4:
            drop_1();
            break;

          case 5:
            drop_1();
            break;

          case 6:
            drop_1();
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 8111: //그리퍼 오픈... (2번째 미션 goal지점)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 900;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[4])  
        {
          case 1: // 원소가 빨간색이면
            ServoMove(Serial2, 5, 300, 2000); ///////////////////그리퍼 open값//////////////
            delay(2000);
            break;

          case 2:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;
          case 3:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 4:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 5:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 6:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
          break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 5 번째 물체 이동 완료 --------------------------------------------------------------------------
//----------------------------------------- 6 번째 물체 목표 -----------------------------------------------------------------------------
    case 900: // 로봇 팔 회전... 1층잡으러갈때 걸리는 거 방지
      if (wait_flag) {
        if (set_cycle > 140) { //wait for 1sec(100*10msec)
          m_seq = 901;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();
        ////////////////////////이동하기 편한 모터 값 체크///////////////////

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 901:// 되돌아가기
      back_goal(902);
      break;
      

    case 902: // 로봇 팔 회전...
      // 두 번째 블록 위치로 회전(1층 or 2층)
      if (wait_flag) {
        if (set_cycle > 190) { //wait for 1sec(100*10msec)
          m_seq = 903;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[5]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지///////////미션 층수 수정//
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층 잡게 로봇 팔 움직이기.
            no_grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8:
            no_grip_1(); //// grip_1()측정해서 수정
            break;
          default: break;

        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


      

    case 903:// 
      switch (object_position[4]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        go_1(904);
        break;

        case 2:
        case 6:
        go_2(904);
        break;

        case 3:
        case 7:
        m_seq=904;
        break;

        case 4:
        case 8:
        go_4(904);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;

      case 904:// 전방으로 약간 직진...(블럭 잡는 지점 바로앞으로)
       if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 905;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

      case 905://2번째 물체 그립 수행
      //임의로 1층 물체
      if (wait_flag) {
        if (set_cycle > 110) { //wait for 1sec(100*10msec)
          m_seq = 906;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[5]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 2층에서 그립
            grip_2();
            break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 1층에서 그립
            grip_1();
            break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 906:  //후진...(그립 수행 후 후진)
      if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = 907;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 60) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -7;
          y_val = 0;
          w_val = 0;
        }
      }
      break;

    case 907: // 로봇 팔 회전...(회전하는데 걸리적 안하게)
      if (wait_flag) {
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 908;  //next step
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (object_position[5]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 1층인지 2층인지
        {
          case 1:
          case 2:
          case 3:
          case 4: // 2층위치이면 위로 팔을 올려도 되니까, detect1 포지션으로 돌리고
          good_move();
          break;

          case 5:
          case 6:
          case 7:
          case 8: // 1층 위치이면 2층을 치면 안되니까.
          good_move();
          break;
        }

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

      case 908:// 3열로 정렬
      switch (object_position[5]) // 정렬한 포지션 값을 기준으로, 첫 번째 원소가 어느 위치인지
      {
        case 1:
        case 5: //1 또는 5번 위치이면 왼쪽으로 가는 sequence로 가기
        back_1(909);
        break;

        case 2:
        case 6:
        back_2(909);
        break;

        case 3:
        case 7:
        m_seq=909;
        break;

        case 4:
        case 8:
        back_4(909);
        break;
      
        default:
          m_seq = 230;
          break;
      }
      break;
      
      case 909://y축 직진...(미션 goal 지점방향으로)
      go_goal(910);
      break;
      
      case 910: //로봇 팔 회전... (2번째 미션 goal지점으로)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 150) { //wait for 1sec(100*10msec)
          m_seq = 911;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[5])////////////////////////////////////미션받고 drop_n추가////////////////////
        {
          case 1: // 원소가 빨간색이면
            drop_1();
            break;

          case 2:
            drop_1();
            break;
          case 3:
            drop_1();
            break;

          case 4:
            drop_1();
            break;

          case 5:
            drop_1();
            break;

          case 6:
            drop_1();
            break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;

    case 911: //그리퍼 오픈... (2번째 미션 goal지점)
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 90) { //wait for 1sec(100*10msec)
          m_seq = 24;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        switch (mission[5])  
        {
          case 1: // 원소가 빨간색이면
            ServoMove(Serial2, 5, 300, 2000); ///////////////////그리퍼 open값//////////////
            delay(2000);
            break;

          case 2:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;
          case 3:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 4:
            ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 5:
          ServoMove(Serial2, 5, 300, 2000);
            delay(2000);
            break;

          case 6:
          ServoMove(Serial2, 5, 300, 2000);
          delay(2000);
          break;
          default: break;
        }
        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;
    //------------------------------------------------- 6 번째 물체 이동 완료 --------------------------------------------------------------------------


    //-------------------------------------------------ending----------------------------------------------
    case 24: //로봇 팔 회전... 직진할 때 안 걸리게
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 60) { //wait for 1sec(100*10msec)
          m_seq = 25;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        good_move();

        wait_flag = 1;  //step 작업 완료
        set_cycle = 0;
      }
      break;


      case 25: //약간 right
      if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = 26;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else {
        if (set_cycle > 75) {  //about 0.9s, about 13.0cm
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -7;
          w_val = 0;
        }
      }
      break;
      
      case 26://ending 존으로 직진 쭉
      if (wait_flag) { //시퀀스가 완료되었다면....
        m_seq = 27;
        wait_flag = 0;
        set_cycle = 0;
      }
      else {
        if (set_cycle > 270) {  //run for 3.7 sec for 115cm////////////////시간 체크 후 변경 //////////////
          if (wait_flag == 0) {
            stop_();
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = -30;
          y_val = 0;
          w_val = 0;
        }
      }
      break;
  }
}


void psd(){
  
   dis_l = analogRead(PSD_L);
   dis_r = analogRead(PSD_R);
   dis_l = dis_l + analogRead(PSD_L);
   dis_r = dis_r + analogRead(PSD_R);
    dis_l = dis_l + analogRead(PSD_L);
    dis_r = dis_r + analogRead(PSD_R);
    
    dis_avg_l = dis_l / 3;
    dis_avg_r = dis_r / 3;
    dis_avg = (dis_avg_l + dis_avg_r) / 2;
    
    dis_l = 0;
    dis_r = 0;
}

void forward(){
    pid();
    m1_ref_spd = 5;
    m2_ref_spd = 5;
    m3_ref_spd = 5;
    m4_ref_spd = 5;
}

void right(){

    pid();
    m1_ref_spd = 5;
    m2_ref_spd = -5;
    m3_ref_spd = 5;
    m4_ref_spd = -5;
   }


void left(){
    pid();
    m1_ref_spd = -5;
    m2_ref_spd = 5;
    m3_ref_spd = -5;
    m4_ref_spd = 5;
}

void turn_l(){
   time = millis();
   while(time +1700 > millis()){
    pid();
    m1_ref_spd = -5;
    m2_ref_spd = -5;
    m3_ref_spd = 5;
    m4_ref_spd = 5;
}
}
void turn_r(){
                 time = millis();
   while(time +1700 > millis()){


    pid();
    m1_ref_spd = 5;
    m2_ref_spd = 5;
    m3_ref_spd = -5;
    m4_ref_spd = -5;
}
}
void stop_(){
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}

void T5ISR(){
    set_cycle++;
    psd();
   // Serial.println(dis_avg);
//    Serial.print(analogRead(A6));
//    Serial.print("             ");
//    Serial.print(analogRead(A7));
//    Serial.print("          ");
    Serial.println(m_seq);
    Serial.println(object_position[0]);
//Serial.println(dis_avg);
Serial.print("mission:    ");
Serial.print(mission[0]);
Serial.print("NUM:  ");
Serial.print(NUM_OBJECT);
Serial.print("detect:    ");
Serial.println(detect_blocks);
    t5_flag = true;
    e1cnt_k = e1cnt;
    e2cnt_k = e2cnt;
    e3cnt_k = e3cnt;
    e4cnt_k = e4cnt;
        
    d_e1cnt = e1cnt_k - e1cnt_k_1;
    d_e2cnt = e2cnt_k - e2cnt_k_1;
    d_e3cnt = e3cnt_k - e3cnt_k_1;
    d_e4cnt = e4cnt_k - e4cnt_k_1;

                    // Store current value in past value
    e1cnt_k_1 = e1cnt_k;
    e2cnt_k_1 = e2cnt_k;
    e3cnt_k_1 = e3cnt_k;
    e4cnt_k_1 = e4cnt_k;
}
//=========================================적재함 팔 위치==================(그리퍼값은 조정(닫힌값으로) x)=========
void drop_1(){
  dae_Servomove(935,450,250,300,620, 2000);
  delay(2000);
}
void drop_2(){
  dae_Servomove(870,400,200,260,620,2000);
  delay(2000);
}
void drop_3(){
  dae_Servomove(810,420,250,400,620,2000);
  delay(2000);
}
void drop_4(){
  dae_Servomove(790,307,63,360,620,2000);
  delay(2000);
}
void drop_5(){
  dae_Servomove(1000,300,00,350,620,2000);
  delay(2000);
}
void drop_6(){
  dae_Servomove(880,300,000,350,620,2000);
  delay(2000);
}
void drop_7(){
  dae_Servomove(780,300,000,350,620,2000);
  delay(2000);
}
void drop_8(){
  dae_Servomove(710,380,150,300,300,2000);
  delay(2000);
}

//=========================미션존 1층, 2층=====================================
void grip_1(){
   dae_Servomove(500,370,70,200,620,2000);
   delay(2000);
}
void grip_2(){
   dae_Servomove(500,260,100,320,620,2000);
   delay(2000);
}

void no_grip_1(){
   dae_Servomove(500,370,70,200,300,2000);
   delay(2000);
}
void no_grip_2(){
   dae_Servomove(500,260,100,320,300,2000);
   delay(2000);
}

//=====================================이동시 편한 로봇팔 위치===============================4
void good_move(){
   dae_Servomove(500,200,500,500,620,2000);
   delay(2000); 
}
//=====================================물체 잡고 직진,후진===================================
void go_grip(int m_seq_n) {
  if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = m_seq_n;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
     else {
        if (set_cycle > 100) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {////////////////aaaa
          pid();
          x_val = 14;
          y_val = 0;
          w_val = 0;
        }
      }
}

void back_grip(int m_seq_n) {
  if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = m_seq_n;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
     else {
        if (set_cycle > 100) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {////////////////aaaa
          pid();
          x_val = -14;
          y_val = 0;
          w_val = 0;
        }
      }
}

//=========================================물체 집고 넣으러 go// 넣고 다시 back ================================
void go_goal(int m_seq_n){
        if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = m_seq_n;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
     else {
        if (set_cycle > 150) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {////////////////aaaa
          pid();
          x_val = 0;
          y_val = 15;
          w_val = 1.3;
        }
      }
      
}

void back_goal(int m_seq_n){ //goal ->3열로
        if (wait_flag) { //시퀀스가 완료되었다면....
        if (set_cycle > 100) { //wait for 1sec(100*10msec)
          m_seq = m_seq_n;
          set_cycle = 0;
          wait_flag = 0;
        }
      }
     else {
        if (set_cycle > 150) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {////////////////aaaa
          pid();
          x_val = 0;
          y_val = -15;
          w_val = 0;
        }
      }
      
}
//=========================================초기지점에서 (1,5)(2,6)(3,7)(4,8) 위치로=============
void go_1(int m_seq_n){
if (wait_flag) {
        if (set_cycle > 90) {
          m_seq = m_seq_n;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 100) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 15;
          w_val = 0;
        }
      }
}
void go_2(int m_seq_n){
  if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = m_seq_n;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 50) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 15;
          w_val = 0;
        }
      }

}


void go_4(int m_seq_n){
  if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = m_seq_n; //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 70) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -11;
          w_val = 0;
        }
      }
}
//=====================================3열 앞으러 정렬 ==================================
void back_1(int m_seq_n){
if (wait_flag) {
        if (set_cycle > 90) {
          m_seq = m_seq_n;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 100) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -15;
          w_val = 0;
        }
      }
}
void back_2(int m_seq_n){
  if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = m_seq_n;  //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 50) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = -15;
          w_val = 0;
        }
      }

}


void back_4(int m_seq_n){
  if (wait_flag) {
        if (set_cycle > 100) {
          m_seq = m_seq_n; //next step, 로봇 팔움직이는 seq로 넘어가기
          set_cycle = 0;
          wait_flag = 0;
        }
      }
      else
      {
        if (set_cycle > 70) {  //about 0.7s
          stop_();
          if (wait_flag == 0) {
            set_cycle = 0;
            wait_flag = 1;
          }
        }
        else {
          pid();
          x_val = 0;
          y_val = 10;
          w_val = 0;
        }
      }
}
//==================================================위치 측정 후 변경===================

void pid(){
  if(t5_flag){
        t5_flag = 0;

    m1speed = d_e1cnt * 17/10;      // * 1200/706.806
    m2speed = d_e2cnt * 17/10;
    m3speed = d_e3cnt * 17/10;
    m4speed = d_e4cnt * 17/10;
  

   //================================================//
     //Robot Control
     MecanumVelocity(x_val, y_val, w_val); //vx[cm/s], vy[cm/s], a_vel[deg/s]
     Motor1Control(m1_ref_spd);
     Motor2Control(m2_ref_spd);
     Motor3Control(m3_ref_spd);
     Motor4Control(m4_ref_spd);
     parameter_clr(); 
  
  }
}

//for Encoder 1 interrupt
void Enc1chA_ISR() {
  if (digitalRead(ENC1_CHA) == HIGH) {
    if (digitalRead(ENC1_CHB) == LOW) e1cnt--;
    else                             e1cnt++;
  }
  else {
    if (digitalRead(ENC1_CHB) == HIGH) e1cnt--;
    else                              e1cnt++;
  }
}

//for Encdoer 2 interrupt
void Enc2chA_ISR() {
  if (digitalRead(ENC2_CHA) == HIGH) {
    if (digitalRead(ENC2_CHB) == LOW) e2cnt--;
    else                             e2cnt++;
  }
  else {
    if (digitalRead(ENC2_CHB) == HIGH) e2cnt--;
    else                              e2cnt++;
  }
}

//for Encdoer 3 interrupt
void Enc3chA_ISR() {
  if (digitalRead(ENC3_CHA) == HIGH) {
    if (digitalRead(ENC3_CHB) == LOW) e3cnt++;
    else                             e3cnt--;
  }
  else {
    if (digitalRead(ENC3_CHB) == HIGH) e3cnt++;
    else                              e3cnt--;
  }
}

//for Encdoer 4 interrupt
void Enc4chA_ISR() {
  if (digitalRead(ENC4_CHA) == HIGH) {
    if (digitalRead(ENC4_CHB) == LOW) e4cnt++;
    else                             e4cnt--;
  }
  else {
    if (digitalRead(ENC4_CHB) == HIGH) e4cnt++;
    else                              e4cnt--;
  }
}

//Motor Control
//motor1
void Motor1Control(float m1_ref_spd) {
  //Error
  m1_err_spd = m1_ref_spd - m1speed;
  m1_derr_spd = m1_err_spd - m1_err_spd_k_1;
  m1_err_sum = m1_err_sum + m1_err_spd;

  m1_err_spd_k_1 = m1_err_spd;

  //PID-Controller
  m1_ctrl_up = Kp1 * m1_err_spd;
  m1_ctrl_ui = Ki1 * m1_err_sum;
  m1_ctrl_ud = Kd1 * m1_derr_spd;

  m1_ctrl_u = (int)(m1_ctrl_up + m1_ctrl_ud + m1_ctrl_ui);
  if (m1_ref_spd == 0) m1_ctrl_u = 0;
  if (m1_ctrl_u >= 0) {
    digitalWrite(M1_I1, HIGH);  //M1: I1(H), I2(L) -> Forward
    digitalWrite(M1_I2, LOW);
    if (m1_ctrl_u > 255) m1_ipwm_u = 255;
    else m1_ipwm_u = (int)m1_ctrl_u;
  }
  else {
    digitalWrite(M1_I1, LOW);  //M1: I1(L), I2(H) -> Backward
    digitalWrite(M1_I2, HIGH);
    if (m1_ctrl_u < -255) m1_ipwm_u = 255;
    else m1_ipwm_u = (int)m1_ctrl_u * (-1);
  }
  analogWrite(M1_PWM, m1_ipwm_u);    //motor input
}
//motor2
void Motor2Control(float m2_ref_spd) {
  //Error
  m2_err_spd = m2_ref_spd - m2speed;
  m2_derr_spd = m2_err_spd - m2_err_spd_k_1;
  m2_err_sum = m2_err_sum + m2_err_spd;

  m2_err_spd_k_1 = m2_err_spd;

  //PID-Controller
  m2_ctrl_up = Kp2 * m2_err_spd;
  m2_ctrl_ui = Ki2 * m2_err_sum;
  m2_ctrl_ud = Kd2 * m2_derr_spd;

  m2_ctrl_u = (int)(m2_ctrl_up + m2_ctrl_ud + m2_ctrl_ui);
  if (m2_ref_spd == 0) m2_ctrl_u = 0;
  if (m2_ctrl_u >= 0) {
    digitalWrite(M2_I1, HIGH);  //M2: I1(H), I2(L) -> Forward
    digitalWrite(M2_I2, LOW);
    if (m2_ctrl_u > 255) m2_ipwm_u = 255;
    else m2_ipwm_u = (int)m2_ctrl_u;
  }
  else {
    digitalWrite(M2_I1, LOW);  //M2: I1(L), I2(H) -> Backward
    digitalWrite(M2_I2, HIGH);
    if (m2_ctrl_u < -255) m2_ipwm_u = 255;
    else m2_ipwm_u = (int)m2_ctrl_u * (-1);
  }
  analogWrite(M2_PWM, m2_ipwm_u);    //motor input
}
//motor3
void Motor3Control(float m3_ref_spd) {
  //Error
  m3_err_spd = m3_ref_spd - m3speed;
  m3_derr_spd = m3_err_spd - m3_err_spd_k_1;
  m3_err_sum = m3_err_sum + m3_err_spd;

  m3_err_spd_k_1 = m3_err_spd;

  //PID-Controller
  m3_ctrl_up = Kp3 * m3_err_spd;
  m3_ctrl_ui = Ki3 * m3_err_sum;
  m3_ctrl_ud = Kd3 * m3_derr_spd;

  m3_ctrl_u = (int)(m3_ctrl_up + m3_ctrl_ud + m3_ctrl_ui);
  if (m3_ref_spd == 0) m3_ctrl_u = 0;
  if (m3_ctrl_u >= 0) {
    digitalWrite(M3_I1, HIGH);  //M3: I1(H), I2(L) -> Forward
    digitalWrite(M3_I2, LOW);
    if (m3_ctrl_u > 255) m3_ipwm_u = 255;
    else m3_ipwm_u = (int)m3_ctrl_u;
  }
  else {
    digitalWrite(M3_I1, LOW);  //M3: I1(L), I2(H) -> Backward
    digitalWrite(M3_I2, HIGH);
    if (m3_ctrl_u < -255) m3_ipwm_u = 255;
    else m3_ipwm_u = (int)m3_ctrl_u * (-1);
  }
  analogWrite(M3_PWM, m3_ipwm_u);    //motor input
}
//motor4
void Motor4Control(float m4_ref_spd) {
  //Error
  m4_err_spd = m4_ref_spd - m4speed;
  m4_derr_spd = m4_err_spd - m4_err_spd_k_1;
  m4_err_sum = m4_err_sum + m4_err_spd;

  m4_err_spd_k_1 = m4_err_spd;

  //PID-Controller
  m4_ctrl_up = Kp4 * m4_err_spd;
  m4_ctrl_ui = Ki4 * m4_err_sum;
  m4_ctrl_ud = Kd4 * m4_derr_spd;

  m4_ctrl_u = (int)(m4_ctrl_up + m4_ctrl_ud + m4_ctrl_ui);
  if (m4_ref_spd == 0) m4_ctrl_u = 0;
  if (m4_ctrl_u >= 0) {
    digitalWrite(M4_I1, HIGH);  //M4: I1(H), I2(L) -> Forward
    digitalWrite(M4_I2, LOW);
    if (m4_ctrl_u > 255) m4_ipwm_u = 255;
    else m4_ipwm_u = (int)m4_ctrl_u;
  }
  else {
    digitalWrite(M4_I1, LOW);  //M4: I1(L), I2(H) -> Backward
    digitalWrite(M4_I2, HIGH);
    if (m4_ctrl_u < -255) m4_ipwm_u = 255;
    else m4_ipwm_u = (int)m4_ctrl_u * (-1);
  }
  analogWrite(M4_PWM, m4_ipwm_u);    //motor input
}

// motor parameter clear
void parameter_clr() {
  //Variables
  //Encoder value
  e1cnt = 0;
  e1cnt_k = 0;
  e1cnt_k_1 = 0;
  d_e1cnt = 0;

  e2cnt = 0;
  e2cnt_k = 0;
  e2cnt_k_1 = 0;
  d_e2cnt = 0;

  e3cnt = 0;
  e3cnt_k = 0;
  e3cnt_k_1 = 0;
  d_e3cnt = 0;

  e4cnt = 0;
  e4cnt_k = 0;
  e4cnt_k_1 = 0;
  d_e4cnt = 0;

  //motor value
  m1speed = 0;
  m2speed = 0;
  m3speed = 0;
  m4speed = 0;

  //for motor control variable
  //motor1
  m1_ref_spd = 0;
  m1_err_spd = 0;
  m1_err_spd_k_1 = 0;
  m1_derr_spd = 0;
  m1_err_sum = 0;
  m1_ctrl_up = 0;
  m1_ctrl_ui = 0;
  m1_ctrl_ud = 0;
  m1_ctrl_u = 0;
  m1_ipwm_u = 0;
  //motor2
  m2_ref_spd = 0;
  m2_err_spd = 0;
  m2_err_spd_k_1 = 0;
  m2_derr_spd = 0;
  m2_err_sum = 0;
  m2_ctrl_up = 0;
  m2_ctrl_ui = 0;
  m2_ctrl_ud = 0;
  m2_ctrl_u = 0;
  m2_ipwm_u = 0;
  //motor3
  m3_ref_spd = 0;
  m3_err_spd = 0;
  m3_err_spd_k_1 = 0;
  m3_derr_spd = 0;
  m3_err_sum = 0;
  m3_ctrl_up = 0;
  m3_ctrl_ui = 0;
  m3_ctrl_ud = 0;
  m3_ctrl_u = 0;
  m3_ipwm_u = 0;
  //motor4
  m4_ref_spd = 0;
  m4_err_spd = 0;
  m4_err_spd_k_1 = 0;
  m4_derr_spd = 0;
  m4_err_sum = 0;
  m4_ctrl_up = 0;
  m4_ctrl_ui = 0;
  m4_ctrl_ud = 0;
  m4_ctrl_u = 0;
  m4_ipwm_u = 0;
}

void MecanumVelocity(float vx, float vy, float dpi) {
  //vx, vy(robot velocity) unit -> cm/sec
  //dpi(robot angular velocity) unit -> deg/sec
  float v1, v2, v3, v4;
  float drot;

  //[deg/sec] to [rad/sec] :( *pi/180)--> cm(rad)/sec
  dpi *= 31.41593;
  dpi /= 18;

  drot = LPLUSD * dpi;
  v1 = vx - vy - drot;
  v2 = vx + vy - drot;
  v3 = vx - vy + drot;
  v4 = vx + vy + drot;

  //wheel linear velocity to wheel speed
  m1_ref_spd = v1 * 10 / 3.141593;
  m2_ref_spd = v2 * 10 / 3.141593;
  m3_ref_spd = v3 * 10 / 3.141593;
  m4_ref_spd = v4 * 10 / 3.141593;
}

///// for Pixy
void SortBlocks() { //인식한 물체들을 x좌표 순서대로 정렬하는 알고리즘.
  for (int i = NUM_OBJECT - 1 ; i > 0 ; i--)
  {
    for (int j = 0; j < i; j++)
    {
      if (x_value[j] > x_value[j + 1])
      {
        int temp = 0;
        temp = mission[j];
        mission[j] = mission[j + 1];
        mission[j + 1] = temp;
        int temp_x = 0;
        temp_x = x_value[j];
        x_value[j] = x_value[j + 1];
        x_value[j + 1] = temp_x;
      }
    }
  }
}

void SortBlocks_row(int sarray[][NUM_OBJECT], int rnum, int cnum) {
  //2차원 배열 중 '행' 값에 대해 오름차순으로 정렬
  //모든 열에 대해서 반복(변수 col)

  int rindex;
  int cindex;

  rindex = rnum;
  cindex = cnum;
  for (int col = 0; col < cindex; col++)
  {
    for (int i = rindex - 1; i > 0 ; i--) // 버블정렬 수행
    {
      for (int j = 0; j < i; j++)
      {
        if (sarray[j][col] > sarray[j + 1][col])
        {
          int temp = sarray[j][col];
          sarray[j][col] = sarray[j + 1][col];
          sarray[j + 1][col] = temp;
        }
      }
    }
  }
}

void positioning(int x[], int y[]) {
  for (int h = 0; h < NUM_OBJECT; h++) {
    if (y[h] < 110) { //8개의 칸 중 윗층
      if (x[h] < 94) {
        object_position[h] = 1; //1번 칸
        /*
          digitalWrite(LED_R, HIGH);
          delay(500);
          digitalWrite(LED_R, LOW);
          delay(500);
        */
      }
      if (94 < x[h] && x[h] < 160) {
        object_position[h] = 2; //2번 칸
        /*
          digitalWrite(LED_G, HIGH);
          delay(500);
          digitalWrite(LED_G, LOW);
          delay(500);
        */
      }
      if (160 < x[h] && x[h] < 221) {
        object_position[h] = 3; //3번 칸
        /*
          digitalWrite(LED_B, HIGH);
          delay(500);
          digitalWrite(LED_B, LOW);
          delay(500);
        */
      }
      if (221 < x[h]) {
        object_position[h] = 4; //4번 칸
        /*
          digitalWrite(LED_R, HIGH);
          digitalWrite(LED_G, HIGH);
          digitalWrite(LED_B, HIGH);
          delay(500);
          digitalWrite(LED_R, LOW);
          digitalWrite(LED_G, LOW);
          digitalWrite(LED_B, LOW);
          delay(500);
        */
      }
    }

    if (110 < y[h]) { //8개의 칸 중 아래층
      if (213 < x[h]) {
        object_position[h] = 8; //8번 칸
        /*
          digitalWrite(LED_R, HIGH);
          digitalWrite(LED_G, HIGH);
          digitalWrite(LED_B, HIGH);
          delay(1500);
          digitalWrite(LED_R, LOW);
          digitalWrite(LED_G, LOW);
          digitalWrite(LED_B, LOW);
          delay(500);
        */
      }
      if (160 < x[h] && x[h] < 213) {
        object_position[h] = 7; //7번 칸
        /*
          digitalWrite(LED_B, HIGH);
          delay(1500);
          digitalWrite(LED_B, LOW);
          delay(500);
        */
      }
      if (101 < x[h] && x[h] < 160) {
        object_position[h] = 6; //6번 칸
        /*
          digitalWrite(LED_G, HIGH);
          delay(1500);
          digitalWrite(LED_G, LOW);
          delay(500);
        */
      }
      if (x[h] < 101) {
        object_position[h] = 5; //5번 칸
        /*
          digitalWrite(LED_R, HIGH);
          delay(1500);
          digitalWrite(LED_R, LOW);
          delay(500);
        */
      }
    }
  }
}

void dae_Servomove(int id11, int id2 , int id3 , int id4 , int id5,int postime){
  ServoMove(Serial2, 11,id11 , postime);
  ServoMove(Serial2, 2,id2 , postime);
  ServoMove(Serial2, 3,id3 , postime);
  ServoMove(Serial2, 4,id4 , postime);
  ServoMove(Serial2, 5,id5 , postime);
}
