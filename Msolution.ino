#include <EEPROM.h>


//LED PIN
#define LED1 31
#define LED2 33
#define LED3 35

//Motor // M1 => Y축 // M2 => X축
#define M1_pwm 5     //M1 속도제어 핀
#define M1_dir 4     //M1 방향제어 핀
#define M2_pwm 6     //M2 속도제어 핀
#define M2_dir 7     //M2 방향제어 핀

//Switch
#define buttonPin_1 14  //Button Input
#define buttonPin_2 15  //Button Input
#define buttonPin_3 16  //Button Input

//Sensor
#define limit_S_x 12 // x축 원점을 찾기 위한 limit 스위치
#define limit_S_y 13 // y축 원점을 찾기 위한 limit 스위치


//Encoder1 => Y축 / Encoder2 = X축
//인터럽트 실질적 핀번호
#define PhaseA 21 // Encoder1의 A상   //Y     
#define PhaseB 20 // Encoder1의 B상   //Y
#define PhaseA_2 19 // Encoder2의 A상 //X
#define PhaseB_2 18 // Encoder2의 B상 //X

//인터럽트 번호
#define interrupt_A 2                         
#define interrupt_B 3
#define interrupt_A_2 4
#define interrupt_B_2 5

//정방향 도중 멈추기위해 잠시거는 역방향 delay 값
#define stop_pulse 50 

// 각 홀의 위치 엔코더 값 
#define Hole_1 190
#define Hole_2 710
#define Hole_3 1000   //1230

//반복횟수(M1 => 1회 / M2 => 2회 / M3 => 3회)
#define repeat_motion_1 1
#define repeat_motion_2 2
#define repeat_motion_3 4

//예약버퍼(배열)
#define Botton_Buf_num 6                     
   
#define X_OFF 50
#define Y_OFF 50








// 원두 불리는 대기 시간 관련 변수
String inputString = ""; // 문자열을 입력 받기 위한 변수
boolean stringComplete = false; // 입력 성공&실패
unsigned int receive_time_data = 0; // 원두 불리는 대기 시간 - 설정하지 않으면 0초
boolean receive_flag = false;
char inChar = ' ';
boolean Time_flag = 0;
int Timer_Value = 30;        //디폴트ㅋ


int i_num = 0;                                  //예약
int Botton_Buf[Botton_Buf_num] = {0,};          //예약
int Botton_Buf1[Botton_Buf_num] = {0,};         //예약


// limit_Switch 제어 변수
boolean LimitState_x = 0;                       //Limit Switch
boolean lastLimitState_x = 0;
boolean x_lim_flag = true;

boolean LimitState_y = 0;
boolean lastLimitState_y = 0;
boolean y_lim_flag = true;                      //Limit Switch


// 각 홀의 위치 제어
int Hole_state = 0;                             //각 Hole 원점 받아오는 변수
int Hole_state_last = 0;                        
int err = 0;                                    // 위치 이동시 오차


//스위치 상태변경시 전달을위해 필요한 변수
boolean buttonState_1 = 0;
boolean lastButtonState_1 = 0;
boolean buttonState_2 = 0;
boolean lastButtonState_2 = 0;
boolean buttonState_3 = 0;
boolean lastButtonState_3 = 0;


//모션 상태 저장 (모션 1,2,3)
unsigned int motion_state_1 = 0;          //Hole1_Motion_Count
unsigned int motion_state_2 = 0;          //Hole2_Motion_Count
unsigned int motion_state_3 = 0;          //Hole3_Motion_Count

unsigned int pos = 0;                     // 몇번홀의 스위치로 이동하는지
unsigned int i_flag = 0;                  // 모션 반복을 위한 flag


int motor1_du_bspeed = 150;               //M1 PWM 듀티비 속도조절
int motor1_du_fspeed = 140;               //M1 PWM 듀티비 속도조절


int Timer_CounterA = 0;                   //Timer_Count_Value
int Timer_CounterB = 0;                   //Timer_Count_Value
int Timer_CounterC = 0;                   //Timer_Count_Value

int Timer_Control_Flag1 = 0;              //Timer_Count_Flag
int Timer_Control_Flag2 = 0;              //Timer_Count_Flag
int Timer_Control_Flag3 = 0;              //Timer_Count_Flag
int Total_Contral_Flag = 0;

//모션제어// 
int y_pos = 0;                            //목표 위치
int x_pos = 0;                            //목표 위치
int y_pos_err = 0;                        // 목표 위치와 현 위치 차이
int x_pos_err = 0;                        // 목표 위치와 현 위치 차이

int motor1_fspeed = 100;                  //앞으로
int motor1_bspeed = 110;                  //뒤로

int motor2_bspeed = 100;                  //앞으로     
int motor2_fspeed = 110;                  //뒤로   
////////////



int spiral_x_position[41] = { 0,11, 0,-16,-28,-22,  0,28,  44, 41, 26,  0,-30,-55,-68, -62,  -38,   0,  41,76, 92, 76, 41,  0,  -38,-62,-68, -55, -30,   0, 26,41,  44,28, 0,-22,-28,-16,  0,11, 0};
int spiral_y_position[41] = {28,50,42,  0,-56,-90,-70, 0,  59,112,140,129, 80,  0,-90,-164, -200,-181,-110, 0,110,181,200,164,   90,  0,-80,-129,-140,-112,-59, 0,  70,90,56,  0,-42,-50,-28, 0};
int subpt[40]             = {11, 6,21, 27, 16, 14, 35,29,  25, 15, 13, 27, 39, 42, 35,  20,   20,  38,  54,52, 34, 19, 25, 39,   43, 37, 24,  13,  19,  28, 28,33,  12,21,28, 20,  7, 13, 14, 5 };


unsigned long old_time=0, new_time, time_cnt=0;


//LED
int LED_Flag1 = 0;
int LED_Flag2 = 0;
int LED_Flag3 = 0;
unsigned long Loop_Cnt1 = 0;
unsigned long Loop_Cnt2 = 0;
unsigned long Loop_Cnt3 = 0;

int Operating_But = 0;
int Operating_But1 = 0;
int i;
int Button_flag = 0;
int Timer_FLAG = 0;
int subi = 0, subn, ii = 0;



// Encoder의 값을 읽어들일 변수 (TEST)
volatile signed int count = 0, oldcount = -1; 
volatile signed int count_2 = 0, oldcount2 = -1;


// Encoder_1_STATE
enum encoder_state
{
  State_0,
  State_1,
  State_2,
  State_3,
} ENCODER_STATE = State_0;

// Encoder_2_STATE
enum encoder_state_2
{
  State_4,
  State_5,
  State_6,
  State_7
} ENCODER_STATE_2 = State_4;

//Timer_Configuration_STATE
enum r_state
{
  R_INIT,
  R_T1,
  R_T2,
  R_T3
} R_STATE;

//Main_STATE (Motion)
enum Hole_State
{
  HH_Motion_IDLE,   //Motion_IDLE
  H1_Motion_M1,      
  H1_Motion_M2,
  H1_Motion_M3,     //H1
  H2_Motion_M1,      
  H2_Motion_M2,
  H2_Motion_M3,     //H2
  H3_Motion_M1,      
  H3_Motion_M2,
  H3_Motion_M3,     //H3
} SUB_MOTION_STATE = HH_Motion_IDLE;

void setup(void) 
{
  pinMode(buttonPin_1, INPUT); // 1번 스위치 핀 모드설정
  pinMode(buttonPin_2, INPUT); // 2번 스위치 핀 모드설정
  pinMode(buttonPin_3, INPUT); // 3번 스위치 핀 모드설정


  pinMode(LED1, OUTPUT); // 1번 led설정
  pinMode(LED2, OUTPUT); // 2번 led설정
  pinMode(LED3, OUTPUT); // 3번 led설정
  digitalWrite(LED1,LOW);  
  digitalWrite(LED2,LOW);  
  digitalWrite(LED3,LOW); 
 //////////////////////// 모터 1 제어 핀 /////////////////////////// 
  pinMode(M1_pwm, OUTPUT);  
  pinMode(M1_dir, OUTPUT);   
  digitalWrite(M1_pwm,LOW);  
   
 //////////////////////// 모터 2 제어 핀 /////////////////////////// 
  pinMode(M2_pwm, OUTPUT);  
  pinMode(M2_dir, OUTPUT);  
  digitalWrite(M2_pwm,LOW);   

  //////////////// ENCODER_1 핀 ////////////////////////
  pinMode(PhaseA, INPUT);
  pinMode(PhaseB, INPUT);

  //////////////// ENCODER_2 핀 ////////////////////////
  pinMode(PhaseA_2, INPUT);
  pinMode(PhaseB_2, INPUT);

  // 초기화
  ENCODER_STATE = State_0;
  ENCODER_STATE_2 = State_4;
  R_STATE = R_INIT;

  // ENCODER_1 interrupt
  attachInterrupt(interrupt_A, ISR_0, CHANGE);
  attachInterrupt(interrupt_B, ISR_1, CHANGE);

  // ENCODDER_2 interrupt
  attachInterrupt(interrupt_A_2, ISR_2, CHANGE);
  attachInterrupt(interrupt_B_2, ISR_3, CHANGE);


  //Timer/Counter Register 
  TIMSK1 = 0<<OCIE1A;  // 비교일치 인터럽트 불허    
  TCCR1A = 1<<COM1A0;  // 비교일치에서 출력 Toggle
  TCCR1B = (1<<WGM12) | (1<<CS12); // CTC모드, 256분주


  TIMSK1 &= 0<<OCIE1B;  // 비교일치 인터럽트 허가
  TCCR1A |= 1<<COM1B0;  // 비교일치에서 출력 Toggle

  TIMSK1 &= 0<<OCIE1C;  // 비교일치 인터럽트 허가
  TCCR1A |= 1<<COM1C0;  // 비교일치에서 출력 Toggle

  OCR1AH = 0xF4;  OCR1AL = 0x24;  // OCR = 0xF424 = 62500
  OCR1BH = 0xF4;  OCR1BL = 0x24;  // OCR = 0xF424 = 62500
  OCR1CH = 0xF4;  OCR1CL = 0x24;  // OCR = 0xF424 = 62500
  
  inputString.reserve(200); // 데이터를 입력 받을 공간 확보

  Serial.begin(115200);
  Serial.println("Motor position : ");
}

void Reserve_Function()
{

  //각 홀이 모션2 + 타이머동작 => 타이머동작시 다른 모션이 끼어들어도 같은모션이 끼어들면 안됨
  if((SUB_MOTION_STATE == H1_Motion_M2) || (Timer_CounterA != 0)){ Button_flag = 1; }
  if((SUB_MOTION_STATE == H2_Motion_M2) || (Timer_CounterB != 0)){ Button_flag = 2; }
  if((SUB_MOTION_STATE == H3_Motion_M2) || (Timer_CounterC != 0)){ Button_flag = 3; }

  while(1) //반복문인 이유는 i_num을 찾기위해서 반복도는 횟수 많아야 5(예약버퍼가 6이라고 가정하면)
  {  
    if(Botton_Buf[i_num]!=0){   //해당버퍼(i_num)에 값이 있다면
      i_num++;                  //다음버퍼[i_num+1]을 체크하기위해 i_num++;
    }
    else if((Botton_Buf[i_num] == 0) && (i_num<Botton_Buf_num-1))
    {        

      Botton_Buf[i_num] = pos;                                      //해당 Hole   버퍼에 저장
      
      if(pos == 1)     {    Botton_Buf1[i_num] = motion_state_1; }  //해당 Motion 버퍼에 저장
      else if(pos == 2){    Botton_Buf1[i_num] = motion_state_2; }  //해당 Motion 버퍼에 저장
      else if(pos == 3){    Botton_Buf1[i_num] = motion_state_3; }  //해당 Motion 버퍼에 저장


      //모션2가 예약되어있고, 홀이 연속적으로 같을때는 해당버퍼 초기화
      if((i_num != 0) && (Botton_Buf1[i_num-1] == 2) && (Botton_Buf[i_num-1] == Botton_Buf[i_num])){
          Botton_Buf[i_num] = 0;                //버퍼를 0으로 초기화
          Botton_Buf1[i_num] = 0;               
                   
          if(pos == 1)       motion_state_1--;  //motion_state값은 Button_Input에서 지정되므로 -1필요
          else if(pos == 2)  motion_state_2--;  
          else if(pos == 3)  motion_state_3--;        
      }

      //타이머인터럽트발생 + 모션2 ==> Button_Flag = N
      //Button_flag가 N이고, 버퍼에 N이 저장, 초기화 
      if((Button_flag==1)&&(Botton_Buf[i_num]==1)){ Button_flag = 0; Botton_Buf[i_num] = 0; Botton_Buf1[i_num] = 0; }
      if((Button_flag==2)&&(Botton_Buf[i_num]==2)){ Button_flag = 0; Botton_Buf[i_num] = 0; Botton_Buf1[i_num] = 0; }  
      if((Button_flag==3)&&(Botton_Buf[i_num]==3)){ Button_flag = 0; Botton_Buf[i_num] = 0; Botton_Buf1[i_num] = 0; } 

    
      i_num = 0;     
      pos = 0;
      break;                                  
    }
  }
}


void SUB_STATE_MACHINE()
{ 
  switch(SUB_MOTION_STATE)
  {
    case HH_Motion_IDLE:      
      M1_stop();
      M2_stop();
      break;
    case H1_Motion_M1:                               
      digitalWrite(LED1,HIGH);  
      if(Operating_But == 1)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_1();
      break;        
    case H1_Motion_M2:                                
      LED_Flag1 = 1;
      if(Operating_But == 1)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_2();
      break;
    case H1_Motion_M3:                              
      if(Operating_But == 1)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_3();
      break;
    case H2_Motion_M1:                               
      digitalWrite(LED2,HIGH);  
      if(Operating_But == 2)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_1();
      break;
    case H2_Motion_M2:                              
      LED_Flag2 = 1;
      if(Operating_But == 2)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_2();
      break;
    case H2_Motion_M3:                                
      if(Operating_But == 2)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_3();
      break;    
    case H3_Motion_M1:                               
     digitalWrite(LED3,HIGH);        
     if(Operating_But == 3)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_1();
      break;
    case H3_Motion_M2:                               
      LED_Flag3 = 1;
      if(Operating_But == 3)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_2();
      break; 
    case H3_Motion_M3:                                
      if(Operating_But == 3)
          Pos_Move();
      else if(Operating_But == 0)
          Motion_3();
      break;
 }
}


void Save_Time_Value()
{
  serialEvent();
  Time_flag = EEPROM.read(1);
  Timer_Value = EEPROM.read(0);

  // 플레그 변화시 플래그 다시저장
  if( Time_flag != receive_flag ){ EEPROM.write(1, receive_flag); }

  // 플래그 Open 시에만 데이터 재입력
  if(Time_flag == true){ EEPROM.write(0, receive_time_data); }
}


void LED_BLINK_FUNC()
{ 
  if((Loop_Cnt1/500)%2 == 1)  digitalWrite(LED1,HIGH);  
  else  digitalWrite(LED1,LOW);   
  
  if((Loop_Cnt2/500)%2 == 1)  digitalWrite(LED2,HIGH);  
  else  digitalWrite(LED2,LOW);  
  
  if((Loop_Cnt3/500)%2 == 1)  digitalWrite(LED3,HIGH);  
  else  digitalWrite(LED3,LOW);     
}

// Serial.println(); TEST용
void loop(void) 
{ 
  while( x_lim_flag == true ) limit_sw_x();     //x축 센서
  while( y_lim_flag == true ) limit_sw_y();     //y축 센서

  Save_Time_Value();                            //EEPROM에 값 저장

    //마지막 버퍼가 비어있다면       //버튼상태 지정   //예약기능
  if(Botton_Buf[Botton_Buf_num-2]==0){ Button_Input(); Reserve_Function(); }


  //타이머ISR 동작중, 모션이 실행될수없는 구간 ==> 타이머 금지구간
  //현재상태가 타이머 금지구간에 있다면, 
  if(Timer_Control_Flag1 == 0 && Timer_Control_Flag2 == 0 && Timer_Control_Flag3== 0)   Total_Contral_Flag = 1;
  else Total_Contral_Flag = 0;

  //1번버퍼에 값이있고, Main_State가 IDEL이고, 타이머 금지구간이 아니라면, 예약된 버퍼값으로 STATE결정
  if((Botton_Buf[0] != 0) && (SUB_MOTION_STATE == HH_Motion_IDLE) && (Total_Contral_Flag == 1) )      // + 각 모션에서 끝났다는 시그널을 받을때
  {
    Total_Contral_Flag = 0;

    //Main_Operating_Func();      //버퍼의 값을 이용해서 STATE결정
                                  

  
    Operating_But = Botton_Buf[0];      //버퍼의 값을 Operating_Buf에 넣어준다.
    Operating_But1 = Botton_Buf1[0];    //Operating_Buf에 의해서 동작을 수행한다.  


    for(i=0;i<Botton_Buf_num-1;i++)   //버퍼값을 다른변수에 대입하고, 버퍼 전체 좌로 쉬프트
    {
      Botton_Buf[i] = Botton_Buf[i+1];        //쉬프트
      Botton_Buf1[i] = Botton_Buf1[i+1];         /////////////////////////////////////////////////////
    }
   
  
  if(Operating_But == 0) SUB_MOTION_STATE = HH_Motion_IDLE;  
  else if( (Operating_But == 1) && (Operating_But1 == 1))   SUB_MOTION_STATE = H1_Motion_M1;
  else if( (Operating_But == 1) && (Operating_But1 == 2))   SUB_MOTION_STATE = H1_Motion_M2;
  else if( (Operating_But == 2) && (Operating_But1 == 1))   SUB_MOTION_STATE = H2_Motion_M1;
  else if( (Operating_But == 2) && (Operating_But1 == 2))   SUB_MOTION_STATE = H2_Motion_M2;
  else if( (Operating_But == 3) && (Operating_But1 == 1))   SUB_MOTION_STATE = H3_Motion_M1;
  else if( (Operating_But == 3) && (Operating_But1 == 2))   SUB_MOTION_STATE = H3_Motion_M2;

  Operating_But1 = 0;     //지워?
}

  
  new_time = millis();   
  
  if(new_time != old_time) {
    SUB_STATE_MACHINE();
    old_time = new_time;
    
    if(LED_Flag1 == 1 )   Loop_Cnt1++;    //H1_LED점멸을 위한 Flag
    if(LED_Flag2 == 1 )   Loop_Cnt2++;    //H2_LED점멸을 위한 Flag
    if(LED_Flag3 == 1 )   Loop_Cnt3++;    //H3_LED점멸을 위한 Flag    
  }    


  LED_BLINK_FUNC();



}   

//////////////////////// 모터 1 방향제어 ///////////////////////////

void M1_stop(void)                    //M1_Stop
{
  digitalWrite(M1_pwm,0); 
  digitalWrite(M1_dir,LOW);   
} 
void M1_forward(char a)            //모터1 정방향_시계 (속도입력)
{
  analogWrite (M1_pwm,a);          //M1 PWM Speed Control
  digitalWrite(M1_dir,HIGH);    
}  
void M1_backward (char a)          //모터1 역방향_반시계 (속도입력)
{
  analogWrite (M1_pwm,a);
  digitalWrite(M1_dir,LOW);   
}

//////////////////////// 모터 2 방향제어 ///////////////////////////

void M2_stop(void)                    //M2_Stop
{  
  digitalWrite(M2_pwm,0); 
  digitalWrite(M2_dir,LOW);    
}   
void M2_forward(char a)            //모터2 정방향_시계 (속도입력)
{
  analogWrite (M2_pwm,a);          //M2 PWM Speed Control
  digitalWrite(M2_dir,HIGH);    
}  
void M2_backward (char a)          //모터2 역방향_반시계 (속도입력)
{
  analogWrite (M2_pwm,a);
  digitalWrite(M2_dir,LOW);   
}

void Motion_1() // 나선형1회 - 거름망 물적시기    
{  
  Spiral(repeat_motion_1);    
}
void Motion_2() // 나선형2회 - 원두 뿔리기    
{  
  Spiral(repeat_motion_2);            
}
void Motion_3() // 나선형4회 - 커피 물 내리기    
{    
  Spiral(repeat_motion_3);    
}

void Spiral(unsigned int a) // 스파이럴 모션 (반복횟수)
{  
  if(time_cnt==0) {    // Update position command for every 10ms
    if(subi == 0) subn = subpt[ii];
    
    // Y축에 의존한 X축 제어
    y_pos = (spiral_y_position[ii]*(subn-subi) + spiral_y_position[ii+1]*subi)/subn + Hole_state;
    x_pos = (spiral_x_position[ii]*(subn-subi) + spiral_x_position[ii+1]*subi)/subn;
    subi++;
    if(subi >= subn) {
      subi = 0;  
      ii++;
      if(ii >= 40) {
       if( i_flag == a )
       {  
         M1_stop();  
         M2_stop();

         if(SUB_MOTION_STATE == H1_Motion_M1)              digitalWrite(LED1,LOW);
         if(SUB_MOTION_STATE == H2_Motion_M1)              digitalWrite(LED2,LOW);  
         if(SUB_MOTION_STATE == H3_Motion_M1)              digitalWrite(LED3,LOW);  

         if(SUB_MOTION_STATE == H1_Motion_M2)            TIMSK1 |= 1<<OCIE1A;
         if(SUB_MOTION_STATE == H2_Motion_M2)            TIMSK1 |= 1<<OCIE1B;
         if(SUB_MOTION_STATE == H3_Motion_M2)            TIMSK1 |= 1<<OCIE1C;
         

         if(SUB_MOTION_STATE == H1_Motion_M3)            {LED_Flag1 = 0; Loop_Cnt1 = 0;}
         if(SUB_MOTION_STATE == H2_Motion_M3)            {LED_Flag2 = 0; Loop_Cnt2 = 0;}
         if(SUB_MOTION_STATE == H3_Motion_M3)            {LED_Flag3 = 0; Loop_Cnt3 = 0;}
        
         SUB_MOTION_STATE = HH_Motion_IDLE;
         ii = 0; i_flag = 0;
         return;
       }   
       else 
       {      
         ii = 0;
         i_flag = (i_flag+1);
       }  
    }
    }
  }
  time_cnt++;
  if(time_cnt >= 4) time_cnt = 0;




    x_pos_err = 3*( x_pos - count_2 );
    y_pos_err = 2*( y_pos - count );  
    if(x_pos_err > 0) {
       x_pos_err += X_OFF;
       if(x_pos_err > motor2_fspeed)  x_pos_err = motor2_fspeed;
       M2_forward(x_pos_err);   
       if(y_pos_err > 0) {
          y_pos_err += Y_OFF;
          if(y_pos_err > motor1_fspeed)  y_pos_err = motor1_fspeed;
           M1_forward(y_pos_err);  
       } else {
          y_pos_err = -y_pos_err;
          y_pos_err += Y_OFF;
          if(y_pos_err > motor1_bspeed)  y_pos_err = motor1_bspeed;
           M1_backward(y_pos_err);  
       }     
    } else {
        x_pos_err = -x_pos_err;
        x_pos_err += X_OFF;
        M2_backward(x_pos_err);   
        if(y_pos_err > 0) {
          y_pos_err += Y_OFF;
          if(y_pos_err > motor1_fspeed)  y_pos_err = motor1_fspeed;
           M1_forward(y_pos_err);          
       } else {
          y_pos_err = -y_pos_err;
          y_pos_err += Y_OFF;
          if(y_pos_err > motor1_bspeed)  y_pos_err = motor1_bspeed;
          M1_backward(y_pos_err);          
       }    
    }

}


/////////////////////////// 각 Hole로 위치이동 ///////////////////////////////


void Pos_Move()
{
  // 스위치 입력에 따른 Hole 위치 값 변화
  switch(Operating_But)
  {
    case 1:
      Hole_state = Hole_1;
      break;
    case 2:  
      Hole_state = Hole_2;
      break;
    case 3:
      Hole_state = Hole_3;
      break;
    default:
     M1_stop();
     break;
  }  
  
 if(Hole_state_last == Hole_state) Operating_But = 0;

 if( (Operating_But == 1) || (Operating_But == 2) || (Operating_But == 3) ) 
 {
  if(count < Hole_state) 
  {
     err = Hole_state - count;
     M1_forward(motor1_du_fspeed);
      if( err < 10 )
      {
        M1_backward(motor1_du_bspeed);
        delay(stop_pulse);
        M1_stop(); 
        //Serial.println("hahahaha");    
        
        Operating_But = 0;       
        Hole_state_last = Hole_state;  
      } 
  }
  else if(count > Hole_state)  
  {
      err =  count - Hole_state;
      M1_backward(motor1_du_bspeed);
      if(err < 10)
      {
        M1_forward(motor1_du_fspeed);
        delay(stop_pulse);
        M1_stop(); 
        Operating_But = 0;        
        Hole_state_last = Hole_state; 
      } 
  }
}
}



void Button_Input()
{
  //////////////////////// 버튼 1 입력 단 /////////////////////////// 
  buttonState_1 = digitalRead(buttonPin_1);
  if(buttonState_1 != lastButtonState_1) {      //버튼이 이전과 다른 상태일 때
    if(buttonState_1 == HIGH){                  //버튼이 HIGH 신호일 때  
      delay(130);        
      if(motion_state_1 == 2)  motion_state_1 = 1;    
      else   motion_state_1++;       
      pos = 1;                
    }
    lastButtonState_1 = buttonState_1;          // 버튼상태 옮겨주기
  }

 //////////////////////// 버튼 2 입력 단 /////////////////////////// 
  buttonState_2 = digitalRead(buttonPin_2);
  if(buttonState_2 != lastButtonState_2) {      //버튼이 이전과 다른 상태일 때
    if(buttonState_2 == HIGH){                  //버튼이 HIGH 신호일 때  
      delay(130);  
      if(motion_state_2 == 2)   motion_state_2  = 1;          
      else   motion_state_2 ++;    
      pos = 2;
      }         
      lastButtonState_2 = buttonState_2;        // 버튼상태 옮겨주기
    }
 
 //////////////////////// 버튼 3 입력 단 /////////////////////////// 
  buttonState_3 = digitalRead(buttonPin_3);
  if(buttonState_3 != lastButtonState_3) {      //버튼이 이전과 다른 상태일 때
    if(buttonState_3 == HIGH){                  //버튼이 HIGH 신호일 때  
      delay(130);  
      if(motion_state_3 == 2)  motion_state_3 = 1;
      else   motion_state_3++;     
      pos = 3;
      }         
      lastButtonState_3 = buttonState_3;        // 버튼상태 옮겨주기
  }

}

inline void receive_data_parse(char inChar)
{
  switch(R_STATE)
  {
    case R_INIT :
      if(inChar == 'T' || inChar == 't')
        R_STATE = R_T1;
      else
        R_STATE = R_INIT;
      break;
    case R_T1 :
      if(inChar >= '3' && inChar <= '9') {
        receive_time_data = ((inChar & 0x0f)*10);
        R_STATE = R_T2;
      }
      else
        R_STATE = R_INIT;
      break;
    case R_T2 :
      if(inChar >= '0' && inChar <= '9') {
        receive_time_data = receive_time_data + (inChar & 0x0f);
        Serial.println(receive_time_data, DEC);
        R_STATE = R_INIT;
      }
      else
      {
        receive_time_data = 0;
        R_STATE = R_INIT;
      }
     break;
     default :
     R_STATE = R_INIT;
     break;
  }
if(inChar == 'C' || inChar == 'c')    receive_flag = false;       
if(inChar == 'O' || inChar == 'o')    receive_flag = true;   
}

void serialEvent()
{
  while(Serial.available())
  {
    char inChar = (char)Serial.read();
    receive_data_parse(inChar);
  }
}

void ISR_0() // A상 관련 인터럽트
{

  switch(ENCODER_STATE) {
    case State_0:
     if((digitalRead(PhaseA) == HIGH) && (digitalRead(PhaseB) == HIGH))  // UP
        ENCODER_STATE = State_1;
        count--; 
        return;
    case State_1:
     if((digitalRead(PhaseA) == LOW) && (digitalRead(PhaseB) == HIGH))  // DOWN
        ENCODER_STATE = State_0;
        count++; 
        return;
    case State_2:
     if((digitalRead(PhaseA) == LOW) && (digitalRead(PhaseB) == LOW))  // UP
        ENCODER_STATE = State_3;
        count--; 
        return;
    case State_3:
     if((digitalRead(PhaseA) == HIGH) && (digitalRead(PhaseB) == LOW))  // DOWN
        ENCODER_STATE = State_2;
        count++;
        return;
  }
}

void ISR_1() // B상 관련 인터럽트
{

  switch(ENCODER_STATE) {
    case State_0:
     if((digitalRead(PhaseA) == LOW) && (digitalRead(PhaseB) == LOW))  // DOWN
        ENCODER_STATE = State_3;
        count++;
        return;
    case State_1:
     if((digitalRead(PhaseA) == HIGH) && (digitalRead(PhaseB) == LOW))  // UP
        ENCODER_STATE = State_2;
        count--; 
        return;
    case State_2:
     if((digitalRead(PhaseA) == HIGH) && (digitalRead(PhaseB) == HIGH))  // DOWN
        ENCODER_STATE = State_1;
        count++;
        return;
    case State_3:
     if((digitalRead(PhaseA) == LOW) && (digitalRead(PhaseB) == HIGH))  // UP
        ENCODER_STATE = State_0;
        count--; 
        return;
  }
}
void ISR_2() // A상 관련 인터럽트
{
  switch(ENCODER_STATE_2) {
     case State_6:      // 역방향 - A_FALLING
      if((digitalRead(PhaseA_2) == LOW) && (digitalRead(PhaseB_2) == HIGH)) { // DOWN
        ENCODER_STATE_2 = State_4;
        count_2++; 
      }
     else if((digitalRead(PhaseA_2) == LOW) && (digitalRead(PhaseB_2) == LOW))  { // UP
        ENCODER_STATE_2 = State_7;
        count_2--; 
     }
        return;
    case State_7:     // 정방향 - A_FALLING
     if((digitalRead(PhaseA_2) == HIGH) && (digitalRead(PhaseB_2) == LOW))  // DOWN
        ENCODER_STATE_2 = State_6;
        count_2++; 
        return;
  }
}
void ISR_3() // B상 관련 인터럽트
{
  switch(ENCODER_STATE_2) {
   case State_4:
      if((digitalRead(PhaseA_2) == HIGH) && (digitalRead(PhaseB_2) == LOW)) { // UP
        ENCODER_STATE_2 = State_6;
       count_2--; 
      }
      if((digitalRead(PhaseA_2) == LOW) && (digitalRead(PhaseB_2) == LOW)) { // DOWN
        ENCODER_STATE_2 = State_7;
        count_2++; // count--;
      }
        return;        
    case State_7:
     if((digitalRead(PhaseA_2) == LOW) && (digitalRead(PhaseB_2) == HIGH))  // UP
        ENCODER_STATE_2 = State_4;
        count_2--; // count++;
        return;
  }
}

void limit_sw_y()
{
    LimitState_y = digitalRead(limit_S_y); 
      if(LimitState_y == LOW)  { M1_stop(); y_lim_flag = false;  delay(180); count = 0; } 
      else if(LimitState_y == HIGH) M1_backward(100);
    lastLimitState_y = LimitState_y;        
}

void limit_sw_x()
{
    LimitState_x = digitalRead(limit_S_x); 
    if(LimitState_x == LOW)  {M2_backward(100); delay(170); M2_stop(); x_lim_flag = false;  delay(180); count_2 = 0; }         
    else if(LimitState_x == HIGH) M2_forward(150);       
    lastLimitState_x = LimitState_x;  
}




ISR(TIMER1_COMPA_vect)        //1s마다 발생 Hole1
{
  Timer_CounterA++;
  
  Serial.println(Timer_CounterA);                                  
  
  if((Timer_CounterA <10) || (Timer_CounterA >Timer_Value-13))   Timer_Control_Flag1 =1;
  else    Timer_Control_Flag1 = 0;       
      
  if(Timer_CounterA == Timer_Value) {
    Timer_CounterA = 0;
    Button_flag = 0;      
    Timer_Control_Flag1 = 0;
    Operating_But = 1;      
    SUB_MOTION_STATE = H1_Motion_M3;  
    TIMSK1 &= ~(1<<OCIE1A);  
  }
}


ISR(TIMER1_COMPB_vect)      //1s마다 발생 Hole2
{
  Timer_CounterB++;
  Serial.println(Timer_CounterB);
  if((Timer_CounterB <10) || (Timer_CounterB >Timer_Value-13))   Timer_Control_Flag2 =1;
  else    Timer_Control_Flag2 = 0;
       
  if(Timer_CounterB == Timer_Value){
    Timer_CounterB = 0;
    Button_flag = 0;
    Timer_Control_Flag2 = 0;
    Operating_But = 2; 
    SUB_MOTION_STATE = H2_Motion_M3;   
    TIMSK1 &= ~(1<<OCIE1B);  
  }
}

ISR(TIMER1_COMPC_vect)      //1s마다 발생 Hole3
{  
  Timer_CounterC++;
    Serial.println(Timer_CounterC);
  if((Timer_CounterC <10) || (Timer_CounterC >Timer_Value-13))   Timer_Control_Flag3 =1;
  else    Timer_Control_Flag3 = 0;

        
  if(Timer_CounterC == Timer_Value){
    Timer_CounterC = 0;
    Button_flag = 0;
    Timer_Control_Flag3 = 0;
    Operating_But = 3;   
    SUB_MOTION_STATE = H3_Motion_M3;  
    TIMSK1 &= ~(1<<OCIE1C); 
  }
}

