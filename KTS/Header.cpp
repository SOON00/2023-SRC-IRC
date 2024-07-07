#include "C:\Users\kgt22\Mbed Programs\SRC_chek\header.h"
#include "C:\Users\kgt22\Mbed Programs\SRC_chek\MPU9250\mpu6050.h"
#include <cstdint>
#include <time.h>
//---------------------------define-------------------------//


// uint16_t raw_array[MASK_LENGTH] = {0,};
// uint16_t raw_array_index = 0;
//----------------------------선언--------------------------//
InterruptIn btn(BUTTON1);
DigitalOut led1(LED1);
static char serialInBuffer[32];
static int data_cnt=0,buff_cnt=0;
char byteIn;


//-------thread용------//
Thread psd_th(osPriorityAboveNormal);
Thread imu_th(osPriorityNormal);

uint64_t Now_time,Work_time,Nowm_time,Workm_time,Nowi_time,Worki_time;
uint16_t chek_time;
//---------------------//
float x_deg = 4;
float y_deg = 4;
//---------------------//
AnalogIn irfl(PA_0);
AnalogIn irfr(PA_1);
AnalogIn irbl(PC_1);
AnalogIn irbr(PB_0);
AnalogIn irfm(PB_1);
AnalogIn irfmr(PA_4);
AnalogIn irfml(PC_0);
AnalogIn irbmr(PC_5); 
AnalogIn irbml(PA_5);
//AnalogIn irbml(PA_5);

GP2A psdfl(PC_3,30,150,60,0);
GP2A psdfr(PC_4,30,150,60,0);
//GP2A psdf(PA_1,7,80,22.5,0.1606);//실험할 때 쓴 psd값
GP2A psdb(PA_7,20,150,60,0);
GP2A psdfm(PA_6,30,150,60,0);


DigitalOut DirL(PC_7);
DigitalOut DirR(PB_6);
PwmOut PwmL(PB_4);
PwmOut PwmR(PB_5);
PwmOut rcServo(PA_8);

int blutime=0;
bool Serial_chk = false;
bool code_start = false;
volatile bool gotPacket = false;
volatile bool gotPacket1 = false;
volatile float data_R[4];

float filter_is_first = 1u;
float prev_data;
float now_data;

uint16_t black = 20000;//ir 검정색 바닥을 봤을 때
double speedL = 0;
double speedR = 0;
int mode =0;
float ang=90.0, inc=1.0, Inc=3.0, INC=5.0;
char preread;
//-------------------servo 각도--------------------//
float angMR = 105;
float angML = 75;
float angLL = 30;
float angRR = 150;
float angMM = 90;
//-------------------------------------------------//
float A,B,C,D;
float dis = 48; //data[1] 가까움 척도(세부조정 필요)
bool color;
int escape_mode=0;
int Rescape_mode = 0;
int blue_escape_mode =0;

uint16_t ir_val[9];
//0 fl
//1 fr
//2 bl
//3 br
//4 fm
//5 fmr
//6 fml
//7 bmr
//8 bml

//0 fl
//1 fr
//2 bl
//3 br"//5 fmr
//6 fml
bool ir_plusval[9];
//0 fl+fr
//1 fmr + fml
//2 fl+fml
//3 fr+fmr
//4 bl+bml
//5 br+bmr
//6 원돌기 걸치는 ir
//7 all blue or red
//8 all black
double psdfl_val;
double psdfr_val;
double psdm_val;
double psdb_val;

//-----------------Time---------------------//

Timer blu_tmr; //bluetooth timer
Timer imu_tmr;
Timer be_tmr;
Timer com_check_tmr;
Timer blue_check_tmr;
Timer little_tmr;
Timer imu_ang_chek_tmr;
Timer back_tmr;
Timer blue_escape_tmr;
Timer escape_blue_go_tmr;
Timer waiting_break_tmr;


int time1;//bluetooth

int tilt_back_escape_time = 1500000; // 세부조정 필요!!!
double blue_escape_time = 2500;
double escape_time = 1000;
int com_chek_time = 1500000;
int back_escape_time = 900;
double little_time = 1000;
double imu_ang_chek_time = 1500;
double blue_escape_time_ex = 3000;
double escape_blue_go_time = 2000;
int fight_back_escape_time = 630; 
int fight_back_break_check_time = 320; 
int waiting_break_time = 5000;

//---------------------------------------------------//
extern Thread imu_th;

RawSerial board(PA_9, PA_10, 115200);
RawSerial pc(USBTX,USBRX,115200);
RawSerial bt(PA_11,PA_12,115200);//bluetooth

//------------------------------mpu9250---------------------------------------//
// float sum = 0;
// uint32_t sumCount = 0;
// char buffer[14];
// float tmp_angle_x, tmp_angle_y, tmp_angle_z;
// float filltered_angle_x, filltered_angle_y, filltered_angle_z;
// float alpha = 0.90;

//MPU9250 mpu9250(D14,D15); // SDA, SCL
//Timer t;

int imu_count = 0;
//Serial pc(USBTX, USBRX, 115200); // tx, rx
//----------------------------------------------------------------------------//
volatile bool All_move = false;
volatile bool blue_all = false;
float ratio = 1;
//-----------------------------------------------------//
MPU6050 mpu6050;    

float pitchAngle = 0;
float rollAngle = 0;
//bool imu_ang_chek=false;
//-------------------------------통신---------------------------------------//
void onSerialRx(){
    Serial_chk = true;
    if(board.readable()){
        //pc.printf("data= %.3f, %.3f, %.3f, %.3f\n\r",data_R[0],data_R[1],data_R[2],data_R[3]);
        byteIn=board.getc();
        if(byteIn==','){
            serialInBuffer[buff_cnt]='\0';
            data_R[data_cnt++]=atof(serialInBuffer);
            buff_cnt=0;
        }
        else if(byteIn=='\n'){
            serialInBuffer[buff_cnt]='\0';
            data_R[data_cnt]=atof(serialInBuffer);
            buff_cnt=0;
            data_cnt=0;
            gotPacket=true;
            gotPacket1 = true;
        }
        else{
            serialInBuffer[buff_cnt++]=byteIn;
        }
    }
}

template <class T> T map(T x,T in_min,T in_max,T out_min,T out_max){
    return(x -in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
//blurtooth 통신 part
void SerialRx_main(){
    if(gotPacket){
        gotPacket = false;
        gotPacket1 = true;
        All_move = true;
    }
}

//------------------------------------------------------------------------//

//----------------------- timer 사용 함수 -------------------------//

void tmr_reset(Timer* _tmr){ // 타이머 리셋
    _tmr->reset();
    _tmr->stop();
    pc.printf("timer ok\n");
}

void Rescape_move(Timer* _timer, double*_tmr, double _speedL,double _speedR){
        if(Rescape_mode == 0){
            _timer -> start();
            while(_timer -> read_ms() < *_tmr){
                servo_move(rcServo);
                speedL = _speedL;
                speedR = _speedR;
                DC_move(speedL,speedR);
                //pc.printf("%f",speedL);
                //whl_move();
                
            }
            Rescape_mode =1;
        }
        else if(Rescape_mode == 1){
            _timer -> reset();
            while(_timer -> read_ms() < *_tmr){
                servo_move(rcServo);
                speedL = _speedL;
                speedR = _speedR;
                DC_move(speedL, speedR);
                //pc.printf("%f",speedL);
                //whl_move();
                
            }
        }
}


//연습용
void escape(Timer* _timer ,double* _tmr, double _speedL,double _speedR){
        if(escape_mode == 0){
            _timer -> start();
            while(_timer -> read_ms() < *_tmr){
                servo_move(rcServo);
                speedL = _speedL;
                speedR = _speedR;
                //pc.printf("%f",speedL);
                All_whl();
                //whl_move();
                
            }
            escape_mode =1;
        }
        else if(escape_mode == 1){
            _timer -> reset();
            while(_timer -> read_ms() < *_tmr){
                servo_move(rcServo);
                speedL = _speedL;
                speedR = _speedR;
                //DC_move(speedL, speedR);
                All_whl();
                //pc.printf("%f",speedL);
                //whl_move();
                
            }
        }
}

//-------------------------------------------------------//

//------------------ 센서,부가부품(servo) 관련 함수 ----------------------//
float kp= 10.0 , ki = 1.0 , kd= 0.1 ;
float error_;
float error_previous;

float P_control, I_control, D_control ;
float control_time = 0.004;
float PID_control;



void servoturn(PwmOut&rc,float deg){
    uint16_t pulseW=map<float>(deg,0.,180.,600.,2400.);
    rc.pulsewidth_us(pulseW);
}

void servo_set(PwmOut &rc){
    rcServo.period_ms(10);
    servoturn(rcServo,90);
}

void servo_move(PwmOut &rc){
    int count;
    if(gotPacket1){
        count = count+1;
        pc.printf("data= %.3f, %.3f, %.3f, %.3f \n\r",data_R[0],data_R[1],data_R[2],data_R[3]);
        //board.printf("data= %.3f, %.3f, %.3f\n\r",data[0],data[1],data[2]);
        

        if (count % 10 == 0){
            //board.printf("data= %.3f, %.3f, %.3f\n\r",data[0],data[1],data[2]);
            count = 0;
        }

        if (data_R[0] >= 0 && data_R[0] <= 400) error_ = abs(data_R[0] - 200);

        P_control = kp*error_;
        I_control = I_control+ki*error_*control_time;
        D_control = kd*(error_-error_previous)/control_time;

        PID_control = P_control + I_control + D_control;
        PID_control = map<float>(PID_control, 0. , 3500. , 0. ,16. );
        if (data_R[0] < 160 && ang>10) {
            ang-=PID_control;
            preread ='L';
        }
        else if (data_R[0] > 240 && data_R[0]<400 && ang<170) {
            ang+=PID_control;
            preread ='R';
        }
        else if(data_R[0]>=160 && data_R[0]<=240){
            preread = 'M';
        }
        if (data_R[0] == 999 && ang>10 && preread == 'L') {
            ang-=10;
            preread ='L';
        }
        else if (data_R[0] == 999 && ang<170 && preread == 'R') {
            ang+=10;
            preread ='R';
        }
        servoturn(rcServo,ang);
        
        error_previous = error_;
        gotPacket1 = false;
        //pc.printf("PID = %f, error_ = %f, ang = %f\n",PID_control,error_,ang);
    }
}

void psd_read(){
    prev_data = psdb.getDistance();
        while(true){
        Now_time = rtos::Kernel::get_ms_count();
       // pc.printf("%11u\n",chek_time - Now_time);
        psdb_val = psdb.getDistance();

        now_data = (prev_data * Alpha) + ((1 - Alpha) * psdb_val);
        prev_data = now_data;

        chek_time = Now_time;
        Work_time = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + (psdcontroltime-(Work_time-Now_time)));
    }
}

void sensor_read(){
        ir_val[0] = irfl.read_u16();
        ir_val[1] = irfr.read_u16();
        ir_val[2] = irbl.read_u16();    
        ir_val[3] = irbr.read_u16();
        ir_val[4] = irfm.read_u16();
        ir_val[5] = irfmr.read_u16();
        ir_val[6] = irfml.read_u16();
        //ir_val[7] = irmm.read_u16();
        ir_val[7] = irbmr.read_u16();
        ir_val[8] = irbml.read_u16();
        //ir_val[9] = irbml.read_u16();

        psdfl_val = psdfl.getDistance();
        psdfr_val = psdfr.getDistance();
        psdm_val = psdfm.getDistance();
        //psdb_val = psdb.getDistance();
}

void sensor_plus(){
    if(ir_val[0]<black && ir_val[1]<black) ir_plusval[0] = true;//앞쪽 색영역
    else ir_plusval[0] = false;
    if(ir_val[5]<black && ir_val[6]<black) ir_plusval[1] = true;//중간앞 ir색영역
    else ir_plusval[1] = false;
    if(ir_val[0]<black && ir_val[6]<black) ir_plusval[2] = true;//fl,fml 색영역
    else ir_plusval[2] = false; 
    if(ir_val[1]<black && ir_val[5]<black) ir_plusval[3] = true;//fr,fmr색영역
    else ir_plusval[3] = false;
    if(ir_val[2]<black && ir_val[8]<black) ir_plusval[4] = true;//bl ,bml 색영역
    else ir_plusval[4] = false;
    if(ir_val[3]<black && ir_val[7]<black) ir_plusval[5] = true;//br, bmr 색영역
    else ir_plusval[5] = false;
    if(ir_val[1]>black && ir_val[0]<black) ir_plusval[6] = true;//fr -> black, fl -> 색영역
    else ir_plusval[6] = false;
    if(ir_val[0]<black && ir_val[1]<black && ir_val[2]< black && ir_val[3]< black && ir_val[4]<black && ir_val[5]<black && ir_val[6]<black
         && ir_val[7]<black && ir_val[8]<black){//ir_val[4]값 빠져있음
            ir_plusval[7]=true;
        }
    else ir_plusval[7] = false;
    if(ir_val[0]>black && ir_val[1]>black && ir_val[2]> black && ir_val[3]> black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black
         && ir_val[7]>black && ir_val[8]>black){
             ir_plusval[8] = true;
         }
    else ir_plusval[8] = false;
}

void sensor_print(){
    //pc.printf("ir_val1 : | %u | %u | %u | %u | %u | %u | %u | %u | %u |\n", ir_val[0], ir_val[1], ir_val[2], ir_val[3], ir_val[4], ir_val[5], ir_val[6], ir_val[7], ir_val[8]); // 확인용 코드
    //pc.printf("ir_plusval : | %d | %d | %d | %d | %d | %d |\n", ir_plusval[0], ir_plusval[1], ir_plusval[2], ir_plusval[3], ir_plusval[4], ir_plusval[5]); // 확인용 코드
   //pc.printf("psdfl_val : | %lf |, psdfr_val : | %lf |, psdm_val : | %lf |, psdb_val : | %lf |\n", psdfl_val, psdfr_val,psdm_val,now_data); // 확인용 코드
   pc.printf("now_data : | %lf |, psdb_val : | %lf |\n", now_data,psdb_val);
}

void blu_print(int a){
        bt.printf("type : %d\n", a);
}

void IR_print(uint16_t a,uint16_t b,uint16_t c,uint16_t d,uint16_t e,uint16_t f,uint16_t g,uint16_t h,uint16_t i){
        bt.printf("IR: %d, %d, %d, %d, %d, %d, %d, %d, %d\n", a, b, c, d, e, f, g, h, i);
}

void PSD_print(uint16_t a,uint16_t b,uint16_t c,uint16_t d){
        bt.printf("IR: %d, %d, %d, %d\n", a, b, c, d);
}

void angle_check(){//ang값이 x도 이상일때 모드를 19로 바꿔주는 코드 추가
    if(rollAngle<-5 ){// ||abs(rollAngle)
        imu_ang_chek_tmr.start();
    }
    else{
        //pc.printf("angle_chek_else\n");
        tmr_reset(&imu_ang_chek_tmr);
        mode = 1;
    }
}



//-------------------------------------------------------//

//------------------ 구동 관련 함수 ----------------------//
void DC_set(){
    PwmL.period_us(66);
    PwmR.period_us(66);
}

void DC_move(float _PwmL, float _PwmR){
    _PwmL = _PwmL * ratio;
    _PwmR = _PwmR * ratio;

    if(_PwmL<0) DirL = 0;
    else DirL = 1;

    if(_PwmR<0) DirR = 0;
    else DirR = 1;

    PwmL = abs(_PwmL);
    PwmR = abs(_PwmR);
}

void DC_ratio_inc(){
    // 모터 비율 조절
    if(ratio < 1.0){
        ratio += 0.1;
        if(ratio >= 1.0){
            ratio = 1.0;
        }
    }
}

//whlie사용하거나 반복문 멈춰야할 때 필수 ★★★
void All_whl(){
    SerialRx_main();
    sensor_read();
    sensor_plus();

    if(All_move == true){
        DC_ratio_inc();
        servo_move(rcServo);
        DC_move(speedL, speedR);
       // pc.printf("All_whl 1\n");
    }

    All_move = false;
}

void first_move(){
    if(data_R[0]==999){
        speedL = 0.60; speedR = -0.60;
        //pc.printf("first_move 1\n");
    }
    else if(data_R[0] < 160){
        speedL = -0.60; speedR = 0.60;
        //pc.printf("first_move 2\n");
    }
    else if(data_R[0]>=160 && data_R[0]<=240){
        speedL = 0.0; speedR = 0.0;
        //pc.printf("first_move 3\n");
        mode = 1;
    }
    else if(data_R[0]>240){
        speedL = 0.60; speedR = -0.60;
        //pc.printf("first_move 4\n");
    }
}

void no_see(){
    if(preread == 'R'){
        speedL = 0.5; speedR =-0.5; 
    }
    else{
        speedL = 0.5; speedR =-0.5;
    }
}

void DC_follow(){
    if(data_R[0]==999){//트래킹 중 상대가 보이지 않을 때
        if(preread == 'R'){
            speedL = 0.6; speedR =-0.6; 
        }
        else{
            speedL = -0.6; speedR = 0.6;
        }
    }
    else{
        if(data_R[1]<dis || ((data_R[1]>dis && ((ang > angLL)&&(ang<50)) || ((ang>130)&&(ang<angRR)) ))){
            float delang = map<float>(abs(ang-90), 0. , 80. ,0.0 ,0.4 );
            if(ang>130){
                A = 1;
                B = -1;
                D=0;
                //pc.printf("slow right\n\r");
            }
            else if(ang>=50 && ang<=130){
                A = 0;
                B = 0;
                D = 0.59;
                //pc.printf("middle\n\r");
            }
            else if(ang<50){
                A=-1;
                B=1;
                D=0;
                //pc.printf("slow left\n\r");
            }
            speedL = 0.4+ A*delang+D;
            speedR = 0.4+ B*delang+D;
        }
        else if(data_R[1]>=dis && (ang <= angLL|| ang>=angRR)){
            if(ang<=angLL){
                speedL = -0.7;
                speedR = 0.7;
                //pc.printf("fast right\n\r");
            }
            else if(ang>=angRR){
                speedL = 0.7;
                speedR = -0.7;
                //pc.printf("fast left\n\r");
            }
        }
        else if(data_R[1]>=dis && ang>=50 && ang<=130){
                speedL = 0.99;
                speedR = 0.99;
                //pc.printf("fast straight\n\r");
        }
    }
}

void green_in_red_left(){
    if(ir_plusval[7]==true){ // 모든 바퀴
        if(now_data >= 100){ // 뒤 PSD 70cm 이상 : 빠른 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            if(angLL < ang){ // 서보 보통 왼쪽
                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70.0, -map<float>(ang, angML, angLL, 0.60, 0.85), -0.50);
            }
            else if(ang <= angLL){ // 서보 매우 왼쪽
                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70.0, -map<float>(ang, angLL, 0.0, 0.85, 0.95), -0.50);
            }
        }
        else if(now_data < 100){ // 뒤 PSD 70cm 이하 : 색 영역 탈출 모드
            blue_all_tmr_move();
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    else if(ir_plusval[0]==true && ir_plusval[1]==true && ir_val[7] < black && ir_val[8]<black && ir_val[2] >black && ir_val[3]>black){ // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.45, -0.30);
    }
    else if(ir_plusval[0]==true && ir_val[5]<black && ir_val[6]<black && ir_val[7]>black && ir_val[8]>black){ // ir 왼쪽 앞 + ir 오른쪽 앞 : 제자리 좌회전 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

        turn_90_tmr_move(&back_tmr, &escape_time, &ir_val[4], black, &data_R[1], -0.60, 0.60);
    }
    else if(ir_plusval[2]==true && ir_plusval[1] ==false&& ir_plusval[3] ==false&& ir_plusval[4] ==false&& ir_plusval[5] ==false){ // 왼쪽 앞 바퀴 ???
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 매우 오른쪽 전진
            speedL = 0.60; speedR = 0.10;

        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
   
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.45, -0.30);
        }
    }
    else if(ir_plusval[2]==true && ir_plusval[4]==true && ir_plusval[3]==false && ir_plusval[5]==false){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 조금 왼쪽 전진
            // speedL = 0.30; speedR = 0.60;
            if(ang <= angLL && data_R[0] < 90){
                turn_180_tmr_move(&back_tmr, &escape_time, &ir_val[1], black, &data_R[1], -0.80, 0.80);
                // speedL = -0.80; speedR = 0.80;
            }
            else{
                speedL = 0.45; speedR = 0.99;
            }

        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.45, -0.30);
        }
    }
    else if(ir_plusval[2]==true && ir_plusval[4]==true && ir_plusval[3]==false && ir_plusval[5] == false){ // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 조금 오른쪽 전진
            // speedL = 0.60; speedR = 0.30;
            speedL = 1.0; speedR = 0.45; // ???
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.45, -0.30);
        }
    }
    else if(ir_plusval[2]==true && ir_plusval[3]==true && ir_plusval[4]==true && ir_plusval[5]==false){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴 : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.45, -0.30);
    }
    else if(ir_plusval[2]==true && ir_plusval[3]==true && ir_plusval[4]==false && ir_plusval[5]==true){ // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.45, -0.30);
    }
    else if(ir_plusval[2]==true && ir_plusval[3]==false && ir_plusval[4]==true && ir_plusval[5]==true){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 뒷 바퀴 : 매우 왼쪽 전진 ???
        speedL = 0.10; speedR = 0.60; // 0.225;
    }
    else if(
        (ir_plusval[8]==true) || // 모두 검은색 : 자유롭게 공격
        (ir_plusval[4]== true  && ir_val[0]>black && ir_val[1]>black && ir_val[3]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black && ir_val[7]>black) || // 왼쪽 뒷 바퀴 : 자유롭게 공격
        (ir_plusval[5]==true && ir_val[0]>black && ir_val[1]>black && ir_val[2]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black && ir_val[8]>black) || // 오른쪽 뒷 바퀴 : 자유롭게 공격
        (ir_plusval[4]==true && ir_plusval[5]==true && ir_val[0]>black && ir_val[1]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black) // 왼쪽 뒷 바퀴 + 오른쪽 뒷 바퀴 : 자유롭게 공격
    ){
        if(ang <= angLL){
            speedL = -map<float>(ang, angLL, 0.0, 0.15, 0.50);
            speedR = 0.50;
        }
        else if(ang > angLL){
            speedL = map<float>(ang, angML, angLL, 0.30, 0.18);
            speedR = 0.60;
        }
    }
    else{ // 그 외 : 왼쪽 전진
        speedL = 0.45; speedR = 1.0;
    }
}


void green_in_red_mid(){
    if(ir_plusval[7]==true){ // 모든 바퀴
        if(now_data >= 90){ // 뒤 PSD 70cm 이상 : 빠른 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.70, -0.70);
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        else if(now_data < 90){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
            blue_all_tmr_move();
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false){ // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.35, -0.35);
    }
    else if(ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞
        if(ang >= 90){ // 서보 조금이라도 오른쪽 : 제자리 우회전 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            turn_90_tmr_move(&back_tmr, &escape_time, &ir_val[4], black, &data_R[1], 0.60, -0.60);
        }
        else if(ang < 90){ // 서보 조금이라도 왼쪽 : 제자리 좌회전 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            turn_90_tmr_move(&back_tmr, &escape_time, &ir_val[4], black, &data_R[1], -0.60, 0.60);
        }
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false){ // 왼쪽 앞 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 매우 오른쪽 전진
            speedL = 0.60; speedR = 0.10;
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.35, -0.35);
        }
    }
    else if(ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false){ // 오른쪽 앞 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 매우 왼쪽 전진
            speedL = 0.10; speedR = 0.60;
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.35, -0.35);
        }
    }
    else if(
        (ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴
        (ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
    ){
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 전진
            speedL = 0.35; speedR = 0.35;
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.35, -0.35);
        }
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.35, -0.35);
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false){ // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.35, -0.35);
    }
    else if(ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == true){ // 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 왼쪽 전진
        speedL = 0.27; speedR = 0.60;
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == true && ir_plusval[4] == true){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 뒷 바퀴 : 오른쪽 전진
        speedL = 0.60; speedR = 0.27;
    }
    else if(
        (ir_plusval[8]==true) || // 모두 검은색 : 자유롭게 공격
        (ir_plusval[4]== true  && ir_val[0]>black && ir_val[1]>black && ir_val[3]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black && ir_val[7]>black) || // 왼쪽 뒷 바퀴 : 자유롭게 공격
        (ir_plusval[5]==true && ir_val[0]>black && ir_val[1]>black && ir_val[2]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black && ir_val[8]>black) || // 오른쪽 뒷 바퀴 : 자유롭게 공격
        (ir_plusval[4]==true && ir_plusval[5]==true && ir_val[0]>black && ir_val[1]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black) // 왼쪽 뒷 바퀴 + 오른쪽 뒷 바퀴 : 자유롭게 공격
    ){
        speedL = 0.60; speedR = 0.60;
    }
    else{ // 그 외 : 전진
        speedL = 0.60; speedR = 0.60;
    }
}

void green_in_red_right(){
    if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == true){ // 모든 바퀴
        if(now_data >= 95){ // 뒤 PSD 70cm 이상 : 빠른 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            if(ang < angRR){ // 서보 보통 오른쪽
                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angMR, angRR, 0.60, 0.85));
            }
            else if(angRR <= ang){ // 서보 매우 오른쪽
                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angRR, 180.0, 0.85, 0.95));
            }
        }

        else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
            blue_all_tmr_move();
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false){ // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 왼쪽 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.30, -0.45);
    }
    else if(ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 : 제자리 우회전 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

        turn_90_tmr_move(&back_tmr, &escape_time, &ir_val[4], black, &data_R[1], 0.60, -0.60);
    }
    else if(ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false){ // 오른쪽 앞 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 매우 왼쪽 전진
            speedL = 0.10; speedR = 0.60;
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.30, -0.45);
        }
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == true){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 조금 왼쪽 전진 -> 왼쪽 전진
            // speedL = 0.30; speedR = 0.60;
            speedL = 0.45; speedR = 1.0; 
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.30, -0.45);
        }
    }
    else if(ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false){ // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
        if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 조금 오른쪽 전진 -> 오른쪽 전진
            if(ang >= angRR && data_R[0] > 290){

                turn_180_tmr_move(&back_tmr, &escape_time, &ir_val[0], black, &data_R[1], 0.80, -0.80);
                // speedL = 0.80; speedR = -0.80;
            }
            else{
                // speedL = 0.60; speedR = 0.30;
                speedL = 1.0; speedR = 0.45;
            }
        }
        else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70 ,-0.30, -0.45);
        }
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true){ // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴 : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.30, -0.45);
    }
    else if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false){ // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.30, -0.45);
    }
    else if(ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == true){ // 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 매우 오른쪽 전진
        speedL = 0.60; speedR = 0.10; // 0.225;

    }
    else if(
        (ir_plusval[8]==true) || // 모두 검은색 : 자유롭게 공격
        (ir_plusval[4]== true  && ir_val[0]>black && ir_val[1]>black && ir_val[3]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black && ir_val[7]>black) || // 왼쪽 뒷 바퀴 : 자유롭게 공격
        (ir_plusval[5]==true && ir_val[0]>black && ir_val[1]>black && ir_val[2]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black && ir_val[8]>black) || // 오른쪽 뒷 바퀴 : 자유롭게 공격
        (ir_plusval[4]==true && ir_plusval[5]==true && ir_val[0]>black && ir_val[1]>black && ir_val[4]>black && ir_val[5]>black && ir_val[6]>black) // 왼쪽 뒷 바퀴 + 오른쪽 뒷 바퀴 : 자유롭게 공격
    ){
        if(angRR <= ang){
            speedL = 0.60;
            speedR = -map<float>(ang, 180.0, angRR, 0.60, 0.18);

        }
        else if(ang < angRR){
            speedL = 0.60;
            speedR = map<float>(ang, angRR, angMR, 0.18, 0.30);

        }
    }
    else{ // 그 외 : 오른쪽 전진
        speedL = 1.0; speedR = 0.45;

    }
}


void blue_all_tmr_move(){
    blue_all = true;

    blue_escape_tmr.start();
    speedL = -0.60; speedR = 0.60; // psdf 2개 미만 거리 가까울 때 기본 동작 : 좌회전
    if(blue_escape_tmr.read_ms() < blue_escape_time_ex){
        if( // psdf 2개 이상 거리 가까움 : 매우 빠른 탈출
            (psdfl_val > 90 && psdm_val >= 30) || // psdfl + psdfm 거리 멀음 : 매우 빠른 전진
            (psdm_val >= 30 && psdfr_val > 90) // psdfm + psdfr 거리 멀음 : 매우 빠른 전진
        ){
            escape_blue_go_tmr.start();
            speedL = 0.0; speedR = 0.0;
            while(ir_val[0] < black && ir_val[1] < black){ // ir 왼쪽 앞 검정색 + ir 오른쪽 앞 검정색 : break
                if(ang < angMM){
                    speedL = map<float>(ang, 0.0, angMM, 0.80, 1.0);
                    speedR = 1.0;
                }
                else if(ang >= angMM){
                    speedL = 1.0;
                    speedR = map<float>(ang, 180.0, angMM, 0.80, 1.0);
                }

                All_whl();
                if(escape_blue_go_tmr.read_ms() > escape_blue_go_time){ // 일정 시간 이상 : break
                    break;
                }
            }
            tmr_reset(&escape_blue_go_tmr);
        }
        else{ // psdf 2개 미만 거리 가까움 : 제자리 오른쪽 회전
            if(data_R[0] != 999){ // 화면 원통 보임
                if(ang <= angLL){ // 상대 매우 왼쪽 : 제자리 오른쪽 회전
                    speedL = -0.60; speedR = 0.60;
                }
                else if(angLL < ang && ang < angRR);// 상대 중간쯤 : 동작 그대로
                else if(ang >= angRR){
                    speedL = 0.60; speedR = -0.60; // 상대 매우 오른쪽 : 제자리 왼쪽 회전
                }
            }
            else{ // 화면 원통 안보임 : 이전값 기반 상대 추적
                no_see();
            }
        }
    }
    else{
        if(ang <= angLL){ // 서보 매우 왼쪽
            speedL = -map<float>(ang, angLL, 0.0, 0.15, 0.50);
            speedR = 0.50;
        }
        else if(angLL < ang && ang <= angML){ // 서보 보통 왼쪽
            speedL = map<float>(ang, angML, angLL, 0.30, 0.18);
            speedR = 0.60;
        }
        else if(angML < ang && ang < angMR){ // 서보 가운데
            speedL = 0.60; speedR = 0.60;
        }
        else if(angMR < ang && ang < angRR){ // 서보 보통 오른쪽
            speedL = 0.60;
            speedR = map<float>(ang, angRR, angMR, 0.18, 0.30);
        }
        else if(angRR < ang){ // 서보 매우 오른쪽
            speedL = 0.50;
            speedR = -map<float>(ang, 180.0, angRR, 0.50, 0.18);
        }
    }
    tmr_reset(&blue_escape_tmr);
}

void turn_90_tmr_move(Timer* _tmr, double* _time, uint16_t* _while_brk_sensor, uint16_t _sensor_val, volatile float* _com_data, double _speedL, double _speedR){
    while(*_while_brk_sensor < _sensor_val){
        speedL = _speedL; speedR = _speedR;

        All_whl();
        if(_tmr->read_ms() > *_time){
            break;
        }
        if(*_com_data <=30){
            break;
        }
        if(
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == true) || // 모든 바퀴
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴
            (ir_plusval[0] == true && ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + ir 오른쪽 앞 O
            (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) || // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 + ir 왼쪽 앞 O
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
        ){
            break;
        }
    }
    _tmr->reset();
    _tmr->stop();
}
void turn_180_tmr_move(Timer* _tmr, double* _time, uint16_t* _while_brk_sensor, uint16_t _sensor_val, volatile float* _com_data, double _speedL, double _speedR){
    while(*_while_brk_sensor > _sensor_val){
        speedL = _speedL; speedR = _speedR;

        All_whl();
        if(_tmr->read_ms() > *_time){
            break;
        }
        if(*_com_data <= 30){
            break;
        }
        if(
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == true) || // 모든 바퀴
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴
            (ir_plusval[0] == true && ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + ir 오른쪽 앞 O
            (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) || // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 + ir 왼쪽 앞 O
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴 + 오른쪽 앞 바퀴
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
        ){
            break;
        }
    }
    _tmr->reset();
    _tmr->stop();
}
void green_out_red_move(){
    // pc.printf("상대 보임 \n"); // 확인용 코드

    if(ang <= angML){ // 서보 왼쪽
        if(angLL < ang){ // 서보 보통 왼쪽
            if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == true){ // 모든 바퀴
                if(now_data >= 95){ // 뒤 PSD 70cm 이상 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)

                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angML, angLL, 0.60, 0.85), -0.50);

                    if(data_R[1]>=40){
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }

                ////////////////////////////////////////////////////////////////////////////////////////////////////
                else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
                    blue_all_tmr_move();
                }
                ////////////////////////////////////////////////////////////////////////////////////////////////////
            }
            else if(
                (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false) || // ir 왼쪽 앞 + ir 오른쪽 앞 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴) 
                (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 뒷 바퀴 + 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            ){
                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angML, angLL, 0.60, 0.85), -0.50);

                if(data_R[1]>=40){
                    fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                }
            }
            else if(ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false){ // 왼쪽 앞 바퀴
                if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격
                    speedL = map<float>(ang, angML, angLL, 0.30, 0.18);
                    speedR = 0.60;
                }
                else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angML, angLL, 0.60, 0.85), -0.50);

                    if(data_R[1]>=40){
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }
            }
            else if(
                (ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴
                (ir_val[0] < black && ir_val[1] > black && ir_val[2] < black && ir_val[3] > black && ir_val[4] > black && ir_val[5] > black && ir_val[6] > black)
            ){
                if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격
                        speedL = map<float>(ang, angML, angLL, 0.30, 0.18);
                        speedR = 0.60;
                }
                else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angML, angLL, 0.60, 0.85), -0.50);

                    if(data_R[1]>=40){
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }
            }
            else{ // 그 외 : 자유롭게 공격
                speedL = map<float>(ang, angML, angLL, 0.30, 0.18);
                speedR = 0.60;
            }
        }
        else if(ang <= angLL){ // 서보 매우 왼쪽
            if(data_R[1]>=40){ // 화면 원통 매우 매우 or 매우 매우 매우 큼
                speedL = -1.0; speedR = 1.0;
            }
            else{ // 화면 원통 매우 매우 or 매우 매우 매우 크지 않음
                if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == true){ // 모든 바퀴
                    if(now_data >= 95){ // 뒤 PSD 70cm 이상 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angLL, 0.0, 0.85, 0.95), -0.50);

                        if(data_R[1]>=40){

                            fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                        }
                    }

                    ////////////////////////////////////////////////////////////////////////////////////////////////////
                    else if(now_data < 90){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
                        blue_all_tmr_move();
                    }
                    ////////////////////////////////////////////////////////////////////////////////////////////////////
                } 
                else if(
                    (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false) || // ir 왼쪽 앞 + ir 오른쪽 앞 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴) 
                    (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                    (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 뒷 바퀴 + 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                    (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 우회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                ){

                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angLL, 0.0, 0.85, 0.95), -0.50);

                    if(data_R[1] >=40){
   
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }
                else if(
                    (ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false) || // 왼쪽 앞 바퀴
                    (ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == true) // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴
                ){
                    if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격
                        speedL = -map<float>(ang, angLL, 0.0, 0.15, 0.50);
                        speedR = 0.50;
                      
                    }
                    else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
                        
                        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -map<float>(ang, angLL, 0.0, 0.85, 0.95), -0.50);

                        if(data_R[1] >=40){
                         
                            fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                        }
                    }
                }
                else{ // 그 외 : 자유롭게 공격
                    speedL = -map<float>(ang, angLL, 0.0, 0.15, 0.50);
                    speedR = 0.50;

                }
            }
        }
    }
    else if(angML < ang && ang < angMR){ // 서보 중간
        if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == true){ // 모든 바퀴
            if(now_data >= 95){ // 뒤 PSD 70cm 이상 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)

                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -0.50);

                if(data_R[1] >=40){
                    fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                }
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            // else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 공격 모드
            //     speedL = 0.60; speedR = 0.60;
            //     where = 93;
            // }
            ////////////////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////////////////
            else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
            
                blue_all_tmr_move();
            }
            ////////////////////////////////////////////////////////////////////////////////////////////////////
        }
        else if(
            (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[5] == false && ir_plusval[4] == false) || // ir 왼쪽 앞 + ir 오른쪽 앞 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴) 
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == false && ir_plusval[4] == true) || // 왼쪽 뒷 바퀴 + 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[5] == true && ir_plusval[4] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
        ){
    
            back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -0.50);

            if(data_R[1] >=40){

                fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
            }
        }
        else if(
            (ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[4] == false && ir_plusval[5] == false) || // 왼쪽 앞 바퀴
            (ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false) // 오른쪽 앞 바퀴
        ){
            if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격
                speedL = 0.60; speedR = 0.60;

            }
            else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -0.50);

                if(data_R[1] >=40){
                    fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                }
            }
        }
        else if(
            (ir_plusval[2] == true && ir_plusval[3] == false && ir_plusval[4] == false && ir_plusval[5] == true) || // 왼쪽 앞 바퀴 + 왼쪽 뒷 바퀴
            (ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == false) // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
        ){
            if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격

                speedL = 0.60; speedR = 0.60;
            }
            else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)

                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -0.50);

                if(data_R[1] >=40){
    
                    fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                }
            }
        }
        else{ // 그 외 : 자유롭게 공격
            speedL = 0.60; speedR = 0.60;

        }
    }
    else if(angMR <= ang){ // 서보 오른쪽
        if(ang < angRR){ // 서보 보통 오른쪽
            if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == true){ // 모든 바퀴
                if(now_data >= 95){ // 뒤 PSD 70cm 이상 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
 
                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angMR, angRR, 0.60, 0.85));

                    if(data_R[1] >=40){
       
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }

                ////////////////////////////////////////////////////////////////////////////////////////////////////
                // else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 공격 모드
                //     speedL = 0.60;
                //     speedR = map<float>(ang, angRR, angMR, 0.18, 0.30);
                //     where = 103;
                // }
                ////////////////////////////////////////////////////////////////////////////////////////////////////

                ////////////////////////////////////////////////////////////////////////////////////////////////////
                else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
  
                    blue_all_tmr_move();
                }
                ////////////////////////////////////////////////////////////////////////////////////////////////////
            }
            else if(
                (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[4] == false && ir_plusval[5] == false) || // ir 왼쪽 앞 + ir 오른쪽 앞 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == true) || // 왼쪽 뒷 바퀴 + 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
            ){

                back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angMR, angRR, 0.60, 0.85));

                if(data_R[1] >=40){
   
                    fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                }
            }
            else if(ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false){ // 오른쪽 앞 바퀴
                if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격
                    speedL = 0.60;
                    speedR = map<float>(ang, angRR, angMR, 0.18, 0.30);

                }
                else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
    
                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angMR, angRR, 0.60, 0.85));

                    if(data_R[1] >=40){
      
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }
            }
            else if(
                (ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == false) || // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
                (ir_val[0] > black && ir_val[1] < black && ir_val[2] > black && ir_val[3] < black && ir_val[4] > black && ir_val[5] > black && ir_val[6] > black) || // ir 오른쪽 앞 + ir 오른쪽 뒤
                (ir_val[0] > black && ir_val[1] > black && ir_val[2] < black && ir_val[3] > black && ir_val[4] > black && ir_val[5] > black && ir_val[6] > black) // ir 오른쪽 가운데
            ){
                if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격 ★★★여기 검토 필요★★★
                        speedL = 0.60;
                        speedR = map<float>(ang, angRR, angMR, 0.18, 0.30);

                }
                else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 우회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
 
                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angMR, angRR, 0.60, 0.85));

                    if(data_R[1] >=40){
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }
            }
            else{ // 그 외 : 자유롭게 공격
                speedL = 0.60;
                speedR = map<float>(ang, angRR, angMR, 0.18, 0.30);

            }
        }
        else if(angRR <= ang){ // 서보 매우 오른쪽
            if(data_R[1] >=40){ // 화면 원통 매우 매우 or 매우 매우 매우 큼
                speedL = 1.0; speedR = -1.0;

            }
            else{ // 화면 원통 매우 매우 or 매우 매우 매우 크지 않음
                if(ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == true){ // 모든 바퀴
                    if(now_data >= 95){ // 뒤 PSD 70cm 이상 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)

                        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angRR, 180.0, 0.85, 0.95));

                        if(data_R[1] >=40){

                            fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                        }
                    }


                    ////////////////////////////////////////////////////////////////////////////////////////////////////
                    else if(now_data < 95){ // 뒤 PSD 70cm 이하 : 자유롭게 공격 ★★★ 색 영역 탈출 모드
                        blue_all_tmr_move();
                    }
                    ////////////////////////////////////////////////////////////////////////////////////////////////////
                }
                else if(
                    (ir_plusval[0] == true && ir_plusval[2] == false && ir_plusval[3] == false && ir_plusval[4] == false && ir_plusval[5] == false) || // ir 왼쪽 앞 + ir 오른쪽 앞 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                    (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false) || // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                    (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == true) || // 왼쪽 뒷 바퀴 + 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                    (ir_plusval[2] == true && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == false) // 왼쪽 앞 바퀴 + 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴 : 좌회 후진 (ir 왼쪽 앞 바퀴, 오른쪽 앞 바퀴 검은색 될때까지, 시간 지나면 자동으로 빠져나옴)
                ){

                    back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angRR, 180.0, 0.85, 0.95));

                    if(data_R[1] >=40){
                        fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                    }
                }
                else if(
                    (ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[4] == false && ir_plusval[5] == false) || // 오른쪽 앞 바퀴
                    (ir_plusval[2] == false && ir_plusval[3] == true && ir_plusval[4] == true && ir_plusval[5] == false) // 오른쪽 앞 바퀴 + 오른쪽 뒷 바퀴
                ){
                    if(ir_plusval[0] == false){ // ir 왼쪽 앞 + ir 오른쪽 앞 X : 자유롭게 공격
                        speedL = 0.50;
                        speedR = -map<float>(ang, 180.0, angRR, 0.50, 0.15);
 
                    }
                    else if(ir_plusval[0] == true){ // ir 왼쪽 앞 + ir 오른쪽 앞 O : 좌회 후진 (ir 가운데 앞 바퀴가 검은색일 때까지, 시간 지나면 자동으로 빠져나옴)
      
                        back_tmr_move<bool>(&back_tmr, &back_escape_time, &ir_plusval[0], "==", true, &now_data, 70, -0.50, -map<float>(ang, angRR, 180.0, 0.85, 0.95));

                        if(data_R[1] >=40){
               
                            fight_back_tmr_move(&back_tmr, &fight_back_escape_time, &fight_back_break_check_time, -1.0, -1.0);
                        }
                    }
                }
                else{ // 그 외 : 자유롭게 공격
                    speedL = 0.50;
                    speedR = -map<float>(ang, 180.0, angRR, 0.50, 0.15);
              
                }
            }
        }
    }
}


void fight_back_tmr_move(Timer* _tmr, int* _time, int* _check_time, double _speedL, double _speedR){
    _tmr->start(); // _tmr->start(); = *_tmr.start(); // 타이머 시작
    while(_tmr->read_ms() < *_time){ // 타이머 일정 시간 이하 : 특정 움직임 유지
        speedL = _speedL; speedR = _speedR;

        All_whl();

        if(_tmr->read_ms() > *_check_time){ // 타이머 일정 시간 이상 : break 판별
            if(data_R[1]>=45){ // 상대 매우 가까움 : break
                break;
            }
        }
        if(ir_val[2] < black || ir_val[3] < black){ // 뒷 ir 색 영역 : break
            break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    waiting_break_tmr.start(); 
    while(waiting_break_tmr.read_ms() < waiting_break_time){
        wait_move();

        All_whl();

        if(now_data < 95){ // 상대 빨간원 근처 + 상대 빨간원 바깥 : break
            if(data_R[2] == 1){
                break;
            }
        }
        else if(now_data >= 95){ // 상대 파란원 근처 + 상대 파란원 바깥 : break
            if(ir_plusval[0]==true){
                break;
            }
        }
        if(data_R[1] <= 15|| data_R[1] >=40){ // 상대 매우 멀음 or 상대 매우 가까움 : break
            break;
        }
    }
    tmr_reset(&waiting_break_tmr);
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    ratio = 0;
    _tmr->reset(); // 타이머 리셋
    _tmr->stop();
}

void wait_move(){
    if(data_R[0] == 999){ // 상대 안보임
        speedL = 0.40; speedR = -0.40;
    }
    else if(data_R[0] < 160){ // 화면 왼쪽 보임
        speedL = -0.40; speedR = 0.40;
    }
    else if(160 <= data_R[0] && data_R[0] < 230){ // 화면 가운데 보임
        speedL = 0.0; speedR = 0.0;
        // pc.printf("mode = 1"); // 확인용 코드
    }
    else if(230 <= data_R[0]){ // 화면 오른쪽 보임
        speedL = 0.40; speedR = -0.40;
    }
}
//-------------------------------------------------------//

//################################
//-----------------------------------------imu-------------------------------------------------//
void imu_main(){
    mpu6050.whoAmI();                              // Communication test: WHO_AM_I register reading 
    mpu6050.calibrate(accelBias,gyroBias);         // Calibrate MPU6050 and load biases into bias registers
    mpu6050.init();      
    mpu6050.complementaryFilter(&pitchAngle, &rollAngle);  // Initialize the sensor
   
    while(true){
        Nowi_time = rtos::Kernel::get_ms_count();

        mpu6050.complementaryFilter(&pitchAngle, &rollAngle); 
        //pc.printf("%.1f,%.1f\r\n",rollAngle,pitchAngle);// send data to matlab
        //wait_ms(40);

        Worki_time = rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count() + (imu_time-(Worki_time-Nowi_time)));
        if(pitchAngle >= 13){
            //blt.printf("imu_thread_reset"); // 확인용 코드

            NVIC_SystemReset();

            // mpu9250.initMPU9250();

            // mpu9250.getAres(); // Get accelerometer sensitivity +-2g 4g 8g
            // mpu9250.getGres(); // Get gyro sensitivity      250  500   1000

            // pitch_p = 0;
            // prev_y = 0;
        }
    }
}
//-------------------------------------------------------------------------------------------------//