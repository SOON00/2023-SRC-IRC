#include "C:\Users\kgt22\Mbed Programs\SRC_chek\Header.h"
#include "C:\Users\kgt22\Mbed Programs\SRC_chek\MPU9250\mpu6050.h"

uint16_t tt;
extern int range_mode;
//------------------------------------- 선언 ------------------------------------//
//------------------------------mpu9250---------------------------------------//
// extern float sum;
// extern uint32_t sumCount;
// extern char buffer[14];
// extern float tmp_angle_x, tmp_angle_y, tmp_angle_z;
// extern float filltered_angle_x, filltered_angle_y, filltered_angle_z;

//extern MPU9250 mpu9250; // SDA, SCL
//extern Timer t;
//Serial pc(USBTX, USBRX, 115200); // tx, rx
extern MPU6050 mpu6050;
extern float pitchAngle;
extern float rollAngle;
//----------------------------------------------------------------------------//
//AnalogIn ir(PA_0);
extern DigitalOut led1;
extern InterruptIn btn;
//----------thread----------------//
extern uint64_t Now_time,Work_time,Nowm_time,Workm_time;
extern Thread psd_th;
extern Thread imu_th;
//extern I2C i2c;
//--------------------------------//
extern float x_deg;
extern float y_deg;
//----------------------------------------------------------------------------//

extern AnalogIn irfl;
extern AnalogIn irfr;
extern AnalogIn irbl;
extern AnalogIn irbr;
extern AnalogIn irfm;
extern AnalogIn irfmr;
extern AnalogIn irfml;
extern AnalogIn irmm;
extern AnalogIn irbmr;
extern AnalogIn irbml;

extern GP2A psdfl;
extern GP2A psdfr;
extern GP2A psdm;
extern GP2A psdb;
extern float now_data;

extern DigitalOut DirL;
extern DigitalOut DirR;
extern PwmOut PwmL;
extern PwmOut PwmR;
extern PwmOut rcServo;

extern float ang, inc,Inc,INC;
extern char preread;
extern int count;
extern float dis;
extern bool color;

extern float angML;
extern float angMR;
extern float angLL;
extern float angRR;

extern volatile bool gotPacket;
extern volatile float data_R[4];
extern uint16_t ir_val[9];
//0 fl
//1 fr
//2 bl
//3 br
//4 fm
//5 fmr
//6 fml
//7 mm
//8 bmr
//9 irbml

extern bool ir_plusval[9];
//0 fl+fr
//1 fmr + fml
//2 fl+fml
//3 fr+fmr
//4 bl+bml
//5 br+bmr
//6 all
extern double psdfl_val;
extern double psdfr_val;
extern double psdm_val;
extern double psdb_val;

extern bool Serial_chk;
extern bool code_start;

extern RawSerial board;
extern RawSerial pc;



//--------------------------------------------------//
extern Timer imu_tmr;
extern Timer be_tmr;//사용
extern Timer re_tmr;//사용
extern Timer com_check_tmr;
extern Timer blue_check_tmr;
extern Timer blu_tmr;
extern Timer little_tmr;
extern Timer imu_ang_chek_tmr;
extern Timer blue_escape_tmr;
extern Timer escape_blue_go_tmr;
extern Timer back_tmr;
extern Timer waiting_break_tmr;


extern double blue_escape_time;
extern double escape_time;//사용중
extern int com_chek_time;//사용중
extern double little_time;
extern double imu_ang_chek_time;
extern double blue_escape_time_ex;
extern int back_escape_time;
extern int waiting_break_time;
//--------------------------------------------------//
extern uint16_t black;
extern int mode;
extern int blue_escape_mode;
extern int imu_count;
extern float A,B,C,D;



extern int blutime;
extern Timer blu_tmr;
//Serial bt(PC_10, PC_11);
//Timer t;
extern int time1;//bluetooth


//---------------------------------------------//
extern volatile bool All_move;
//---------------------------------------------//



//------------------------------main--------------------------------------//
int main(){
    //osThreadSetPriority(osThreadGetId(), osPriorityRealtime7);
    

    DC_set();
    servo_set(rcServo);
    board.attach(&onSerialRx);
    color = false;

    imu_th.start(&imu_main);
    psd_th.start(&psd_read);
    //imu_ang_chek_tmr.start();
    //pc.printf("main\n");
    
    while(true){

        SerialRx_main();
        sensor_read();
        sensor_plus();
        if(All_move == true){
            //DC_ratio_inc();
            //sensor_print1();
            //pc.printf("data= %.3f, %.3f, %.3f\n\r",data_R[0],data_R[1],data_R[2]);
            com_check_tmr.start();

            if(mode ==0){//초반 구동 상대방 찾기(카메라 정중앙 들어오면 mode =1)
                pc.printf("mode == 0\n");
                first_move();
            }

            else if(mode == 1){//본 코드 시작
                //pc.printf("mode == 1\n");
                angle_check();//각도 체크
                servo_move(rcServo);//서보 움직임
                
//------새로 추가한 코드---------//
                if(data_R[2]==0){
                    if(ang < angML){
                        if(data_R[0] == 999){
                            no_see();
                        }
                        else if(data_R[1]<=40){
                            green_in_red_left();
                        }
                        else if(data_R[1]>40){
                            green_in_red_left();

                            if(abs(speedL) <=0.60 && abs(speedR) <= 0.60){
                                speedL = speedL * (1.65);
                                speedR = speedR * (1.65);
                            }
                        }
                    }
                    else if(angML <= ang && angMR > ang){
                        if(data_R[0] == 999){
                            no_see();
                        }
                        else if(data_R[1]<=40){
                            green_in_red_mid();
                        }
                        else if(data_R[1]>40){
                            green_in_red_mid();

                            if(abs(speedL) <=0.60 && abs(speedR) <= 0.60){
                                speedL = speedL * (1.65);
                                speedR = speedR * (1.65);
                            }
                        }
                    }
                    else if(angMR <= ang){
                        if(data_R[0] == 999){
                            no_see();
                        }
                        else if(data_R[1]<=40){
                            green_in_red_right();
                        }
                        else if(data_R[1]>40){
                            green_in_red_right();

                            if(abs(speedL) <=0.60 && abs(speedR) <= 0.60){
                                speedL = speedL * (1.65);
                                speedR = speedR * (1.65);
                            }
                        }
                    }
                }
                else if(data_R[2]==1){
                        angle_check(); // 로봇 각도 5도 이상 체크
                        if(data_R[0] == 999){ // 화면 원통 안보임
                            // pc.printf("상대 안보임 \n"); // 확인용 코드

                            no_see();
                        }
                        else if(data_R[1]<=45){ // 화면 원통 작음 or 보통 or 큼
                            green_out_red_move();

                        }
                        else if(data_R[1]>45){ // 화면 원통 매우 큼 or 매우 매우 큼 or 매우 매우 매우 큼
                                if(ang <= angLL && psdm_val <= 10){ // 앞 PSD 10cm 이하 + 각도 매우 왼쪽 : 매우 빠른 후진
                                    back_tmr_move<double>(&back_tmr, &back_escape_time, &psdm_val, "<", 20.0, &now_data, 70, -1.0, -0.6);

                                }
                                else if(ang >= angRR && psdm_val <= 10){ // 앞 PSD 10cm 이하 + 각도 매우 오른쪽 : 매우 빠른 후진
                                    back_tmr_move<double>(&back_tmr, &back_escape_time, &psdm_val, "<", 20.0, &now_data, 70, -0.6, -1.0);

                                }
                                else{ // 그 외
                                    green_out_red_move();
                                }
                            
                            if(imu_ang_chek_tmr.read_ms() > imu_ang_chek_time){
                                mode = 19;
                                     //pc.printf("robot angle\n");
                            }
                            if(abs(speedL) <= 0.60 && abs(speedR) <= 0.60){
                                speedL = speedL * (1.666);
                                speedR = speedR * (1.666);
                            }
                        }
                    }
                
//------------------------------//
                if((ang<70 || ang>110) && (psdfl_val < 40 || psdfr_val<40) && (ir_plusval[8]==true)){//벽을 볼때, 검정색영역 위
                    //pc.printf("wall - 1\n");
                    //적을 쫓아가서 직진하다가 적이 옆으로 빠지고 
                    //우리가 다시 트래킹하기 전에 벽에 너무 가까운 상태여서 박히는 경우
                    if(ang<70){//마지막으로 적을 좌측에서 봤을 경우
                        //pc.printf("side wall turn right\n\r");
                        speedL= -0.5;
                        speedR= 0.5;
                    }
                    else if(ang>110){//마지막으로 적을 우측, 중앙에서 봤을 경우
                        //pc.printf("side wall turn left\n\r");
                        speedL= 0.5;
                        speedR= -0.5;
                    }
                }
                else if (data_R[0] == 999 && (psdfl_val < 40 || psdfr_val < 40) &&(ir_plusval[8]==true)){//검정색 영역 위
                    //pc.printf("wall - 2\n");
                    //카메라,  서보 오류로 적을 잃어버리거나 찾지 못해서 벽에 박히는 경우
                    if(preread == 'R'){//마지막으로 적을 우측에서 봤을 경우
                        speedL= 0.5;
                        speedR= -0.5;
                    }
                    else{//마지막으로 적을 좌측, 중앙에서 봤을 경우 / preread값이 없는 경우
                        speedL= -0.5;
                        speedR= 0.5;
                    }
                }
                else if(ir_plusval[8]==true){//ir 전부 검정색을 볼때
                    //pc.printf("All black\n");
                
                    if(data_R[0] == 999){//적이 안 보이면 제자리 회전
                        no_see();
                        //pc.printf("no_see\n");
                    }
                    else{//적이 보이면 트래킹 (DC_follow 함수 다듬기)
                        DC_follow();
                        //pc.printf("DC_follow\n");
                    }
                }
                else if(ir_plusval[7]==true){//ir 전부 색영역일때
                    //pc.printf("All color\n");
                    if(now_data>60){//빨간 색 탈출 코드 - 전방 ir이 검정을 볼 때까지 후진 후 트래킹
                        mode=-1;//while 대신 후진을 위한 mode 변경
                    }
                    else{//파란 색 탈출 코드 - 빈 공간 찾아서 후방 ir이 검정을 볼 때까지 직진 후 트래킹
                        mode=-5;//안정적인 탈출코드 구현을 위해 mode 변경
                    }
                }
                else{//ir이 하나라도 들어갔을 때
                    //pc.printf("Any color\n");
                    if(ir_plusval[0]==true && now_data >=100 && data_R[1]<=40){//여기 now_data가 문제
                        //전방 ir이 빨간 색에 들어가고 상대가 멀리 있을 때 (공격시가 아닌 트래킹하는 상황)
                        if(ang <= angML){//적이 좌측에 보이면 우측 ir이 색영역에 들어갈 때까지 좌회전 후 트래킹
                            mode=-2;//while 대신 좌회전을 위한 mode 변경
                        }
                        else{//적이 우측에 보이면 좌측 ir이 색영역에 들어갈 때까지 우회전 후 트래킹
                            mode=-3;//while 대신 우회전을 위한 mode 변경
                        }
                    }
                    else if((ir_plusval[0]==true && now_data>=100)||
                            //전방 ir이 파란 색에 들어갔을 때 (공격시)
                            (ir_plusval[0]==true && now_data<100 && data_R[1]>40))
                            //전방 ir이 빨간 색에 들어가고 상대가 가까이 있을 때 (공격시)
                    {
                        if(ang>angML && ang<angMR && (data_R[0] != 999)){//공격시 색영역에 상대가 전방에 있으면 대기
                            //pc.printf("stay\n");
                            if(data_R[1]>45){
                                speedL=0;
                                speedR=0;
                            }
                            else if(data_R[1]<=45){
                                All_whl();
                            }
                            
                        }
                        else{
                            if(ang<90 && data_R[1]>45){//적이 좌측으로 탈출했을 때 좌회전해서 트래킹
                                escape(&be_tmr,&escape_time,-0.7,0.8);//while 대신 좌회전을 위한 mode 변경
                            }
                            else if(ang>=90 && data_R[1]>45){//적이 우측으로 탈출했을 때 우회전해서 트래킹
                                escape(&be_tmr,&escape_time,0.8,-0.7);//while 대신 우회전을 위한 mode 변경
                            }
                        }
                    }
                    

                    else{//그 외의 경우 트래킹 (다른 세분화할 경우가 있으면 위에 else if로 추가)
                        DC_follow();
                        //pc.printf("DC_follow\n");
                    }
                }
               
            }//mode = 1 (else if(mode ==1))
            else if(mode == -5){//파란원 탈출 코드 mode
                //pc.printf("mode == -5\n");
                angle_check();
                servo_move(rcServo);
                if(blue_escape_mode==0){
                    blue_check_tmr.start();
                    blue_escape_mode =1;
                }
                else if(blue_escape_mode ==1){
                    blue_check_tmr.reset();
                }
                
                if(blue_check_tmr.read_ms()<blue_escape_time){
                    Rescape_move(&little_tmr,&little_time,-0.3,-0.3);
                    if(data_R[1]>40 && ir_plusval[7]==true){
                        speedL = 0.99;speedR = 0.99;
                    }
                    else if(data_R[1]>40 && ir_plusval[7]== false){
                        speedL = -0.3; speedR = -0.3;
                    }
                }
                else if(blue_check_tmr.read_ms()>blue_escape_time){
                    if(ir_val[0]>black && ir_val[1]>black){
                        speedL = 0.99; speedR = 0.99;
                        mode = 1;
                    }
                    else {
                        Rescape_move(&little_tmr,&little_time,-0.5,-0.5);
                        if(psdfl_val < 130 || psdfr_val<130){//상대가 전방에 있을 때 제자리 회전
                            speedL = 0.5; speedR = -0.5;
                        }
                        else{
                            if(ir_val[2]>black && ir_val[3]>black){//후방 ir이 검정이 되면 정지
                                speedL=0;
                                speedR=0;
                                mode=1;//본 코드로 복귀
                                All_whl();
                             }
                            else{
                                    speedL=0.8;
                                    speedR=0.8;
                                }
                        }
                    }
                }
            }

            else if(mode == -1){//전방 ir이 검정을 볼 때까지 후진하는 mode (타이머로 변경하고싶으면 변경해도 됨)
                //pc.printf("mode == -1\n");
                //servo_move(rcServo);
                angle_check();
                escape(&be_tmr,&escape_time,-0.6,-0.6);
                speedL = 0;speedR = 0;
                mode = 1;
                All_whl();
            }
            else if(mode == -2){//우측 ir이 색을 볼 때까지 좌회전하는 mode (타이머로 변경하고싶으면 변경해도 됨)
                //pc.printf("mode == -2\n");
                angle_check();
                servo_move(rcServo);
                if(data_R[1]<=50){
                        if(ir_plusval[0]==true){
                            speedL = -0.5;
                            speedR = 0.5;
                        }
                        else if(ir_val[0]>black && ir_val[7]<black){
                            speedL = 0.0; speedR = 0.0;
                        }
                        else if((ir_val[1]<black && ir_val[0]>black) && ir_val[7]<black && (preread =='R'&& preread =='M')){
                            speedL = 0.8; speedR = 0.35;
                        }
                        else if((ir_val[1]<black && ir_val[0]>black && ir_val[3]<black) && ir_val[7]<black && preread =='L'){
                            mode =1;
                            DC_follow();
                        }
                        else if(ir_plusval[8]==true){
                            mode = 1;
                            All_whl();
                        }
                        else if(ir_val[1]<black && ir_val[0]>black && ir_val[3]<black){
                            speedL = 0.3;
                            speedR = 0.3;
                            //pc.printf("mode o done");
                        }
                    }
                else if(data_R[1]>50){
                    speedL = 0.99; speedR = 0.99;
                    mode =1;
                    All_whl();
                    }
                else if(rollAngle <-5){
                    All_whl();
                }
                else if(ir_plusval[7]==true){
                    mode =1;
                    All_whl();
                }
            }
            
            else if(mode == -3){//좌측 ir이 색을 볼 때까지 우회전하는 mode (타이머로 변경하고싶으면 변경해도 됨)
                //pc.printf("mode == -3\n");
                angle_check();
                servo_move(rcServo);
                if(data_R[1]<=50){
                        if(ir_plusval[0]==true){
                            speedL = 0.5;
                            speedR = -0.5;
                        }
                        else if((ir_plusval[6] == true && ir_val[2]<black) && (preread =='L' ||preread =='M')){
                            speedL = 0.35; speedR = 0.8;
                        }
                        else if((ir_plusval[6] == true && ir_val[3] > black && ir_val[2]<black) && preread =='R'){
                            mode =1;
                            DC_follow();
                        }
                        else if(ir_plusval[8]==true){
                            mode = 1;
                            All_whl();
                        }
                        else if(ir_plusval[6] == true && ir_val[3] > black && ir_val[2]<black){
                            speedL = 0.3;
                            speedR = 0.3;
                            //pc.printf("mode o done");
                        }
                    }
                else if(data_R[1]>50){
                    speedL = 0.99; speedR = 0.99;
                    mode =1;
                    All_whl();
                    }
                else if(rollAngle <-5){
                    All_whl();
                }
                else if(ir_plusval[7]==true){
                    mode =1;
                    All_whl();
                }
            }
            
            else if(mode ==19){
                angle_check();
                servo_move(rcServo);//서보 움직임
                //pc.printf("imu running\n");
                //들렸을 때 구동
                /*전방 ir사용이 불가능, 나머지 ir들의 신뢰성도 낮아짐
                전방 psd 사용 불가, 후방 psd 신뢰성 저하

                실험을 통해 중앙 ir과 후방 ir 사용이 가능한 지 알아보고 코드 작성

                들린 상태로 우리가 상대를 미는 경우도 있으므로 그 상태로 상대와 같이 색영역에 들어가면 실점

                들린 상태에서 전방 ir사용 불가, 중앙 ir로 색영역에 들어가는지 판단해서 정지하도록 코드 작성
                //*/
                //pc.printf("imu_ok\n");

                if(ir_val[7]>black && ir_val[8]>black && data_R[1]>50){
                    speedL = 0.99; speedR = 0.99;
                }
                else if(ir_val[7]<black && ir_val[8]<black ){
                    if(rollAngle < -1){
                        speedL = -0.5; speedR = -0.5;
                    }
                    else{
                        All_whl();
                    }
                }
                else if(ir_val[8]<black && ir_val[7]>black){
                    speedL = -0.9; speedR = 0.9;
                }
                else if(ir_val[7]<black && ir_val[8]>black){
                    speedL = 0.9; speedR = -0.9;
                }
                else if(abs(pitchAngle) > 5){
                    speedL = 0.8; speedR = 0.8;
                    if(abs(pitchAngle)<5){
                        if(preread == 'L'){
                            speedL = -0.8; speedR = 0.8;
                        }
                        else if(preread =='R'){
                            speedL = 0.8; speedR = -0.89;
                        }
                        All_whl();
                    }
                }
            }
            All_move = false;
            tmr_reset(&com_check_tmr);
            //tmr_reset(&imu_ang_chek_tmr);
        }//All_move = true

        else if(All_move == false){
            //com_check_tmr.start();
            if(com_check_tmr.read_us() > com_chek_time){
                speedL = 0.45; speedR = -0.45;             
            }
        }
        DC_move(speedL,speedR);
    }//main_while
}//int main()