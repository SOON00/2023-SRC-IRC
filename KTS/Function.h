# ifndef Function_H
# define Function_H

#include "mbed.h"
#include "GP2A.h"
#include "Servo.h"
#include <string>
#include <stdlib.h>
#include "rtos.h"
#include "Thread.h"

#define psdcontroltime 50
#define maincontroltime 10
#define imu_time 100
#define Alpha 0.8
//#include "Servo.h"

template <class T> T map(T x,T in_min,T in_max,T out_min,T out_max);
void sensor_read();
void sensor_print();
void sensor_plus();

void psd_read();//Thread사용

void servo_set(PwmOut &rc);
void servo_move(PwmOut &rc);
void servoturn(PwmOut&rc,float deg);
void onSerialRx();
void SerialRx_main();

void DC_set();
void DC_move(float _PWML, float _PWMR);
void DC_follow();
void Button_start();
void blu_print(int a);
void IR_print(uint16_t a,uint16_t b,uint16_t c,uint16_t d,uint16_t e,uint16_t f,uint16_t g,uint16_t h,uint16_t i);
void PSD_print(uint16_t a,uint16_t b,uint16_t c,uint16_t d);
void All_whl();
void DC_ratio_inc();
void first_move();

void tmr_move(Timer* _timer,int* _tmr , double _speedL,double _speedR);
void tmr_reset(Timer* _timer);

extern double speedL;
extern double speedR;
extern float ratio;

void Rescape_move(Timer* __timer, double*__tmr, double __speedL,double __speedR);
void escape(Timer* __timer, double*__tmr, double __speedL,double __speedR);
void whl_move();

void imu_main();
void imu_read();
void no_see();
void angle_check();

void wait_move();
void green_in_red_right();
void green_in_red_left();
void green_in_red_mid();
void blue_all_tmr_move();
void green_out_red_move();
void fight_back_tmr_move(Timer* _tmr, int* _time, int* _check_time, double _speedL, double _speedR);
void turn_90_tmr_move(Timer* _tmr, double* _time, uint16_t* _while_brk_sensor, uint16_t _sensor_val, volatile float* _com_data, double _speedL, double _speedR);
void turn_180_tmr_move(Timer* _tmr, double* _time, uint16_t* _while_brk_sensor, uint16_t _sensor_val, volatile float* _com_data, double _speedL, double _speedR);

template<class T> void back_tmr_move(Timer* _tmr, int* _time, T* _while_brk_sensor, const char* _inequality, T _sensor_val, float* psd_val, double psd_dis, double _speedL, double _speedR){
    _tmr->start(); // _tmr->start(); = *_tmr.start();
    if(_inequality[0] == '='){
        while(*_while_brk_sensor == _sensor_val){
            // if(*psd_val > *psd_dis){
                speedL = _speedL; speedR = _speedR;
            // }
            // else if(*psd_val <= *psd_dis){
            //     speedL = _speedR; speedR = _speedL;
            // }

            All_whl();
            if(_tmr->read_ms() > *_time){
                break;
            }
        }

        if(_speedL <= -0.95 || _speedR <= -0.95){
            ratio = 0;
        }
        _tmr->reset();
        _tmr->stop();
    }
    else if(_inequality[0] == '>'){
        while(*_while_brk_sensor > _sensor_val){
            // if(*psd_val > *psd_dis){
                speedL = _speedL; speedR = _speedR;
            // }
            // else if(*psd_val <= *psd_dis){
            //     speedL = _speedR; speedR = _speedL;
            // }

            All_whl();
            if(_tmr->read_ms() > *_time){
                break;
            }
        }

        if(_speedL <= -0.95 || _speedR <= -0.95){
            ratio = 0;
        }
        _tmr->reset();
        _tmr->stop();
    }
    else if(_inequality[0] == '<'){
        while(*_while_brk_sensor < _sensor_val){
            // if(*psd_val > *psd_dis){
                speedL = _speedL; speedR = _speedR;
            // }
            // else if(*psd_val <= *psd_dis){
            //     speedL = _speedR; speedR = _speedL;
            // }

            All_whl();
            if(_tmr->read_ms() > *_time){
                break;
            }
        }

        if(_speedL <= -0.95 || _speedR <= -0.95){
            ratio = 0;
        }
        _tmr->reset();
        _tmr->stop();
    }
}
#endif