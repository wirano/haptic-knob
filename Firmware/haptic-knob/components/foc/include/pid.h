#ifndef PID_H
#define PID_H


#include <stdint.h>

typedef struct
{
    float Err;                 //误差
    float ErrLim;              //经过限幅的误差
    float Integral;            //积分结果
    float IntegralLim;         //经过限幅的积分
    float ErrOld;              //上一次的误差
    float Differential;        //微分结果
    float DifferentialLim;     //经过限幅的微分
    float Result;              //运算结果
    float OutDelta;            //输出的增量
    float OutOld;              //上一次的输出
    float Out;                 //运算结果经过限幅后的输出
}PIDDataSt;

typedef struct
{
    float Kp;                  //比例系数
    float Ki;                  //积分系数
    float Kd;                  //微分系数
    float ErrLim;              //比例误差限幅
    float ErrOutLim;           //比例输出限幅
    float InteErrLim;          //积分误差限幅
    float InteLim;             //积分限幅
    float InteOutLim;          //积分输出限幅
    float DiffLim;             //微分限幅
    float DiffOutLim;          //微分输出限幅
    float OutLim;              //输出限幅
    float OutDeltaLim;         //输出增量限幅
}PIDParamSt;

typedef struct
{
    PIDDataSt Data;            //PID数据结构体
    PIDParamSt Param;          //PID参数结构体
}PIDSt;


float MyPID(PIDSt *pid, uint16_t hz, float expect, float feedback);

void MyPIDSetParameter(PIDSt *pid,
                       float kp, float ki, float kd,
                       float err_lim, float err_out_lim,
                       float inte_err_lim, float inte_lim, float inte_out_lim,
                       float diff_lim, float diff_out_lim,
                       float out_lim ,float out_delta_lim);

#endif //PID_H
