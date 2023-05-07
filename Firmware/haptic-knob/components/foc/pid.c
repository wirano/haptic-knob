#include "pid.h"
#include <math.h>
#include "foc_utils.h"


#define LIMIT(x, min, max) (((x) <= (min)) ? (min) : (((x) > (max)) ? (max) : (x)))


/**
 * @brief PID控制器
 * @param pid PID结构体指针
 * @param hz 任务频率
 * @param expect 期望值
 * @param feedback 反馈值
 * @return PID输出值
 */
float MyPID(PIDSt *pid, uint16_t hz, float expect, float feedback)
{
    if (hz == 0)
    {
        pid->Data.Err = 0;
        pid->Data.ErrLim = 0;
        pid->Data.Integral=0;
        pid->Data.IntegralLim = 0;
        pid->Data.ErrOld = 0;
        pid->Data.Differential = 0;
        pid->Data.DifferentialLim = 0;
        pid->Data.Result = 0;
        pid->Data.Out = 0;
        pid->Data.OutOld=0;
    }
    else
    {
        pid->Data.Err = expect - feedback;
        pid->Data.ErrLim = LIMIT(pid->Data.Err, -1 * pid->Param.ErrLim, pid->Param.ErrLim);
        pid->Data.Integral += ( (LIMIT(pid->Data.Err, -1 * pid->Param.InteErrLim, pid->Param.InteErrLim)) / (float) hz );
        pid->Data.IntegralLim = LIMIT(pid->Data.Integral, -1 * pid->Param.InteLim, pid->Param.InteLim);
        pid->Data.Integral = pid->Data.IntegralLim;
        pid->Data.Differential = (pid->Data.Err - pid->Data.ErrOld) * (float) hz;
        pid->Data.DifferentialLim = LIMIT(pid->Data.Differential, -1 * pid->Param.DiffLim, pid->Param.DiffLim);
        pid->Data.ErrOld = pid->Data.Err;

        pid->Data.Result = LIMIT(pid->Param.Kp * pid->Data.ErrLim, -1 * pid->Param.ErrOutLim, pid->Param.ErrOutLim ) +
                           LIMIT(pid->Param.Ki * pid->Data.IntegralLim, -1 * pid->Param.InteOutLim, pid->Param.InteOutLim ) +
                           LIMIT(pid->Param.Kd * pid->Data.DifferentialLim, -1 * pid->Param.DiffOutLim, pid->Param.DiffOutLim );
        pid->Data.Out = LIMIT(pid->Data.Result, -1 * pid->Param.OutLim, pid->Param.OutLim);
        pid->Data.OutDelta = LIMIT(pid->Data.Out-pid->Data.OutOld,-1*pid->Param.OutDeltaLim,pid->Param.OutDeltaLim) ;
        pid->Data.Out = pid->Data.OutOld+pid->Data.OutDelta;
        pid->Data.OutOld=pid->Data.Out;
    }

    return pid->Data.Out;
}

/**
 * @brief 为PID控制器设置参数，因为参数比较多，所以将控制器参数分开单独设置，而不采用传参的方式传入PID控制器，避免繁杂的传参
 * @param pid PID结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param err_lim 比例误差限幅
 * @param err_out_lim 比例输出限幅
 * @param inte_err_lim 积分误差限幅
 * @param inte_lim 积分限幅
 * @param inte_out_lim 积分输出限幅
 * @param diff_lim 微分限幅
 * @param diff_out_lim 微分输出限幅
 * @param out_lim 输出限幅
 * @param out_delta_lim 输出增量限幅
 * @out 无
 */
void MyPIDSetParameter(PIDSt *pid,
                              float kp, float ki, float kd,
                              float err_lim, float err_out_lim,
                              float inte_err_lim, float inte_lim, float inte_out_lim,
                              float diff_lim, float diff_out_lim,
                              float out_lim ,float out_delta_lim)
{
    pid->Param.Kp=kp;
    pid->Param.Ki=ki;
    pid->Param.Kd=kd;
    pid->Param.ErrLim=err_lim;
    pid->Param.ErrOutLim=err_out_lim;
    pid->Param.InteErrLim=inte_err_lim;
    pid->Param.InteLim=inte_lim;
    pid->Param.InteOutLim=inte_out_lim;
    pid->Param.DiffLim=diff_lim;
    pid->Param.DiffOutLim=diff_out_lim;
    pid->Param.OutLim=out_lim;
    pid->Param.OutDeltaLim=out_delta_lim;

    pid->Data.Integral=0;
    pid->Data.ErrOld=0;
    pid->Data.OutOld=0;
}