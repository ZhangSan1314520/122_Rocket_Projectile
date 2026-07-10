#pragma once

#define PID_DtHZ (1.0f/16000.0f)

class PID
{
public:

// 构造函数，带默认参数，方便使用
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float dt = PID_DtHZ, float i_max = 0.66*1,
        float max = 1.0f, float min = -1.0f)
        : _kp(kp), _ki(ki), _kd(kd), _dt(dt), _i_max(i_max),_max(max), _min(min),
          _error(0.0f), _prev_error(0.0f), P_OUT(0.0f), I_OUT(0.0f), D_OUT(0.0f),
          _output(0.0f), max_out_flag(false) {}

    float _kp; // 比例增益
    float _ki; // 积分增益
    float _kd; // 微分增益
    float _dt; // 采样时间
    float _i_max; //

    float _max; // 输出上限
    float _min; // 输出下限
    float _error; // 当前误差
    float _prev_error; // 上一次误差


    float update(float error)
    {
        _error = error;

        P_OUT = _kp * _error; 

        I_OUT += _ki * _error*_dt;
        if (I_OUT>=_i_max) I_OUT = _i_max;
        if (I_OUT<=-_i_max) I_OUT = -_i_max;

        D_OUT = _kd * (_error - _prev_error)/_dt;
        // Calculate output
        _output = P_OUT + I_OUT + D_OUT; //???

        // Clamp output
        if (_output > _max)
        {
            _output = _max;
            // max_out_flag = true; // 设置输出上限标志位 
        }else if (_output < _min)
        {
            _output = _min;
            // max_out_flag = true; // 设置输出上限标志位
        }
        _prev_error = _error;

        return _output;
    }

    void reset()
    {
        P_OUT = I_OUT = D_OUT = _output = _error =_prev_error = 0;
    }


private:
    float P_OUT;
    float I_OUT;
    float D_OUT;
    float _output; // PID输出
    bool max_out_flag; // 输出上限标志位

};