
#ifndef PID_H
#define PID_H

#include <ros/ros.h>

class PIDController
{
public:
    PIDController(double Kp, double Ki, double Kd, double alpha);

    ~PIDController();

    double 
    calculateControlSignal(double target, double current_value, double dt, double alpha);

private:
    double Kp_;
    double Ki_;
    double Kd_;
    double error_;
    double previous_error_;
    double integral_;
    double derivative_;
    double output_;
    double alpha;
    double smoothed_dFe;
};


#endif // PID_H