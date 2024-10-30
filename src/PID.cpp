#include <robot_control/PID.h>

// Constructor for initializing gains and setting initial values to zero
PIDController::PIDController(double Kp, double Ki, double Kd, double alpha) 
        : Kp_(Kp), Ki_(Ki), Kd_(Kd),
         smoothed_dFe(0.0), error_(0.0), previous_error_(0.0),
         integral_(0.0), derivative_(0.0), output_(0.0)

{} 

PIDController::~PIDController(){}

double PIDController::calculateControlSignal(double target, double current_value, double dt, double alpha) {
    error_ = target - current_value;
    integral_ += error_*dt;
    derivative_ = (error_ - previous_error_)/dt;
    smoothed_dFe = alpha*smoothed_dFe + (1-alpha)*derivative_;
    previous_error_ = error_;
    output_ = Kp_ * error_ + Ki_ * integral_ + Kd_ * smoothed_dFe;
    return output_;
}
