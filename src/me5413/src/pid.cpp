#include "me5413/pid.hpp"


namespace controller {

PID::PID(double dt, double max, double min, double Kp, double Ki, double Kd,int max_queue_size) :
  dt_(dt),
  max_(max),
  min_(min),
  Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  pre_error_(0),
  integral_(0),
  max_queue_size(20)
{};


double PID::calculate(const double reference_value, const double feedback_value) {

    // Calculate error
    double error = reference_value - feedback_value;

    // Proportional term
    const double P_term = Kp_ * error;

    // Integral term
    error_queue_.push(error * dt_);
    integral_ += error * dt_;
    
    if (error_queue_.size() > max_queue_size)
    {
        integral_ -= error_queue_.front();
        error_queue_.pop();
    }
    
    const double I_term = Ki_ * integral_;

    // Derivative term
    const double derivative = (error - pre_error_) / dt_;
    const double D_term = Kd_ * derivative;

    // Calculate total output
    double output = P_term + I_term + D_term;

    // Restrict to max/min
    output = std::min(output, max_);
    output = std::max(output, min_);

    // Save error to previous error
    pre_error_ = error;

    return output;
};

}

