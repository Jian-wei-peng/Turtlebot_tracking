#pragma once

#include <cmath>
#include <queue>


namespace controller {

class PID {

public:
    PID() = default;
    PID(double dt, 
        double max, 
        double min, 
        double Kp, 
        double Ki, 
        double Kd, 
        int max_queue_size);
    ~PID() = default;

    double calculate(const double setpoint, const double pv); 

private:
    double dt_;
    double max_;
    double min_;
    double Kp_;
    double Ki_;
    double Kd_;
    double pre_error_;
    double integral_;

    std::queue<double> error_queue_;
    int max_queue_size;
};


} // namespace controller