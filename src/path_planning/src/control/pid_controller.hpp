#pragma once

#include <algorithm>
#include <cmath>

/** @brief A standard Proportional-Integral-Derivative (PID) Controller */
class PIDController {
private:
    double kp;            /**< Proportional gain: Reacts to the current error */
    double ki;            /**< Integral gain: Reacts to the accumulation of past errors */
    double kd;            /**< Derivative gain: Reacts to the rate of change of the error */

    double integral;     /**< Accumulator for the I-term */
    double prev_error;   /**< Stores the error from the previous time step */

    double max_integral; /**< Limit to prevent the I-term from growing infinitely */
public:
    /**
     * @brief Constructs the PID controller with specific tuning gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param max_i The maximum absolute value the integral accumulator is allowed to reach
     */
    PIDController(double kp, double ki, double kd, double max_i = 1.0)
        : kp(kp), ki(ki), kd(kd), integral(0.0), prev_error(0.0), max_integral(max_i) {}

    /**
     * @brief Computes the control output required to reach the target value
     * @param target_value The desired state
     * @param current_value The current state
     * @param dt The time delta in seconds since the last calculation
     * @return The control effort to apply
     */
    double calculate(double target_value, double current_value, double dt);
};

double PIDController::calculate(double target_value, double current_value, double dt) {
    if (dt <= 0.0) return 0.0;

    // Calculate error
    double error = target_value - current_value;

    // Calculate Proportional Term
    double p_term = kp * error;

    // Calculate Integral Gain
    integral += error * dt;
    integral = std::max(-max_integral, std::min(max_integral, integral));
    double i_term = ki * integral;

    // Calculate Derivative gain
    double d_term = kd * ((error - prev_error) / dt);
    prev_error = error;

    // Return p + i + d
    return p_term + i_term + d_term;
}