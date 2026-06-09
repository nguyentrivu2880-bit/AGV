#ifndef SIMPLE_PID_H
#define SIMPLE_PID_H

#include <Arduino.h>

class SimplePID {
  public:
    float kp, ki, kd;

    float setpoint = 0;
    float integral = 0;
    float lastError = 0;

    float outMin = -255;
    float outMax = 255;

    float dt = 0.01;

    SimplePID(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
    }

    void setOutputLimits(float minOut, float maxOut) {
      outMin = minOut;
      outMax = maxOut;
    }

    float compute(float input, float setpoint) {

      float error = setpoint - input;

      integral += error * dt;
      integral = constrain(integral, -1000, 1000);

      float derivative = (error - lastError) / dt;

      float output = kp * error + ki * integral + kd * derivative;

      output = constrain(output, outMin, outMax);

      lastError = error;

      return output;
    }
};

#endif