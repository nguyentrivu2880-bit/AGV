#ifndef Motor_h
#define Motor_h
#include "Arduino.h"

#define PWM_FREQ        5000   // Tần số PWM (Hz)
#define PWM_RESOLUTION  8      // Độ phân giải 8-bit (0–255)

class Motor {
  public:
    // ch_plus / ch_minus: kênh LEDC (0–15), mỗi motor dùng 2 kênh riêng
    Motor(int plus, int minus, int en_a, int en_b, int ch_plus, int ch_minus);
    void rotate(int value);

    int plus, minus;
    int en_a, en_b;

  private:
    int _ch_plus;
    int _ch_minus;
};

#endif