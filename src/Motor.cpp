#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int plus, int minus, int en_a, int en_b, int ch_plus, int ch_minus) {
  Motor::plus  = plus;
  Motor::minus = minus;
  Motor::en_a  = en_a;
  Motor::en_b  = en_b;
  _ch_plus     = ch_plus;
  _ch_minus    = ch_minus;

  // ESP32: khởi tạo LEDC thay cho analogWrite
  ledcSetup(_ch_plus,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(_ch_minus, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(plus,  _ch_plus);
  ledcAttachPin(minus, _ch_minus);

  // Encoder dùng INPUT_PULLUP (tránh dùng GPIO 34,35,36,39 vì không có pull-up)
  pinMode(en_a, INPUT_PULLUP);
  pinMode(en_b, INPUT_PULLUP);
}

void Motor::rotate(int value) {
  if (value >= 0) {
    // (12V / 16V) * 255 ≈ 190
    int out = map(value, 0, 100, 0, 190);
    ledcWrite(_ch_plus,  out);
    ledcWrite(_ch_minus, 0);
  } else {
    int out = map(value, 0, -100, 0, 190);
    ledcWrite(_ch_plus,  0);
    ledcWrite(_ch_minus, out);
  }
}