#include <Arduino.h>
#include "myOpener.h"

// #include <Servo.h>
// https://tsunelab-programming.com/raspipico-sg90
// https://www.denshi.club/parts/2021/04/raspberry-pi-pico-7.html
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
uint32_t period_pwmclk = 0;
const pin_size_t SERVO_1 = 21;
const pin_size_t SERVO_2 = 22;
// Servo servo1;
// Servo servo2;

void Servo_init(int pin) {
  period_pwmclk = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS) / 5;
  // 125MHz(build option, selectable) / 100div(pwm_config_set_clkdiv_int) / 50Hz(Servo) = 125000kHz / 5

  // Serial.print("CLOCKS_FC0_SRC_VALUE_CLK_SYS:");
  // Serial.println(frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS), DEC);
  // Serial.print("period_pwmclk:");
  // Serial.println(period_pwmclk, DEC);

  gpio_set_function(pin, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO port number (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(pin);

  // get default pwm confg
  pwm_config cfg = pwm_get_default_config();

  // set pwm config modified div mode and div int value
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
  pwm_config_set_clkdiv_int(&cfg, 100);
  pwm_init(slice_num, &cfg, false);

  // Set period of period_pwmclk (0 to period_pwmclk-1 inclusive)
  pwm_set_wrap(slice_num, (period_pwmclk - 1));
  // Set channel A or B output high for one cycle before dropping
  if (pin % 2) {  //odd number
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  } else {  //even numberf
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
  }
}

static uint16_t set_pwm_duty(uint16_t period_cycle, float cycletime, float hightime) {
  float count_pms = (float)period_cycle / cycletime * hightime;
  return ((uint16_t)count_pms);
}

void Servo_write(int pin, float angle_deg) {
  float pulse_ms = (angle_deg * 1.9 / 180) + 1.45;
  static float period_ms = 20.0;
  uint16_t count = set_pwm_duty(period_pwmclk, period_ms, pulse_ms);

  // Find out which PWM slice is connected to GPIO port number (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(pin);

  // Set channel A output high for one cycle before dropping
  if (pin % 2) {  //odd number
    pwm_set_chan_level(slice_num, PWM_CHAN_B, count);
  } else {  //even number
    pwm_set_chan_level(slice_num, PWM_CHAN_A, count);
  }

  // Set the PWM running
  pwm_set_enabled(slice_num, true);
}

void MY_OPENER::init() {
  //servo1.attach(SERVO_1);
  //servo2.attach(SERVO_2);
  Servo_init(SERVO_1);
  OPENER::init();
}

void MY_OPENER::open() {
  //servo1.write(93);
  //servo2.write(93);
  Servo_write(SERVO_1, 9);
}

void MY_OPENER::close() {
  //servo1.write(30);
  //servo2.write(30);
  Servo_write(SERVO_1, -53);
}
