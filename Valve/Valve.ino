#include <SPI.h>
#include <string.h>

// https://files.seeedstudio.com/wiki/XIAO-RP2040/res/Seeed-Studio-XIAO-RP2040-v1.3.pdf
#define TWE_Valve D2
#define TWE_Drain D3
#define TWE_CHECK D4
#define PIN_Drain D0
#define PIN_Valve D1  // RP2040:27
#define NC D7

// https://tsunelab-programming.com/raspipico-sg90
// https://www.denshi.club/parts/2021/04/raspberry-pi-pico-7.html
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
uint32_t period_pwmclk = 25000;  //value of period cycles **CPU clock 125MHz**
const float open_deg = 45;
const float close_deg = -45;

bool need_over_close = true;

typedef enum {
  STANDBY,
  CHECK,
  VALVE,
  TWE_DRAIN,
  ES_DRAIN
} MODE;
MODE mode = STANDBY;

const int count_TWE_threshold = 10;
int count_TWE_Valve = 0;
int count_TWE_Drain = 0;

const unsigned long int Servo_open_period = 20000;  //20s
unsigned long int Servo_Valve_open_time = 0;
const unsigned long int Drain_open_period = 120000;  //120s, 2min
unsigned long int Drain_open_time = 0;

unsigned long int checkmode_enter_time = 0;

bool timer_10Hz = false;

void setup() {
  pinMode(PIN_Drain, OUTPUT);
  digitalWrite(PIN_Drain, HIGH);

  pinMode(TWE_CHECK, INPUT);
  pinMode(TWE_Valve, INPUT);
  pinMode(TWE_Drain, INPUT);

  Servo_init(PIN_Valve);
  Servo_write(PIN_Valve, close_deg);

  Serial.begin(115200);
  Serial1.begin(115200);
  delay(500);

  analogReadResolution(12);
}

void loop() {
  bool es_drain_start = false;
  bool es_drain_stop = false;
  bool es_valve = false;

  bool twe_valve = false;
  bool twe_drain = false;

  while (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    data.trim();

    if (data == "drain-start") {
      es_drain_start = true;
    }
    if (data == "drain-stop") {
      es_drain_stop = true;
    }
    if (data == "valve") {
      es_valve = true;
    }
    if (data == "valve-check") {
      mode = CHECK;
      checkmode_enter_time = millis();
    }
  }

  if (digitalRead(TWE_Valve) == LOW) {
    count_TWE_Valve++;
  } else {
    count_TWE_Valve = 0;
  }
  if (digitalRead(TWE_Drain) == LOW) {
    count_TWE_Drain++;
  } else {
    count_TWE_Drain = 0;
  }
  if (count_TWE_Valve > 0 && count_TWE_Drain > 0) {
    Serial.println("TWELITE_ERR");
    count_TWE_Valve = 0;
    count_TWE_Drain = 0;
  }

  if (count_TWE_Valve > count_TWE_threshold) {
    twe_valve = true;
  }

  if (count_TWE_Drain > count_TWE_threshold) {
    twe_drain = true;
  }

  switch (mode) {
    case STANDBY:
      if (twe_valve || es_valve) {
        float now_pwm = close_deg;

        /*
        now_pwm += 50;
        Valve_Servo.write(now_pwm);
        delay(500);
        for (int i = 0; i < 25; i++) {
          now_pwm += 1;
          Valve_Servo.write(now_pwm);
          delay(40);
        }
        for (int i = 0; i < 48; i++) {
          now_pwm += 1;
          Valve_Servo.write(now_pwm);
          delay(4);
        }*/

        now_pwm = open_deg;
        //Valve_Servo.write(now_pwm);
        Servo_write(PIN_Valve, now_pwm);

        Servo_Valve_open_time = millis();
        mode = VALVE;
        need_over_close = true;
      }
      if (twe_drain) {
        digitalWrite(PIN_Drain, LOW);
        Drain_open_time = millis();
        mode = TWE_DRAIN;
      }
      if (es_drain_start) {
        digitalWrite(PIN_Drain, LOW);
        Drain_open_time = millis();
        mode = ES_DRAIN;
      }
      break;

    case CHECK:
      if (millis() - checkmode_enter_time >= 5000) {
        toSTANDBY();
      }
      break;

    case VALVE:
      if (millis() - Servo_Valve_open_time >= Servo_open_period) {
        if (need_over_close) {
          //Valve_Servo.write(close_deg - 10);
          Servo_write(PIN_Valve, close_deg - 10);
          delay(1000);
          need_over_close = false;
        }
        //Valve_Servo.write(close_deg);
        Servo_write(PIN_Valve, close_deg);
        if (!twe_valve) {
          toSTANDBY();
        }
        if (es_drain_start) {
          digitalWrite(PIN_Drain, LOW);
          Drain_open_time = millis();
          mode = ES_DRAIN;
        }
      }
      break;

    case TWE_DRAIN:
      if (!twe_drain) {
        toSTANDBY();
      }
      if (millis() - Drain_open_time >= Drain_open_period) {
        digitalWrite(PIN_Drain, HIGH);
        //Valve_Servo.write(close_deg);
        Servo_write(PIN_Valve, close_deg);
      }
      break;

    case ES_DRAIN:
      if (es_drain_stop) {
        toSTANDBY();
      }
      if (millis() - Drain_open_time >= Drain_open_period) {
        toSTANDBY();
      }
      break;

    default:
      break;
  }

  static uint32_t last_send_ms = 0;
  if (millis() - last_send_ms > 50) {
    last_send_ms = millis();
    switch (mode) {
      case STANDBY:
        Serial1.write('/');
        break;
      case CHECK:
        Serial1.write('C');
        break;
      case VALVE:
        Serial1.write('V');
        break;
      case TWE_DRAIN:
        Serial1.write('D');
        break;
      case ES_DRAIN:
        Serial1.write('d');
        break;
      default:
        break;
    }
  }
}

void toSTANDBY() {
  mode = STANDBY;
  //Valve_Servo.write(close_deg);
  Servo_write(PIN_Valve, close_deg);
  digitalWrite(PIN_Drain, HIGH);
}

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

static uint16_t set_pwm_duty(uint16_t period_cycle, float cycletime, float hightime) {
  float count_pms = (float)period_cycle / cycletime * hightime;
  return ((uint16_t)count_pms);
}