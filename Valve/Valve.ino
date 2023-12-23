#include <SPI.h>
#include <string.h>

#define TWE_Valve D2
#define TWE_Drain D3
#define TWE_CHECK D4
#define PIN_Drain D0
#define PIN_Valve 27
#define NC D7

typedef enum {
  STANDBY,
  CHECK,
  VALVE,
  TWE_DRAIN,
  ES_DRAIN
} MODE;
MODE mode = STANDBY;


#include "pico/stdlib.h"
#include "hardware/pwm.h"
#define DF_MOT_PERIOD_CYCLE 25000  //value of period cycles
#define DF_MOT_CYCLE_TIME 20.00F   //time of one cycle[ms]
#define DF_MOT_DUTY_N90_DEG 0.50F  //-90deg time of high level[ms]
#define DF_MOT_DUTY_N70_DEG 0.71F  //-70deg time of high level[ms]
#define DF_MOT_DUTY_N65_DEG 0.76F  //-65deg time of high level[ms]
#define DF_MOT_DUTY_N60_DEG 0.82F  //-60deg time of high level[ms]
#define DF_MOT_DUTY_N30_DEG 1.13F  //-30deg time of high level[ms]
#define DF_MOT_DUTY_0_DEG 1.45F    //  0deg time of high level[ms]
#define DF_MOT_DUTY_P30_DEG 1.80F  //+30deg time of high level[ms]
#define DF_MOT_DUTY_P60_DEG 2.10F  //+60deg time of high level[ms]
#define DF_MOT_DUTY_P90_DEG 2.40F  //+90deg time of high level[ms]
const float open_pwm = 1.90F;
const float close_pwm = 1.00F;

bool need_over_close = true;

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

  gpio_set_function(PIN_Valve, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO port number (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(PIN_Valve);
  uint16_t count;

  // get default pwm confg
  pwm_config cfg = pwm_get_default_config();

  // set pwm config modified div mode and div int value
  pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
  pwm_config_set_clkdiv_int(&cfg, 100);
  pwm_init(slice_num, &cfg, false);

  // Set period of DF_MOT_PERIOD_CYCLE (0 to DF_MOT_PERIOD_CYCLE-1 inclusive)
  pwm_set_wrap(slice_num, (DF_MOT_PERIOD_CYCLE - 1));
  // Set channel A or B output high for one cycle before dropping
  if (PIN_Valve % 2) {  //odd number
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  } else {  //even numberf
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
  }
  Valve_Servo(close_pwm);

  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
}

void loop() {
  bool es_drain_start = false;
  bool es_drain_stop = false;
  bool es_valve = false;

  bool twe_valve = false;
  bool twe_drain = false;

  if (digitalRead(TWE_Valve) == LOW) count_TWE_Valve++;
  else
    count_TWE_Valve = 0;
  if (digitalRead(TWE_Drain) == LOW) count_TWE_Drain++;
  else
    count_TWE_Drain = 0;

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
        float now_pwm = close_pwm;

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

        now_pwm = open_pwm;
        //Valve_Servo.write(now_pwm);
        Valve_Servo(now_pwm);

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
          //Valve_Servo.write(close_pwm - 10);
          Valve_Servo(close_pwm - 0.1);
          delay(1000);
          need_over_close = false;
        }
        //Valve_Servo.write(close_pwm);
        Valve_Servo(close_pwm);
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
        //Valve_Servo.write(close_pwm);
        Valve_Servo(close_pwm);
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
}

void toSTANDBY() {
  mode = STANDBY;
  //Valve_Servo.write(close_pwm);
  Valve_Servo(close_pwm);
  digitalWrite(PIN_Drain, HIGH);
}

void Valve_Servo(float degree) {
  uint16_t count = set_pwm_duty(DF_MOT_PERIOD_CYCLE, DF_MOT_CYCLE_TIME, degree);

  // Find out which PWM slice is connected to GPIO port number (it's slice 0)
  uint snum = pwm_gpio_to_slice_num(PIN_Valve);

  // Set channel A output high for one cycle before dropping
  if (PIN_Valve % 2) {  //odd number
    pwm_set_chan_level(snum, PWM_CHAN_B, count);
  } else {  //even number
    pwm_set_chan_level(snum, PWM_CHAN_A, count);
  }

  // Set the PWM running
  pwm_set_enabled(snum, true);
}
static uint16_t set_pwm_duty(uint16_t period_cycle, float cycletime, float hightime) {
  float count_pms = (float)period_cycle / cycletime * hightime;
  return ((uint16_t)count_pms);
}