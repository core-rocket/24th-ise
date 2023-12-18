#include <SPI.h>
#include <string.h>
#include <Servo.h>

#define TWE_Valve D2
#define TWE_Drain D3
#define TWE_CHECK D4
#define Drain D0
#define Valve D1
#define NC D7



typedef enum {
  STANDBY,
  CHECK,
  VALVE,
  TWE_DRAIN,
  ES_DRAIN
} MODE;
MODE mode = STANDBY;

Servo Valve_Servo;
const int open_pwm = 168;
const int close_pwm = 50;

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
  pinMode(Drain, OUTPUT);
  digitalWrite(Drain, HIGH);

  pinMode(TWE_CHECK, INPUT);
  pinMode(TWE_Valve, INPUT);
  pinMode(TWE_Drain, INPUT);


  Valve_Servo.attach(Valve);
  Valve_Servo.write(close_pwm);

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
        int now_pwm = close_pwm;

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
        Valve_Servo.write(now_pwm);

        Servo_Valve_open_time = millis();
        mode = VALVE;
        need_over_close = true;
      }
      if (twe_drain) {
        digitalWrite(Drain, LOW);
        Drain_open_time = millis();
        mode = TWE_DRAIN;
      }
      if (es_drain_start) {
        digitalWrite(Drain, LOW);
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
          Valve_Servo.write(close_pwm - 10);
          delay(1000);
          need_over_close = false;
        }
        Valve_Servo.write(close_pwm);
        if (!twe_valve) {
          toSTANDBY();
        }
        if (es_drain_start) {
          digitalWrite(Drain, LOW);
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
        digitalWrite(Drain, HIGH);
        Valve_Servo.write(close_pwm);
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
  Valve_Servo.write(close_pwm);
  digitalWrite(Drain, HIGH);
}
