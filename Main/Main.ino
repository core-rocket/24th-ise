// global data
float data_bno_accel_x_mss = 0;
float data_bno_accel_y_mss = 0;
float data_bno_accel_z_mss = 0;
float data_bme_pressure_hPa = 0;
float data_bme_temperature_degC = 0;
float data_bme_altitude_m = 0;
uint32_t data_gnss_latitude_udeg = 0;
uint32_t data_gnss_longitude_udeg = 0;
float data_bat_v = 0;
float data_ext_v = 0;
bool data_key_sw_active = false;

// pinout
const pin_size_t MIF_TX = 2;
const pin_size_t MIF_RX = 3;
const pin_size_t BNO_SDA = 4;
const pin_size_t BNO_SCL = 5;
const pin_size_t VALVE_TX = 6;
const pin_size_t VALVE_RX = 7;
const pin_size_t ES920_TX = 8;
const pin_size_t ES920_RX = 9;
const pin_size_t BME_SDA = 10;
const pin_size_t BME_SCL = 11;
const pin_size_t E220_TX = 12;
const pin_size_t E220_RX = 13;
const pin_size_t GNSS_TX = 14;
const pin_size_t GNSS_RX = 15;
const pin_size_t KEY_SW = 20;
// const pin_size_t SERVO_1 = 21;
// const pin_size_t SERVO_2 = 22;
const pin_size_t EXT_V = 26;
const pin_size_t BAT_V = 27;

// Servo


// KeySW


// BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t accelerometerData;

// BME280
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
const float SEALEVELPRESSURE_HPA = 1013.25;

// SAM-M8Q
#include <TinyGPS++.h>
TinyGPSPlus gps;
SerialPIO Serial_GNSS(GNSS_TX, GNSS_RX, 512);
// https://arduino-pico.readthedocs.io/en/latest/piouart.html

// ES920LR
#define Serial_ES920 Serial2
// https://moyoi-memo.hatenablog.com/entry/2022/02/15/112100
String downlink = "";
String response = "";
bool need_response_usb = false;
bool need_response_es920 = false;

// W25Q128

// Opener
#include "myOpener.h"
MY_OPENER opener(OPENER::SHINSASYO);

// Valve
char valve_mode = '/';
SerialPIO Serial_Valve(VALVE_TX, VALVE_RX, 32);

// MissionInterface
SerialPIO Serial_MIF(MIF_TX, MIF_RX, 256);

// setup()ではdelay()使用可
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  // デバッグ出力
  Serial.begin(115200);

  Wire.setSDA(BNO_SDA);
  Wire.setSCL(BNO_SCL);
  while (!bno.begin()) {
    Serial.print("BNO055 ERR");
    delay(100);
  }
  bno.setMode(OPERATION_MODE_CONFIG);
  delay(25);
  uint8_t savePageID = bno.read8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR);
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 0X01);
  bno.write8(Adafruit_BNO055::BNO055_ACCEL_DATA_X_LSB_ADDR, 0X17); // page2なのでホントはACC_DATA_X_LSBではなくACC_DATA_X_LSBにアクセス
  delay(10);
  bno.write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, savePageID);
  bno.setMode(OPERATION_MODE_ACCONLY);
  delay(20);

  Wire1.setSDA(BME_SDA);
  Wire1.setSCL(BME_SCL);
  while (!bme.begin(0x76, &Wire1)) {
    Serial.print("BME280 ERR");
    delay(100);
  }
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,
    Adafruit_BME280::SAMPLING_X1,
    Adafruit_BME280::SAMPLING_X2,
    Adafruit_BME280::SAMPLING_NONE,
    Adafruit_BME280::FILTER_X16,
    Adafruit_BME280::STANDBY_MS_0_5);

  analogReadResolution(12);
  pinMode(KEY_SW, INPUT);

  Serial_ES920.setTX(ES920_TX);
  Serial_ES920.setRX(ES920_RX);
  Serial_ES920.setFIFOSize(64);
  Serial_ES920.begin(115200);
  while (Serial_ES920.available()) {
    Serial_ES920.read();
  }

  Serial_GNSS.begin(9600);
  Serial_Valve.begin(115200);
  Serial_MIF.begin(115200);

  opener.init();
}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  static uint32_t time_100Hz = 0;
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;

    static int count_10Hz = 0;
    count_10Hz++;
    if (count_10Hz > 10) {
      count_10Hz = 0;

      // 10Hzで実行する処理

      data_bat_v = analogRead(BAT_V) * 3.3 * 11 / (1 << 12);
      data_ext_v = analogRead(EXT_V) * 3.3 * 11 / (1 << 12);

      // フライト = 回路としてOPEN = LOW
      // バッテリー駆動でなくUSB駆動の場合常にLOWなので除外
      data_key_sw_active = (digitalRead(KEY_SW) == LOW) && data_bat_v > 1;
      static bool last_data_key_sw_active = false;
      if (data_key_sw_active && !last_data_key_sw_active && opener.mode == OPENER::CHECK) {
        opener.goREADY();
      }
      if (!data_key_sw_active && last_data_key_sw_active) {
        if (opener.open_judge.prohibitOpen) {
          opener.goCHECK();
        } else {
          opener.goCHECK();
          opener.clear_prohibitOpen();
        }
      }
      last_data_key_sw_active = data_key_sw_active;

      // デバッグ出力
      if (need_response_usb) {
        Serial.println("response:" + response);
        need_response_usb = false;
      } else {
        Serial.println(downlink);
      }
      // Serial.print("BNO055, ");
      // Serial.print(data_bno_accel_x_mss);
      // Serial.print(", ");
      // Serial.print(data_bno_accel_y_mss);
      // Serial.print(", ");
      // Serial.print(data_bno_accel_z_mss);
      // Serial.print(", ");

      // Serial.print("BME280, ");
      // Serial.print(data_bme_pressure_hPa);
      // Serial.print(", ");
      // Serial.print(data_bme_temperature_degC);
      // Serial.print(", ");
      // Serial.print(data_bme_altitude_m);
      // Serial.print(", ");

      // Serial.print("GNSS, ");
      // Serial.print(data_gnss_latitude_udeg);
      // Serial.print(", ");
      // Serial.print(data_gnss_longitude_udeg);
      // Serial.print(", ");

      // Serial.print("voltage, ");
      // Serial.print(data_bat_v);
      // Serial.print(", ");
      // Serial.print(data_ext_v);
      // Serial.print(", ");
      // Serial.print("\n");
    }

    // 100Hzで実行する処理


    // BNO055から100Hzで測定
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    data_bno_accel_x_mss = accelerometerData.acceleration.x;
    data_bno_accel_y_mss = accelerometerData.acceleration.y;
    data_bno_accel_z_mss = accelerometerData.acceleration.z;

    // BME280から100Hzで測定
    data_bme_pressure_hPa = bme.readPressure() / 100.0F;
    data_bme_temperature_degC = bme.readTemperature();
    data_bme_altitude_m = bme.readAltitude(SEALEVELPRESSURE_HPA);

    bool new_judge = opener.opener_100Hz(-data_bno_accel_z_mss, data_bme_altitude_m);
  }


  // 常に実行する処理

  // SAM-M8QのUARTを常に読み出し
  while (Serial_GNSS.available()) {
    gps.encode(Serial_GNSS.read());
    if (gps.location.isUpdated()) {
      data_gnss_latitude_udeg = gps.location.lat() * 1000000;
      data_gnss_longitude_udeg = gps.location.lng() * 1000000;
    }
  }

  // バルブ電装状態受信
  while (Serial_Valve.available()) {
    valve_mode = Serial_Valve.read();
  }


  // テレメトリ生成
  downlink = "";
  switch (opener.mode) {
    case OPENER::CHECK:
      downlink += 'C';
      break;
    case OPENER::READY:
      downlink += 'R';
      break;
    case OPENER::FLIGHT:
      downlink += 'F';
      break;
    case OPENER::OPENED:
      downlink += 'O';
      break;
  }
  downlink += opener.open_judge.prohibitOpen ? 'E' : '/';       // Emergency
  downlink += opener.open_judge.apogee_time ? 'T' : '/';        // Time
  downlink += opener.open_judge.apogee_descending ? 'P' : '/';  // Pressure
  downlink += opener.open_judge.meco_time ? 'T' : '/';          // Time
  downlink += opener.open_judge.meco_acc ? 'A' : '/';           // Acceleration
  if (opener.lift_off_judge == OPENER::NONE) {
    downlink += '/';
  }
  if (opener.lift_off_judge == OPENER::ACCSEN) {
    downlink += 'A';
  }
  if (opener.lift_off_judge == OPENER::ALTSEN) {
    downlink += 'P';
  }
  downlink += valve_mode;
  downlink += String(data_key_sw_active ? 'K' : '/') + ',';  // Key
  downlink += String(data_bno_accel_z_mss, 1) + ',';
  downlink += String(data_bme_temperature_degC, 1) + ',';
  downlink += String(data_bme_altitude_m, 1) + ',';
  downlink += String(data_gnss_latitude_udeg % 1000000) + ',';
  downlink += String(data_gnss_longitude_udeg % 1000000) + ',';
  downlink += String(data_bat_v, 1) + ',';
  downlink += String(static_cast<int>(data_ext_v));

  // テレメトリダウンリンク
  const uint32_t downlink_rate_ms = 2500;
  static uint32_t last_downlink_ms = 0;
  if (millis() - last_downlink_ms > downlink_rate_ms) {
    last_downlink_ms = millis();
    if (need_response_es920) {
      Serial_ES920.print("response:" + response + "\r\n");
      need_response_es920 = false;
    } else {
      Serial_ES920.print(downlink + "\r\n");
    }
  }


  // コマンドアップリンク
  String uplink = "";
  if (Serial.available()) {
    uplink = Serial.readStringUntil('\n');
  }
  if (Serial_ES920.available()) {
    uplink = Serial_ES920.readStringUntil('\n');
  }
  uplink.trim();
  // if (uplink != "") {
  //   Serial.print("uplink:");
  //   Serial.println(uplink);
  // }

  if (uplink.indexOf("NG") == 0) {
    Serial.println(uplink);
  }

  if (uplink == "emst") {
    opener.prohibitOpen();
  }
  if (uplink == "clr") {
    opener.clear_prohibitOpen();
  }
  if (uplink == "open") {
    opener.goCHECK();
    opener.clear_prohibitOpen();
    opener.manualOpen();
  }
  if (uplink == "close") {
    opener.goCHECK();
    opener.manualClose();
  }
  if (uplink == "check") {
    opener.goCHECK();
  }
  if (uplink == "ready") {
    opener.goREADY();
  }

  if (uplink == "drain-start") {
    Serial_Valve.print("drain-start\n");
  }
  if (uplink == "drain-stop") {
    Serial_Valve.print("drain-stop\n");
  }
  if (uplink == "valve") {
    Serial_Valve.print("valve\n");
  }
  if (uplink == "valve-check") {
    Serial_Valve.print("valve-check\n");
  }

  if (uplink == "mif-on") {
    Serial_MIF.print("mif-on\n");
  }
  if (uplink == "mif-off") {
    Serial_MIF.print("mif-off\n");
  }

  float uplink_float = uplink.toFloat();
  if (uplink_float != 0) {
    opener.set_open_threshold_time_ms(uplink_float * 1000);
    response = "open:" + String(static_cast<float>(opener.get_open_threshold_time_ms()) / 1000.0, 2);
    need_response_usb = true;
    need_response_es920 = true;
  }
}
