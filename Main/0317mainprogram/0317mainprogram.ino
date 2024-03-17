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
int emst = 0;
bool accelopen = false, altitudeopen = false;
int accelgetout = 0, altitudegetout = 0;

// pinout
const pin_size_t MIF_TX = 2;
const pin_size_t MIF_RX = 3;
const pin_size_t BNO_SDA = 4;
const pin_size_t BNO_SCL = 5;
const pin_size_t VALVE_TX = 6;
const pin_size_t VALVE_RX = 7;
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
#include <algorithm>  // 必要なヘッダーファイルをインクルード
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"
#define PIN_WIRE_SDA (9u)
#define PIN_WIRE_SCL (10u)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t accelerometerData;

sensors_event_t* event;
int acceljudge = 0;              //加速度による判定(0=未判定,1=離床判定,2=開放判定)
int accelcount = 0;              //閾値を5回満たすカウントをする
int phase_now = 0;               //現在のフェーズを区別(0=CHEAK,1=READY,2=FLIGHT,3=OPENED)
int Openjudge = 1;               //開放禁止コマンド用状態表示用(0=NOT_OPEN,1=OPEN)
double BNO[10];                  //BNOデータ取得、格納に用いる
unsigned long time_data_ms = 0;  //離床判定タイマー(燃焼終了検知)

//BNO設定
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address

// BME280
//BME280のライブラリーを取り込みます。
#include <Adafruit_BME280.h>
//1013.25は地球の海面上の大気圧の平均値(1気圧)です。
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
unsigned long delayTime;
float temperature;      //気温
float altitude;         //高度
int altitudejudge = 0;  //気圧による判定(0=未判定,1=離床判定,2=開放判定)
int altitudecount = 0;
double BME[10];  //BMEデータ取得、格納に用いる

// SAM-M8Q
#include <TinyGPS++.h>
TinyGPSPlus gps;
SerialPIO Serial_GNSS(GNSS_TX, GNSS_RX, 512);
// https://arduino-pico.readthedocs.io/en/latest/piouart.html

// ES920LR
#define Serial_ES920 Serial2
// https://moyoi-memo.hatenablog.com/entry/2022/02/15/112100
String downlink = "";
String uplink = "";
String uplink_string = "";

// W25Q128


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
  bno.write8(Adafruit_BNO055::BNO055_ACCEL_DATA_X_LSB_ADDR, 0X17);  // page2なのでホントはACC_DATA_X_LSBではなくACC_DATA_X_LSBにアクセス
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
  // pinMode(KEY_SW, INPUT);

  // Serial_ES920.setTX(ES920_TX);
  // Serial_ES920.setRX(ES920_RX);
  Serial_ES920.setTX(8);
  Serial_ES920.setRX(9);
  Serial_ES920.setFIFOSize(64);
  Serial_ES920.begin(115200);
  while (Serial_ES920.available()) {
    Serial_ES920.read();
  }

  Serial_GNSS.begin(9600);
  Serial_Valve.begin(115200);
  Serial_MIF.begin(115200);

  //opener.init();
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
      getevent();
      data_bat_v = analogRead(BAT_V) * 3.3 * 11 / (1 << 12);
      data_ext_v = analogRead(EXT_V) * 3.3 * 11 / (1 << 12);

      // フライト = 回路としてOPEN = LOW
      // バッテリー駆動でなくUSB駆動の場合常にLOWなので除外

      data_key_sw_active = (digitalRead(KEY_SW) == LOW) && data_bat_v > 1;
      static bool last_data_key_sw_active = false;
      if (data_key_sw_active && !last_data_key_sw_active && phase_now == 0) {
        phase_now = 1;
      }
      // if (!data_key_sw_active && last_data_key_sw_active) {
      //   if (opener.open_judge.prohibitOpen) {
      //     gocheck();
      //   } else {
      //     gocheck();
      //     opener.clear_prohibitOpen();
      //   }
      // }
      last_data_key_sw_active = data_key_sw_active;

      // デバッグ出力
      /*
      Serial.print("BNO055, ");
      Serial.print(data_bno_accel_x_mss);
      Serial.print(", ");
      Serial.print(data_bno_accel_y_mss);
      Serial.print(", ");
      Serial.print(data_bno_accel_z_mss);
      Serial.print(", ");

      Serial.print("BNO055, ");
      Serial.print(data_bme_pressure_hPa);
      Serial.print(", ");
      Serial.print(data_bme_temperature_degC);
      Serial.print(", ");
      Serial.print(data_bme_altitude_m);
      Serial.print(", ");

      Serial.print("GNSS, ");
      Serial.print(data_gnss_latitude_udeg);
      Serial.print(", ");
      Serial.print(data_gnss_latitude_udeg);
      Serial.print(", ");
      */
      // if (need_response_usb) {
      //   Serial.println("response:" + response);
      //   need_response_usb = false;
      // } else {
      //   Serial.println(downlink);
      // }
    }

    //100Hz
    // BME280から100Hzで測定
    if (altitudejudge == 0) {
      if (altitudecount > 2) {  //2回連続で閾値を超えたときの処理
        altitudejudge = 1;      //離床判定・フェーズ移行
        altitudecount = 0;      //開放判定に向けた初期化
      } else {
        if (calculateMediumBME() < -190) {
          altitudecount++;
        } else {
          altitudecount = 0;  //初期化
        }
      }
    }
    if (altitudejudge == 1) {
      if (altitudecount > 2) {  //5回連続で閾値を下回ったときの処理
        altitudejudge = 2;      //開放判定・フェーズ移行
      }
      if (calculateMediumBME() > -160) {
        altitudecount++;
      } else {
        altitudecount = 0;  //初期化
      }
    }
  }

  // 常に実行する処理

  // コマンドアップリンク
  //ES920LR設定用
  // if (Serial.available()) {
  //   downlink = Serial.readStringUntil('\n');
  //   Serial.println(downlink);
  // }
    if (Serial.available()) {
    char c = (char)Serial.read();
    Serial_ES920.write(c);
  }

  // if (Serial_ES920.available()) {  //動作検証用(使わない)
  //   uplink_string = Serial_ES920.read();
  //   uplink_string.trim();
  //   Serial.println(uplink_string);
  // }

  uplink = "";
  if (Serial_ES920.available()) {
    uplink = Serial_ES920.readStringUntil('\n');
  }
  if (uplink != "") {  //動作検証用(使わない)
    Serial.print("uplink:");
    Serial.println(uplink);
  }
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t accelerometerData;
  //CHEACK・NOTOPEN・READYコマンド受信
  //READYコマンド(CHECKphaseであることが条件)
  if (phase_now == 0) {
    if (uplink.substring(0, uplink.length() - 1) == "READY") {  //READYコマンド
      phase_now = 1;                                            // "READY"が入力された場合、phase_nowに１を代入する
      Serial.println("READY_transitioned");                     //動作確認用
      Serial.println(phase_now);
    }
  }
  if (uplink.substring(0, uplink.length() - 1) == "emst") {
    emst = 1;
    Serial.println("emergency");  //動作確認用
    Serial.println("NOTopened");
  }
  if (uplink.substring(0, uplink.length() - 1) == "CHECK") {  //CHECKコマンド
    gocheck();                                                // "CHECK"が入力された場合、phaseに0を代入
    Serial.println("CHECK_transitioned");                     //動作確認用
  }


  //離床判定
  if (phase_now == 1 && (altitudejudge == 1 || acceljudge == 1)) {  //READYフェーズに移行かつ、離床判定後(加速度or気圧で判定)である場合
    //高度による離床判定
    //10Hz処理に記述

    //加速度による離床判定
    //10Hz処理に記述


    //高度・加速度による離床判定
    if (acceljudge == 1) {                          //加速度による離床判定
      phase_now = 2;                                //フェーズ移行
      Serial.println("FLIGHT_accel_transitioned");  //シリアルモニタに状態移行を表示
      accelgetout = 1;
      time_data_ms = millis();                                   //離床判定タイマー
    } else if (altitudejudge == 1 /*高度による離床判定条件*/) {  //高度による離床判定
      phase_now = 2;                                             //フェーズ移行
      Serial.println("FLIGHT_altitude_transitioned");            //シリアルモニタに状態移行を表示
      altitudegetout = 1;
      time_data_ms = millis();  //離床判定タイマー
    }
  }

  //開放判定
  if (phase_now == 2) {
    //高度による開放判定
    //100Hz処理に記述する

    //加速度による開放判定
    //100Hz処理に記述

    //高度・加速度による開放判定
    //加速度での開放判定
    if ((acceljudge == 2) && (phase_now == 2) && (millis() - time_data_ms > 12000 /*とりあえず燃焼時間12秒設定*/)) {
      accelopen = true;
    }
    //高度による開放判定
    if ((altitudejudge == 2) || (millis() - time_data_ms > 12000 /*とりあえず燃焼時間120秒設定*/)) {
      altitudeopen = true;
    }
    if (((accelopen) && (altitudeopen) && (emst == 0))) {  //高度と加速度の閾値超過連続回数がともに5回以上かつ開放禁止コマンドが入力されていないとき
      phase_now = 3;                                       //フェーズ移行
      Serial.println("OPENED_transitioned");
    }
  }

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
  if (phase_now == 0) {
    downlink += 'C';
  } else if (phase_now == 1) {
    downlink += 'R';
  } else if (phase_now == 2) {
    downlink += 'F';
  } else if (phase_now == 3) {
    downlink += 'O';
  }

  if (emst == 0) {
    downlink += '/';  // Emergency
  } else {
    downlink += 'E';  // Emergency
  }

  //離床条件
  if (altitudejudge >= 1) {
    downlink += 'P';  // Pressure
  } else {
    downlink += '/';  // Pressure
  }
  if (acceljudge >= 1) {
    downlink += 'A';
  } else {
    downlink += '/';
  }
  if (accelgetout == 1) {
    downlink += 'a';
  } else if (altitudegetout == 1) {
    downlink += 'p';

  } else {
    downlink += '/';
  }

  //開放条件
  if (phase_now >= 2) {
    if (millis() - time_data_ms >= 5000) {
      downlink += 'T';  // mecoTime
    } else {
      downlink += '/';  // mecoTime
    }
    if (millis() - time_data_ms >= 12000) {
      downlink += 'T';  // apogeeTime
    } else {
      downlink += '/';  //apogeeTime
    }
  } else {
    downlink += '/';
    downlink += '/';
  }
  if (acceljudge == 2) {
    downlink += 'A';
  } else {
    downlink += '/';
  }
  if (altitudejudge == 2) {
    downlink += 'P';  // Pressure
  } else {
    downlink += '/';
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
    Serial_ES920.println(downlink);
    Serial.println(downlink);
  }
}

//BNO055の100Hzで測定する際に用いる関数
//BNOからセンサ値が0でない値のみを返す関数
double getBNO(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -10000000;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;

  } else {
    Serial.print("Unk:");
  }

  return z;
}
//BNOの中央値算出
double calculateMedianBNO(sensors_event_t* event) {
  double BNO[10];
  for (int i = 0; i < 10; i++) {
    BNO[i] = getBNO(event);
  }

  // 中央値を計算するために配列をソートする
  std::sort(BNO, BNO + 10);
  double median = BNO[4];  // 中央値

  data_bno_accel_z_mss = median;  // データを更新
  return median;
}

//BMEデータ中央値計算(気圧値)
double calculateMediumBME() {
  double BME[4];
  double tmp = 0, medium = 0, Medium = 0, MEDIUM;
  for (int i = 0; i < 4; i++) {
    data_bme_temperature_degC = bme.readTemperature();
    BME[i] = bme.readAltitude(SEALEVELPRESSURE_HPA);
  }

  // 中央値を計算するために配列をソートする
  std::sort(BME, BME + 4);
  medium = BME[2];  // 中央値
  Medium = MEDIUM;
  MEDIUM = (Medium - medium) / 0.5;
  return MEDIUM;
}


void gocheck() {
  acceljudge = 0;     //加速度による判定(0=未判定,1=離床判定,2=開放判定)
  accelcount = 0;     //閾値を5回満たすカウントをする
  altitudejudge = 0;  //高度による判定(0=未判定,1=離床判定,2=開放判定)
  altitudecount = 0;  //閾値を5回満たすカウントをする
  phase_now = 0;      //現在のフェーズを区別(0=CHEAK,1=READY,2=FLIGHT,3=OPENED)
  emst = 0;           //開放禁止コマンド用状態表示用(0=NOT_OPEN,1=OPEN)
  altitudegetout = 0;
  accelgetout = 0;
  return;
}
void getevent() {
  // 100Hzで実行する処理
  // BNO055から100Hzで測定
  if (acceljudge == 0) {
    if (calculateMedianBNO(&accelerometerData) <= -9) {  //中央値が閾値を超えた回数カウント
      accelcount++;
    } else {
      accelcount = 0;  //初期化
    }

    if (accelcount > 5) {  //5回連続で閾値を超えたときの処理
      acceljudge = 1;      //離床判定・フェーズ移行
      accelcount = 0;      //開放判定に向けた初期化
    }
  }
  if (acceljudge == 1) {
    if (calculateMedianBNO(&accelerometerData) > -3) {  //中央値が閾値を下回った回数カウント
      accelcount++;
    } else {
      accelcount = 0;  //初期化
    }

    if (accelcount > 5) {  //5回連続で閾値を下回ったときの処理
      acceljudge = 2;      //開放判定・フェーズ移行
    }
  }
  return;
}
