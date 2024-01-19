// global data
float data_bno_accel_x_mss = 0;
float data_bno_accel_y_mss = 0;
float data_bno_accel_z_mss = 0;
float data_bme_pressure_hPa = 0;
float data_bme_temperature_degC = 0;
float data_bme_altitude_m = 0;
uint32_t data_gnss_latitude_udeg = 0;
uint32_t data_gnss_longitude_udeg = 0;

// Servo


// KeySW


// BNO055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

sensors_event_t* event;
int acceljudge = 0;           //加速度による判定(0=未判定,1=離床判定,2=開放判定)
int altitudejudge = 0;        //気圧による判定(0=未判定,1=離床判定,2=開放判定)
int accelcount = 0;           //閾値を5回満たすカウントをする
int phase_now = 0;            //現在のフェーズを区別(0=CHEAK,1=READY,2=FLIGHT,3=OPENED)
int Openjudge = 1;            //開放禁止コマンド用状態表示用(0=NOT_OPEN,1=OPEN)
double BNO[10];               //BNOデータ取得、格納に用いる
unsigned long time_data = 0;  //離床判定タイマー(燃焼終了検知)

//BNO設定
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// BME280


// SAM-M8Q


// ES920LR


// W25Q128



// setup()ではdelay()使用可
void setup() {
  // デバッグ出力
  Serial.begin(115200);
  //BNO055
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
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

      // デバッグ出力
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
    }

    // 100Hzで実行する処理


    // BNO055から100Hzで測定
    if (acceljudge == 0) {
      if (accelcount > 5) {  //5回連続で閾値を超えたときの処理
        acceljudge = 1;      //離床判定・フェーズ移行
        accelcount = 0;      //開放判定に向けた初期化
      }
      if (mediumBNO(event) > 46) {
        accelcount++;
      } else {
        accelcount = 0;  //初期化
      }
    }
    if (acceljudge == 1) {
      if (accelcount > 5) {  //5回連続で閾値を下回ったときの処理
        acceljudge = 2;      //開放判定・フェーズ移行
      }
      if (mediumBNO(event) < 10) {
        accelcount++;
      } else {
        accelcount = 0;  //初期化
      }
    }

    // BME280から100Hzで測定
    // ～測定処理～
    // data_bme_pressure_hPa = ~;
    // data_bme_temperature_degC = ~;
    // data_bme_altitude_m = ~;
  }

  // 常に実行する処理
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t accelerometerData;
  //CHEACK・NOTOPEN・READYコマンド受信
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // 改行までの入力を読み取る
    Serial.println(input);
    if (input.substring(0, input.length() - 1) == "CHECK") {  //CHECKコマンド
      phase_now = 0;                                          // "CHECK"が入力された場合、phaseに0を代入
      Serial.println("CHECK_transitioned");
    } else if (input.substring(0, input.length() - 1) == "NOTOPEN") {  //開放禁止コマンド
      Openjudge = 0;                                                   // "NOTOPEN"が入力された場合、Openjudgeに0(開放禁止)を代入
      Serial.println("NOTOPEN_Openjudge=0");
    } else if (input.substring(0, input.length() - 1) == "OPEN") {  //開放禁止取り消しコマンド
      Openjudge = 1;                                                // "OPEN"が入力された場合、Openjudgeに1を代入
      Serial.println("OPEN_Openjudge=1");
    }
  }
  //READYコマンド(CHECKphaseであることが条件)
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // 改行までの入力を読み取る
    if (phase_now == 0) {
      if (input.substring(0, input.length() - 1) == "READY") {  //READYコマンド
        phase_now = 1;                                          // "READY"が入力された場合、phase_nowに１を代入する
        Serial.println("READY_transitioned");
      }
    }
  }
  //離床判定
  if (phase_now == 1 && (altitudejudge == 1 || acceljudge == 1)) {  //READYフェーズに移行かつ、離床判定後(加速度or気圧で判定)である場合
    //高度による離床判定
    //10Hz処理に記述

    //加速度による離床判定
    //10Hz処理に記述


    //高度・加速度による離床判定
    if (acceljudge == 1) {                    //加速度による離床判定
      phase_now = 2;                          //フェーズ移行
      Serial.println("FLIGHT_transitioned");  //シリアルモニタに状態移行を表示
      time_data = millis();                   //離床判定タイマー
    } else if (/*高度による離床判定条件*/) {  //高度による離床判定
      phase_now = 2;                          //フェーズ移行
      Serial.println("FLIGHT_transitioned");  //シリアルモニタに状態移行を表示
      time_data = millis();                   //離床判定タイマー
    }
  }

  //開放判定
  if (phase_now == 2) {
    //高度による開放判定
    //10Hz処理に記述する

    //加速度による開放判定
    //10Hz処理に記述

    //高度・加速度による開放判定
    if (((acceljudge == 2) || (time_data > 12000 /*とりあえず燃焼時間12秒設定*/)) && ((altitudejudge == 2) && (time_data > 12000 /*とりあえず燃焼時間12秒設定*/)) && Openjudge == 1) {  //高度と加速度の閾値超過連続回数がともに5回以上かつ開放禁止コマンドが入力されていないとき
      phase_now = 3;                                                                                                                                                                    //フェーズ移行
      Serial.println("OPENED_transitioned");
    }
  }

  // SAM-M8QのUARTを常に読み出し
  // ～測定処理～
  // data_gnss_latitude_udeg = ~;
  // data_gnss_longitude_udeg = ~;
}

//BNO055の100Hzで測定する際に用いる関数
//BNOからセンサ値が0でない値のみを返す関数
double getBNO(sensors_event_t* event) {
  double x = -1000000, y = -1000000, z = -1000000;
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    z = event->acceleration.z;
    while (x == 0) {
      z = event->acceleration.z;
    }
  } else {
    Serial.print("Unk:");
  }
  return z;
}

//BNOデータ値の中央値計算
double mediumBNO(sensors_event_t* event) {
  double tmp = 0, medium = 0;
  for (int i = 0; i < 10; i++) {
    BNO[i] = getBNO(event);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = 1; j < 9; j++) {
      if (BNO[j - 1] < BNO[j]) {
        tmp = BNO[j - 1];
        BNO[j - 1] = BNO[j];
        BNO[j] = tmp;
      }
    }
  }
  medium = BNO[4];
  return medium;
}
