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
  #include "MedianFilterLib.h"
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
  
  double z = 0;
  float medianaccel = 0;                 //加速度中央値計算
  float medianaltitude = 0;              //高度中央値計算
  int phase_now = 0;                     //現在のフェーズを区別(0=CHEAK,1=READY,2=FLIGHT,3=OPENED)
  int Openjudge = 1;                     //開放禁止コマンド用状態表示用(0=NOT_OPEN,1=OPEN)
  unsigned long time_data = 0;           //離床判定タイマー(燃焼終了検知)
  bool altitudeJudge = 0;                //高度による離床・開放のフェーズ区別変数(0=離床, 1=開放) //ここでフェーズ移行用の変数を2つ用意したのは、開放動作の終了を指示するためである
  bool accelJudge = 0;                   //加速度による離床・開放のフェーズ区別変数(0=離床, 1=開放)
  const int Count = 10;                  //1度の中央値計算で要するデータ数
  float altitude[10];                    //高度データを格納する配列
  float accel[10];                       //加速度データを格納する配列
  int altitudeIndex = 0;                 //高度データ配列内の現在のインデックス数
  int accelIndex = 0;                    //加速度データ配列内の現在のインデックス数
  int consecutivealtitudeCount = 0;      //高度による離床・開放判定の閾値超過連続回数の記録変数
  int cosecutiveaccelCount = 0;          //加速度による離床・開放判定の閾値超過連続回数の記録変数
  float altitudeReleseThreshold = 2.00;  //高度による離床判定の閾値
  float accelReleseThreshold = 20;       //加速度による離床判定の閾値
  float altitudeOpenThreshold = 0.50;    //高度による開放判定の閾値
  float accelOpenThreshold = 5.00;       //加速度による開放判定の閾値
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
    // ～測定処理～
    // data_bno_accel_x_mss = ~;
    // data_bno_accel_y_mss = ~;
    // data_bno_accel_z_mss = ~;


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
    if (phase_now == 1 && altitudeJudge == 0 && accelJudge == 0) {  //READYフェーズに移行かつ、離床フェーズである場合
      //高度による離床判定
      //高度のデータを配列に格納
      altitude[altitudeIndex] = altitudeValue;  //BMEによって取得した値を代入する
      altitudeIndex = (altitudeIndex + 1) % Count;
  
      //高度データが10回読み取られたら中央値を計算
      if (altitudeIndex == 0) {
        medianaltitude = MedianFilter(altitude, Count);
        altitudeIndex = 0;
      }
  
      //高度の中央値が閾値を超えた場合、連続回数に+1
      if (medianaltitude > altitudeReleseThreshold) {
        consecutivealtitudeCount++;
      } else {
        consecutivealtitudeCount = 0;  //閾値を下回ったら連続回数を0へリセット
      }
  
      //加速度による離床判定
      //加速度のデータを配列に格納
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      accel[accelIndex] = bnoEvent(&accelerometerData);
      accelIndex = (accelIndex + 1) % Count;
  
      //加速度データが10回読み取られたら中央値を計算
      if (altitudeIndex == 0) {
        medianaccel = MedianFilter(accel, Count);
        accelIndex = 0;
      }
  
      //加速度の中央値が閾値を超えた場合、連続回数に+1
      if (medianaccel > accelReleaseThreshold) {
        cosecutiveaccelCount++;
      } else {
        cosecutiveaccelCount = 0;  //閾値を下回ったら連続回数を0へリセット
      }
  
      //高度・加速度による離床判定
      if (consecutivealtitudeCount == 5) {  //高度と加速度の閾値超過連続回数がどちらか5回以上になった場合
        altitudeJudge = 1;                  //高度によって開放フェーズへ移行 //"Judge = 0"を実行条件とするif文の中で"Judge = 1"としたらどうなる？
        consecutivealtitudeCount = 0;       //高度の連続閾値超過回数をリセット
        phase_now = 2;
        Serial.println("FLIGHT_transitioned");  //シリアルモニタに状態移行を表示
        accelOpenThreshold = -10;               //開放判定に用いる加速度の閾値変(上回る閾値から下回る閾値へ)
        time_data = millis();                   //離床判定タイマー
        //後々、CAN通信で”離床判定”とダウンリンクするように設定
      } else if (cosecutiveaccelCount == 5) {
        accelJudge = 1;            //加速度によって開放フェーズ平行
        cosecutiveaccelCount = 0;  //加速度の連続閾値超過回数をリセット
        phase_now = 2;
        Serial.println("FLIGHT_transitioned");  //シリアルモニタに状態移行を表示
        accelOpenThreshold = -10;               //開放判定に用いる加速度の閾値変(上回る閾値から下回る閾値へ)
        time_data = millis();                   //離床判定タイマー
        //後々、CAN通信で”離床判定”とダウンリンクするように設定
      }
    }
  
    //開放判定
    if (phase_now == 2) {
      //高度による開放判定
      //高度のデータを配列に格納
      altitude[altitudeIndex] = altitudeValue;  //BMEによって取得した値を代入する
      altitudeIndex = (altitudeIndex + 1) % Count;
  
      //高度データが10回読み取られたら中央値を計算
      if (altitudeIndex == 0) {
        medianaltitude = MedianFilter<float>(altitude, Count);
        altitudeIndex = 0;
      }
  
      //高度の中央値が閾値を超えた場合、連続回数に+1
      if (medianaltitude > altitudeOpenThreshold) {
        consecutivealtitudeCount++;
      } else {
        consecutivealtitudeCount = 0;  //閾値を下回ったら連続回数を0へリセット
      }
  
      //加速度による開放判定
      //加速度のデータを配列に格納
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      accel[accelIndex] = bnoEvent(&accelerometerData);
      accelIndex = (accelIndex + 1) % Count;
  
      //加速度データが10回読み取られたら中央値を計算
      if (altitudeIndex == 0) {
        medianaccel = MedianFilter(accel, Count);
        accelIndex = 0;
      }
  
      //加速度の中央値が閾値を下回った場合、連続回数に+1
      if (medianaccel < accelOpenThreshold) {
        cosecutiveaccelCount++;
      } else {
        cosecutiveaccelCount = 0;  //閾値を下回ったら連続回数を0へリセット
      }
  
      //高度・加速度による開放判定
      if (((consecutivealtitudeCount == 5) || (time_data > 5000 /*とりあえず燃焼時間5秒設定*/)) && ((cosecutiveaccelCount == 5) && (time_data > 5000 /*とりあえず燃焼時間5秒設定*/)) && Openjudge == 1) {  //高度と加速度の閾値超過連続回数がともに5回以上かつ開放禁止コマンドが入力されていないとき
        phase_now = 3;
        Serial.println("OPENED_transitioned");
        altitudeJudge = 0;  //高度によって開放フェーズを終了
        accelJudge = 1;     //加速度によって開放フェーズを終了
        consecutivealtitudeCount = 0;
        cosecutiveaccelCount = 0;
        digitalWrite(2, HIGH);  //MosFetに電圧を印加
        delay(HeatingTime);     //ニクロム線を規定時間加熱(8s)
        digitalWrite(2, LOW);   //MosFetへの電圧印加を停止
        //後々、CAN通信で”開放判定”とダウンリンクするように設定
      }
    }

  // SAM-M8QのUARTを常に読み出し
  // ～測定処理～
  // data_gnss_latitude_udeg = ~;
  // data_gnss_longitude_udeg = ~;
}
//BNO055データ取得用関数
double bnoEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000;  // 問題を見つけやすいダミーの値

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    if (event->acceleration.z != 0) {//データ取得エラーを排除
      z = event->acceleration.z;
    }
  }
  return z;
}
