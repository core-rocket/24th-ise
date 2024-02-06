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


// BME280


// SAM-M8Q


// ES920LR


// W25Q128



// setup()ではdelay()使用可
void setup() {
  // デバッグ出力
  Serial.begin(115200);
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

  // SAM-M8QのUARTを常に読み出し
  // ～測定処理～
  // data_gnss_latitude_udeg = ~;
  // data_gnss_longitude_udeg = ~;
}
