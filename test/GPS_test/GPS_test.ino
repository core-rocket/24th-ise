#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);  //シリアルモニターに表示させるシリアル通信
  Serial1.begin(9600);   //GPSと通信するシリアル通信
}

void loop() {                        // run over and over
  while (Serial1.available() > 0) {  //GPSと通信ができたとき
    char c = Serial1.read();         //UART通信で情報を読み取って変数cに代入
    //Serial.print(c);
    gps.encode(c);  //座標を数値に変換
    if (gps.location.isUpdated()) {
      Serial.print("LAT=");
      Serial.println(gps.location.lat(), 6);  //緯度
      Serial.print("LONG=");
      Serial.println(gps.location.lng(), 6);  //経度
      Serial.print("ALT=");
      Serial.println(gps.altitude.meters());  //高度
    }
  }
}