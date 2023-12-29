#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
int phase=0;
double median;
double z_value = 0; 
double zValues[5]; // 5つのz値を格納する配列
int zCount = 0; // 現在のz値の数

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop(void)
{
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t  accelerometerData;
  //bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //printEvent(&accelerometerData);
if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // 改行までの入力を読み取る
    Serial.println(input);
    if (input.substring(0, input.length() - 1) == "CHECK") {
      phase = 0; // "CHECK"が入力された場合、phaseに0を代入
      Serial.println("CHECK_PHASE");
    }
  }
if(phase==0){
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  z_value = printEvent(&accelerometerData, z_value);
  
  
  delay(100);
  if(median<15){
    //
  }else{
    Serial.println("FLIGHT_PHASE");
    phase=1;
    delay(1000);
  }
}else if(phase==1){
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  z_value = printEvent(&accelerometerData, z_value);

  delay(100);
  if(median<0){
    Serial.println("opened");
    phase=2;
    delay(1000);
  }else{
    //
  }
}else{
  Serial.println("OPENED");
  delay(3000);
}



}
/*
double printEvent(sensors_event_t* event, double& z) {
  double x = -1000000, y = -1000000; // dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }
  if(z!=0){
  Serial.print("Accl:");
  Serial.print("x= ");
  Serial.print(x);
  Serial.print(" y= ");
  Serial.print(y);
  Serial.print(" z= ");
  Serial.println(z);
  }else{}
  return z; // Return z value
}
*/
double printEvent(sensors_event_t* event, double& z) {
  double x = -1000000, y = -1000000; // 問題を見つけやすいダミーの値

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;

    // z値を配列に追加し、カウントを増やす
    if(z==0){

    }else{
    zValues[zCount] = z;
    zCount++;
    }
    // 5回z値を取得したら、中央値を計算する
    if (zCount == 3) {
      
      // 配列を昇順にソートする
      for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
          if (zValues[i] > zValues[j]) {
            double temp = zValues[i];
            zValues[i] = zValues[j];
            zValues[j] = temp;
          }
        }
      }
      // 中央値を計算する
      if (zCount % 2 == 0) {
        median = (zValues[zCount / 2 - 1] + zValues[zCount / 2]) / 2.0;
      } else {
        median = zValues[zCount / 2];
      }
      for(int k=0;k<3;k++){
        Serial.println(zValues[k]);
      }
      // 中央値をシリアルモニタに出力
      Serial.print("Median z value: ");
      Serial.println(median);
      
      // カウントをリセットする
      zCount = 0;
    }
  } else {
    Serial.print("Unk:");
  }
  return z; // z値を返す
}