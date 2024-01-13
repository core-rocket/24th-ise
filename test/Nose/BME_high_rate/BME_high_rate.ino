#include <Wire.h>
#include <Adafruit_BME280.h>


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

unsigned long delayTime;
const int clockFrequency = 400000;  //I2C bus speed

void setup() {
  Serial.begin(115200);
  unsigned status;
  Wire.setSDA(6);
  Wire.setSCL(7);
  Wire.setClock(clockFrequency);
  Wire.begin();
  status = bme.begin(0x76);
}


void loop() {
  printValues();
  delay(5);
}


void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
