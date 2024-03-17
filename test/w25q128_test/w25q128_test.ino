#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#ifndef FLASH_CONFIG_H_
#define FLASH_CONFIG_H_

#define EXTERNAL_FLASH_USE_CS 22
#define EXTERNAL_FLASH_USE_SPI SPI

Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

#endif

Adafruit_SPIFlash flash(&flashTransport);

FatVolume fatfs;  // file system object from SdFat

#define FILE_NAME "data.csv"  //ファイル名を指定

void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println("Adafruit SPI Flash FatFs Simple Datalogging Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while (1) {
      delay(1);
    }
  }
  Serial.print("Flash chip JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println(
      "Was the flash chip formatted with the fatfs_format example?");
    while (1) {
      delay(1);
    }
  }

  Serial.println("Mounted filesystem!");
  Serial.println("Logging data every 60 seconds...");
}

void loop() {
  // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.
  File32 dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
    // Take a new data reading from a sensor, etc.  For this example just
    // Write a line to the file.  You can use all the same print functions
    // as if you're writing to the serial monitor.  For example to write
    // two CSV (commas separated) values:
    dataFile.print("Sensor #1");
    dataFile.print(",");
    dataFile.print(reading, DEC);  //必要なデータの代入
    dataFile.println();
    dataFile.close();
    Serial.println("Wrote new measurement to data file!");
  } else {
    Serial.println("Failed to open data file for writing!");
  }

  Serial.println("Trying again in 60 seconds...");

  // Wait 60 seconds.
  delay(60000L);  //時間を適切な時間に修正
}