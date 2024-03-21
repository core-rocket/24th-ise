#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

//フォーマット関連
// Since SdFat doesn't fully support FAT12 such as format a new flash
// We will use Elm Cham's fatfs f_mkfs() to format
#include "ff.h"
#include "diskio.h"

// for flashTransport definition
#define EXTERNAL_FLASH_USE_CS 17
#define EXTERNAL_FLASH_USE_SPI SPI
Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);


Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

// up to 11 characters
#define DISK_LABEL "EXT FLASH"

typedef enum mode {
  SLEEP,
  CLEARING,
  ACTIVE,
} MODE;

MODE mode;

void format_fat12(void) {
// Working buffer for f_mkfs.
#ifdef __AVR__
  uint8_t workbuf[512];
#else
  uint8_t workbuf[4096];
#endif

  // Elm Cham's fatfs objects
  FATFS elmchamFatfs;

  // Make filesystem.
  FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
    Serial.print(F("Error, f_mkfs failed with error code: "));
    Serial.println(r, DEC);
    while (1) yield();
  }

  // mount to set disk label
  r = f_mount(&elmchamFatfs, "0:", 1);
  if (r != FR_OK) {
    Serial.print(F("Error, f_mount failed with error code: "));
    Serial.println(r, DEC);
    while (1) yield();
  }

  // Setting labelFile
  Serial.println(F("Setting disk label to: " DISK_LABEL));
  r = f_setlabel(DISK_LABEL);
  if (r != FR_OK) {
    Serial.print(F("Error, f_setlabel failed with error code: "));
    Serial.println(r, DEC);
    while (1) yield();
  }

  // unmount
  f_unmount("0:");

  // sync to make sure all data is written to flash
  flash.syncBlocks();

  Serial.println(F("Formatted flash!"));
}

void check_fat12(void) {
  // Check new filesystem
  if (!fatfs.begin(&flash)) {
    Serial.println(F("Error, failed to mount newly formatted filesystem!"));
    while (1) delay(1);
  }
}
//////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  flash.begin();
  fatfs.begin(&flash);
}

void loop() {
  int reading = random(0, 100);
  File32 dataFile;

  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');  // 受信した文字列を読み取ります
    inputString.trim();
    // 前後の空白文字を削除します
    if (inputString == "A") {
      mode = ACTIVE;
    } else if (inputString == "S") {
      mode = SLEEP;
    } else if (inputString == "C") {
      mode = CLEARING;
    }
  }

  switch (mode) {
    case CLEARING:
      //フォーマットリセット
      // Call fatfs begin and passed flash object to initialize file system
      Serial.println(F("Creating and formatting FAT filesystem (this takes ~60 seconds)..."));
      format_fat12();
      check_fat12();

      // Done!
      Serial.println(F("Flash chip successfully formatted with new empty filesystem!"));

      mode = SLEEP;
      break;

    case ACTIVE:
      // Configuration for the datalogging file:
      #define FILE_NAME "data.csv"

      //ファイルオープン
      // Open the datalogging file for writing.  The FILE_WRITE mode will open
      // the file for appending, i.e. it will add new data to the end of the file.
      dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
      // Check that the file opened successfully and write a line to it.

      //データ記録
      dataFile.print("Sensor #1");
      dataFile.print(",");
      dataFile.print(reading);
      dataFile.println();
      // Finally close the file when done writing.  This is smart to do to make
      // sure all the data is written to the file.

      //ファイルクローズ
      dataFile.close();
      Serial.println("active");
      break;

      // Serial.println("Trying again in 5 seconds...");

      // Wait 60 seconds.
      // delay(5000L);

    case SLEEP:
      Serial.println("sleep");
      break;
  }
}

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C" {

  DSTATUS disk_status(BYTE pdrv) {
    (void)pdrv;
    return 0;
  }

  DSTATUS disk_initialize(BYTE pdrv) {
    (void)pdrv;
    return 0;
  }

  DRESULT disk_read(
    BYTE pdrv,    /* Physical drive nmuber to identify the drive */
    BYTE *buff,   /* Data buffer to store read data */
    DWORD sector, /* Start sector in LBA */
    UINT count    /* Number of sectors to read */
  ) {
    (void)pdrv;
    return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
  }

  DRESULT disk_write(
    BYTE pdrv,        /* Physical drive nmuber to identify the drive */
    const BYTE *buff, /* Data to be written */
    DWORD sector,     /* Start sector in LBA */
    UINT count        /* Number of sectors to write */
  ) {
    (void)pdrv;
    return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
  }

  DRESULT disk_ioctl(
    BYTE pdrv, /* Physical drive nmuber (0..) */
    BYTE cmd,  /* Control code */
    void *buff /* Buffer to send/receive control data */
  ) {
    (void)pdrv;

    switch (cmd) {
      case CTRL_SYNC:
        flash.syncBlocks();
        return RES_OK;

      case GET_SECTOR_COUNT:
        *((DWORD *)buff) = flash.size() / 512;
        return RES_OK;

      case GET_SECTOR_SIZE:
        *((WORD *)buff) = 512;
        return RES_OK;

      case GET_BLOCK_SIZE:
        *((DWORD *)buff) = 8;  // erase block size in units of sector size
        return RES_OK;

      default:
        return RES_PARERR;
    }
  }
}
