/*
   mpu.h
*/

#ifndef _MPU
#define _MPU

#include <Wire.h>

void mpu_setup();
void mpu_execute();
int16_t mpu_readint();
double mpu_temp();
void mpu_print(bool raw);
void mpu_calibrate();
bool mpu_detect(unsigned int quack_threshold);

#endif


/*
   logger.h
*/

#ifndef _LOGGER
#define _LOGGER

void log(const char *message);
void log2(const char *category, const char* message);

#endif


/*
   quack.h
*/

#ifndef _QUACK
#define _QUACK

void setup_quack();
void quack();

#endif

/**********
   main.c
 **********/

#include <Arduino.h>

// threshold to trigger a desk bump
#define QUACK_THRESHOLD 6000

// for testing set to 1
#define AUTO_QUACK 0

void setup() {
  Serial.begin(9600);
  log2("init", "serial started");

  setup_quack();
  
  while (AUTO_QUACK == 1) {
    quack();
    delay(800);
  }

  log2("mpu", "setting up..");
  mpu_setup();
  mpu_calibrate();
  log2("mpu", "complete");
}

void loop() {
  mpu_execute();

  if (mpu_detect(QUACK_THRESHOLD)) {
    quack();
    delay(3000);
    mpu_execute();
    mpu_execute();
    mpu_execute();
  } else {
    delay(100);
  }
}



/*
   mpu.c
*/

#include <stdio.h>
#include <Wire.h>

#define MPU_ADDR 0x68
#define PWR_MGMT 0x6B

// first of 14 registers
#define ACCEL_XOUT_H 0x3B
#define NUM_REGISTERS 14

// maximum difference we'll accept total acceleration in order to pass a calibration check
#define CHECK_ACCEL_MAX 400

// number of consecutive checks we must pass to be calibrated
#define CHECK_ACCEL_COUNT 5

// delay between accel checks
#define CHECK_ACCEL_DELAY 500

int16_t init_acX, init_acY, init_acZ, init_tmp, init_gyX, init_gyY, init_gyZ;
int16_t prev_acX, prev_acY, prev_acZ, prev_tmp, prev_gyX, prev_gyY, prev_gyZ;
int16_t diff_acX, diff_acY, diff_acZ, diff_tmp, diff_gyX, diff_gyY, diff_gyZ;
int16_t acX, acY, acZ, tmp, gyX, gyY, gyZ;

void mpu_setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT);
  Wire.write(0); // wakes up the MPU-6050
  Wire.endTransmission(true);

  delay(2000);
  mpu_execute();

  log2("mpu", "setup complete");
}

void mpu_execute() {
  // save previous values
  prev_acX = acX;
  prev_acY = acY;
  prev_acZ = acZ;
  prev_tmp = tmp;
  prev_gyX = gyX;
  prev_gyY = gyY;
  prev_gyZ = gyZ;

  // capture new values
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, NUM_REGISTERS, true);
  acX = mpu_readint(); // ACCEL_XOUT
  acY = mpu_readint(); // ACCEL_YOUT
  acZ = mpu_readint(); // ACCEL_ZOUT
  tmp = mpu_readint(); // TEMP_OUT
  gyX = mpu_readint(); // GYRO_XOUT
  gyY = mpu_readint(); // GYRO_YOUT
  gyZ = mpu_readint(); // GYRO_ZOUT

  // compute diff
  diff_acX = acX - prev_acX;
  diff_acY = acY - prev_acY;
  diff_acZ = acZ - prev_acZ;
  diff_tmp = tmp - prev_tmp;
  diff_gyX = gyX - prev_gyX;
  diff_gyY = gyY - prev_gyY;
  diff_gyZ = gyZ - prev_gyZ;
}

int16_t mpu_readint() {
  return Wire.read() << 8 | Wire.read();
}

double mpu_temp() {
  // equation for temperature in C
  return tmp / 340.00 + 36.53;
}

void mpu_print(bool raw) {
  char buffer[40];
  Serial.println("acX\tacY\tacZ\tgyX\tgyY\tgyZ\ttmp");

  if (raw) {
    sprintf(buffer, "%d\t%d\t%d\t%d\t%d\t%d\t%f", acX, acY, acZ, gyX, gyY, gyZ, mpu_temp());
  } else {
    sprintf(buffer, "%d\t%d\t%d\t%d\t%d\t%d\t%f", acX - init_acX, acY - init_acY, acZ - init_acZ, gyX - init_gyX, gyY - init_gyY, gyZ - init_gyZ, mpu_temp);
  }

  Serial.println(buffer);
}

void mpu_calibrate() {
  int total_diff;
  int consecutive_checks = 0;
  bool pass = false;
  char buffer[100];

  log2("mpu", "starting calibration");

  do {
    mpu_execute();

    total_diff = abs(diff_acX) + abs(diff_acY) + abs(diff_acZ);
    pass = total_diff < CHECK_ACCEL_MAX;

    if (pass)
      consecutive_checks++;
    else
      consecutive_checks = 0;

    sprintf(buffer, "calibration (%d,\t%d,\t%d)\ttotal (%d)\tpass (%d)\tchecks (%d)", diff_acX, diff_acY, diff_acZ, total_diff, pass, consecutive_checks);
    delay(CHECK_ACCEL_DELAY);
    log2("mpu", buffer);
  } while (consecutive_checks < CHECK_ACCEL_COUNT);

  // set initial values
  init_acX = acX;
  init_acY = acY;
  init_acZ = acZ;
  init_gyX = gyX;
  init_gyY = gyY;
  init_gyZ = gyZ;
  init_tmp = tmp;

  quack();
  log2("mpu", "calibrated!");
}

bool mpu_detect(unsigned int quack_threshold) {
  int total_diff;
  char buffer[50];

  total_diff = abs(diff_acX) + abs(diff_acY) + abs(diff_acZ);

  if (total_diff >= CHECK_ACCEL_MAX) {
    sprintf(buffer, "total_diff %d", total_diff);
    log2("mpu", buffer);
  }

  return total_diff >= quack_threshold;
}

/*
   quack.c
*/

// https://www.arduino.cc/en/Tutorial/SimpleAudioPlayer
#include <SPI.h>
#include <SD.h>
#include <Audio.h>

#define SAMPLE_BLOCK_SIZE 1024

void setup_quack() {
  Sd2Card card;
  SdVolume volume;
  SdFile root;

  if (!card.init(SPI_HALF_SPEED, 52)) {
    log2("SD", "card init failed!");
  } else {
    log2("SD", "card initialized!");
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        log2("SD", "SD1");
        break;
      case SD_CARD_TYPE_SD2:
        log2("SD", "SD2");
        break;
      case SD_CARD_TYPE_SDHC:
        log2("SD", "SDHC");
        break;
      default:
        log2("SD", "Unknown");
    }
  }

  if (!!volume.init(card)) {
    log2("SD", "volume init failed!");
  } else {
    log2("SD", "volume initialized!");
    Serial.print("Clusters:          ");
    Serial.println(volume.clusterCount());
    Serial.print("Blocks x Cluster:  ");
    Serial.println(volume.blocksPerCluster());

    Serial.print("Total Blocks:      ");
    Serial.println(volume.blocksPerCluster() * volume.clusterCount());
    Serial.println();

    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    Serial.print("Volume type is:    FAT");
    Serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    Serial.print("Volume size (Kb):  ");
    Serial.println(volumesize);
    Serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

    root.openRoot(volume);
    root.ls(LS_R | LS_DATE | LS_SIZE);
  }

  if (!SD.begin(52)) {
    log2("SD", "failed!");
  } else {
    log2("SD", "set up!");
  }

  Audio.begin(88200, 100);
}

void quack() {
  log("quack!");

  int count = 0;
  File myFile = SD.open("hayes.wav");
  if (!myFile) {
    log2("SD", "error opening hayes.wav");
    while (true);
  }

  short buffer[SAMPLE_BLOCK_SIZE];

  // until the file is not finished
  while (myFile.available()) {
    // read from the file into buffer
    myFile.read(buffer, sizeof(buffer));

    // Prepare samples
    int volume = 1024;
    Audio.prepare(buffer, SAMPLE_BLOCK_SIZE, volume);
    // Feed samples to audio
    Audio.write(buffer, SAMPLE_BLOCK_SIZE);

    // Every 100 block print a '.'
    count++;
    if (count == 100) {
      Serial.print(".");
      count = 0;
    }
  }
  myFile.close();
}



/*
   logger.c
*/

#include <stdio.h>

#define LOG 1
#define MAX_LENGTH 200

void log(const char *message) {
  Serial.println(message);
}

void log2(const char *category, const char* message) {
  char buffer[MAX_LENGTH];

  if (LOG == 1) {
    sprintf(buffer, "%s: %s", category, message);
  }

  Serial.println(buffer);
}
