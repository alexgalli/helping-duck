#define QUACK_THRESHOLD_TOP 10000
#define NON_QUACK_CHANGE_TOP 40
#define AUTO_QUACK_TOP 0
#define VOLUME_TOP 1024

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
#define QUACK_THRESHOLD QUACK_THRESHOLD_TOP

// for testing set to 1
#define AUTO_QUACK AUTO_QUACK_TOP

void setup() {
  Serial.begin(9600);
  log2("init", "serial started");

  setup_quack();

  while (AUTO_QUACK == 1) {
    quack(false);
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
    quack(false);
    delay(100);
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

  quack(true);
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
#include "Audio.h"

#define SAMPLE_BLOCK_SIZE 1024
#define NON_QUACK_CHANCE NON_QUACK_CHANGE_TOP
#define VOLUME VOLUME_TOP

int quack_file_count, other_file_count;

void setup_quack() {
  SdFile root;
  SdVolume volume;

  if (!SD.begin(52)) {
    log2("SD", "failed!");
  } else {
    log2("SD", "set up!");
    // dunno why this is needed, but it's magic
    root.openRoot(volume);
  }

  // count files
  File dir, entry;
  char logbuffer[20];
  dir = SD.open("quack");
  while (true) {
    entry = dir.openNextFile();
    if (! entry) break;
    quack_file_count++;
    entry.close();
  }
  dir.close();
  sprintf(logbuffer, "counted %d quack files", quack_file_count);
  log2("SD", logbuffer);
  
  dir = SD.open("other");
  while (true) {
    entry = dir.openNextFile();
    if (! entry) break;
    other_file_count++;
    entry.close();
  }
  dir.close();
  sprintf(logbuffer, "counted %d quack files", other_file_count);
  log2("SD", logbuffer);

  Audio.begin(44100, 100);
}

void quack(bool always_quack) {
  log("quack!");

  // first, select a sound type
  int file_index;
  File dir;
  char logbuffer[20];
  bool is_quack = always_quack || random(0, 100) > NON_QUACK_CHANCE;
  if (is_quack) {
    dir = SD.open("quack");
    if (always_quack) {
      file_index = 0;
    } else {
      file_index = random(0, quack_file_count);
    }
    sprintf(logbuffer, "selected quack %d", file_index);
    log2("quack", logbuffer);
  } else {
    dir = SD.open("other");
    file_index = random(0, other_file_count);
    sprintf(logbuffer, "selected non-quack %d", file_index);
    log2("quack", logbuffer);
  }

  // open our file
  File audioFile;
  int i = 0;
  while (true) {
    audioFile = dir.openNextFile();
    if (i == file_index) {
      break;
    }
    audioFile.close();
    i++;
  }
  dir.close();

  // play file
  if (!audioFile) {
    log2("quack", "error opening file");
    while (true);
  } else {
    log2("quack", audioFile.name());
  }

  short buffer[SAMPLE_BLOCK_SIZE];

  // until the file is not finished
  while (audioFile.available()) {
    // read from the file into buffer
    audioFile.read(buffer, sizeof(buffer));

    // Prepare samples
    Audio.prepare(buffer, SAMPLE_BLOCK_SIZE, VOLUME);
    // Feed samples to audio
    Audio.write(buffer, SAMPLE_BLOCK_SIZE);
  }
  audioFile.close();
  // fix stuttering by hacking Audio.cpp: https://forum.arduino.cc/index.php?topic=284345.0
  Audio.rst();
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
