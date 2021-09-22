/*
  Buoy Test 7a
  Requests sonde data from Arduino b. Collects other data including date and time from GPS,
  battery voltage measured from A0, and latitude and longitude. Data sent to Hologram dashboard using Sparkfun's LTE Cat M1/NB-IoT Shield.
  Data and timestamp also written to file on SD card using Adafruit's micro-SD breakout board.
  Board sleeps between data collection/transmission periods, wakes Arduino b as necessary.
  Compatible with Buoy_Test_7b or Buoy_Test_6b.
  SparkFun RedBoard Artemis - Sparkfun Apollo3 board manager v1.2.1 (warning: does not work with v2.1.1)
  07/19/2021 - Owen Li
*/

#include <SparkFun_LTE_Shield_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SPI.h>                                  // For SD card
#include <SD.h>
#include <Wire.h>                                 // For I2C
#include "RTC.h"                                  // RTC library included with the Aruino_Apollo3 core

// Objects
LTE_Shield     myLTE;                             // Create a LTE_Shield object to use throughout the sketch
SFE_UBLOX_GNSS myGNSS;                            // GPS
File           myFile;                            // File for writing to on SD card
APM3_RTC       myRTC;

// LTE
const char HOLOGRAM_DEVICE_KEY[] = "()<sx[}c";    // Unique to SIM card, find on Hologram dashboard
const char HOLOGRAM_URL[] = "cloudsocket.hologram.io";
const unsigned int HOLOGRAM_PORT = 9999;
const int RETRIES = 5;
const unsigned int INTERVAL = 15;                  // Interval between waking, max 60 [minutes]

// SD Card
const char FILENAME[] = "Buoy2.txt";

// Pins
#define SARA_POWER_PIN 5                          // For turning LTE shield on/off
#define CHIP_SELECT 10                            // To SD card board
#define BATT_VOLTAGE_PIN A0
#define BOARD_B_PIN 6                             // For waking Arduino b
#define DEVICE_STATUS_CHAR_LENGTH 42
#define MAX_OPERATORS 10

// Variable Initializations
#define SONDE_DATA_LENGTH 90                      // Must match value in sketch for Arduino b
#define TIMESTAMP_LENGTH 11
unsigned long targetTime = 0;
bool bypassSD = false;
String currentOperator = "";
String currentApn = "";
IPAddress ip(0, 0, 0, 0);
IPAddress check(0, 0, 0, 0);
const String APN = "hologram";


void setup() {

  Serial.begin(9600);                   // Serial monitor for troubleshooting and debugging only
  Serial.println(F("\nBuoy Test 8a"));

  // I2C
  Wire.begin();                         // Start the I2C Bus as Master

  // SD Card
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println(F("SD init failed"));
    bypassSD = true;
  } else {
    Serial.println(F("SD init success"));
  }

  // LTE
  if ( myLTE.begin(Serial1, 115200) ) {    // Using hardware serial (pins 0 and 1). Make sure serial switch on shield is set to "HARD"
    Serial.println(F("LTE connected"));
  }
  pinMode(BATT_VOLTAGE_PIN, INPUT);

  // GPS
  myGNSS.begin();                       // Connect to the u-blox module using Wire port
  myGNSS.setI2COutput(COM_TYPE_UBX);    // Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.powerSaveMode();               // Turn on power saving (defaults to true)

  // RTC
  myRTC.setTime(0, 0, 0, 0, 1, 1, 20);  // 00:00:00.000, Jan 1st, 2020 (hour, minutes, seconds, hundredths, day, month, year)
  myRTC.setAlarmMode(5);                // Set the RTC alarm to automatically trigger on hours rollover
  myRTC.attachInterrupt();              // Attach RTC alarm interrupt

}

void loop() {

  // Board wakes up here

  // Wake up
  Serial.print(F("Alarm interrupt: "));
  printRTCDateTime();
  wakeBoardB();

  int tries = 0;
  // Timestamp for SD from GPS
  char timestamp[12] = "\0";
  snprintf(timestamp, 12, "%02d-%02d %02d:%02d", myGNSS.getMonth(), myGNSS.getDay(), myGNSS.getHour(), myGNSS.getMinute());
  while (myGNSS.getTimeValid() == false) {  // Retry until valid time received
    myGNSS.getMonth();    // Discard old values
    myGNSS.getDay();
    myGNSS.getHour(); //NOTE: The hour returned is in GPS time which is based off of UTC and so will be 5(-ish) hours ahead of central time (depending on daylight savings).
    myGNSS.getMinute();
    timestamp[0] = '\0';
    snprintf(timestamp, 12, "%02d-%02d %02d:%02d", myGNSS.getMonth(), myGNSS.getDay(), myGNSS.getHour(), myGNSS.getMinute());
    Serial.println(timestamp);
    tries++;
    if (tries >= RETRIES * 5) {
      tries = 0;
      myRTC.setTime(0, 0, 0, 0, 1, 1, 20);      // (hour, minutes, seconds, hundredths, day, month, year)
      Serial.print(F("Resetting RTC to "));
      printRTCDateTime();

      // Set the RTC's alarm
      myRTC.setAlarm(0, INTERVAL, 0, 0, 1, 1);

      // Enter deep sleep and await RTC alarm interrupt
      Serial.println(F("Sleeping"));
      goToSleep();
    }
    delay(1000);
  }

  // Battery voltage
  uint16_t battVoltage = analogRead(BATT_VOLTAGE_PIN) * 5000 / 1023;
  char charBattVoltage[5] = "\0";
  sprintf(charBattVoltage, "%03d", battVoltage);

  // GPS position
  long latitude = myGNSS.getLatitude();
  long longitude = myGNSS.getLongitude();
  char charLatitude[8] = "\0";
  char charLongitude[8] = "\0";
  snprintf(charLatitude, 8, "%0+ld", latitude);
  snprintf(charLongitude, 8, "%0+ld", longitude);

  // Assemble device health string
  char deviceStatus[DEVICE_STATUS_CHAR_LENGTH + 1] = "\0";          // 42 total chars below + 1 null
  strcat(deviceStatus, "&field1=");      // 8
  strcat(deviceStatus, charBattVoltage); // 3
  strcat(deviceStatus, "&field2=");      // 8
  strcat(deviceStatus, charLatitude);    // 7
  strcat(deviceStatus, "&field3=");      // 8
  strcat(deviceStatus, charLongitude);   // 7

  // Request sonde data from Arduino b
  delay(5000);                                      // delay guarantees Arduino b will collect new data from sonde before data request
  //Serial.print(F("Requesting sonde data: "));
  char sondeData[SONDE_DATA_LENGTH] = "\0";
  for (int i = 0; i < (SONDE_DATA_LENGTH / 32 + 1); i++) { // request packets
    if (i == SONDE_DATA_LENGTH / 32) {
      Wire.requestFrom(9, SONDE_DATA_LENGTH % 32 - 1); // request remaining bytes from slave device 9
    } else {
      Wire.requestFrom(9, 32);                      // request 32 (max buffer size) bytes from slave device 9
    }
    while (Wire.available()) {                      // slave may send less than requested
      char c[2];
      c[0] = Wire.read();                           // receive a byte as character
      c[1] = '\0';
      strcat(sondeData, c);                         // save byte
    }
  }
  Serial.println(sondeData);

  // Save data to SD card
  char fileLine[TIMESTAMP_LENGTH + DEVICE_STATUS_CHAR_LENGTH + SONDE_DATA_LENGTH + 3] = "\0";
  strcat(fileLine, timestamp);
  strcat(fileLine, " ");
  strcat(fileLine, deviceStatus);
  strcat(fileLine, " ");
  strcat(fileLine, sondeData);
  if (!bypassSD) {
    myFile = SD.open(FILENAME, FILE_WRITE);
    if (myFile) {
      myFile.println(fileLine);
      myFile.close();
      Serial.print(F("Writing SD "));
      Serial.println(fileLine);
      delay(1000);
    } else {
      // if the file didn't open, print an error:
      Serial.println(F("Error opening file"));
    }
  }

  myLTE.getAPN(&currentApn, &ip);

  if (myLTE.getOperator(&currentOperator) == 0) { //check for lte connection
    if (ip != check) {                            //check hologram connection
      // Send sonde data over LTE
      for (int tries = 0; tries < RETRIES; tries++) {
        if (sendHologramMessage(sondeData, "SONDEDATA")) {
          break;
        }
      } // for (int tries...)

      // Send device status over LTE
      for (int tries = 0; tries < RETRIES; tries++) {
        if (sendHologramMessage(deviceStatus, "DEVSTATUS")) {
          break;
        }
      }
    }// for (int tries...)
  } else {
    reconnectLTE();
  }

  // Prepare for sleep
  // Sync RTC to GPS time
  myGNSS.getMinute();                     // Discard stale values
  myGNSS.getSecond();                     // Discard stale values
  int GPSMinute = myGNSS.getMinute();
  int GPSSecond = myGNSS.getSecond();
  while (myGNSS.getTimeValid() == false) { // Retry until valid time received
    delay(500);
  }

  GPSMinute = GPSMinute % INTERVAL;                         // Take remainder to prevent hour/day rollover issues
  myRTC.setTime(0, GPSMinute, GPSSecond, 0, 1, 1, 20);      // (hour, minutes, seconds, hundredths, day, month, year)
  Serial.print(F("Resetting RTC to "));
  printRTCDateTime();

  // Set the RTC's alarm
  myRTC.setAlarm(0, INTERVAL, 0, 0, 1, 1);                  // 00:INTERVAL:00.000, Jan 1st, 2020 (hour, minutes, seconds, hundredths, day, month). Note: No year alarm register

  // Enter deep sleep and await RTC alarm interrupt
  Serial.println(F("Sleeping"));
  goToSleep();

} // void loop()



bool sendHologramMessage(const char *message, const char *topic) {

  int socket;// = -1;

  // New lines are not handled well

  // Construct a JSON-encoded Hologram message string:
  char hologramMessage[strlen(message) + 40] = "\0";
  strcat(hologramMessage, "{\"k\":\"");
  strcat(hologramMessage, HOLOGRAM_DEVICE_KEY);
  strcat(hologramMessage, "\",\"d\":\"");
  strcat(hologramMessage, message);
  strcat(hologramMessage, "\",\"t\":\"");
  strcat(hologramMessage, topic);
  strcat(hologramMessage, "\"}");

  // Create a socket
  socket = myLTE.socketOpen(LTE_SHIELD_TCP);
  // On success, socketOpen will return a value between 0-5. On fail -1.
  if (socket >= 0) {
    // Use the socket to connect to the Hologram server
    Serial.print(F("Connecting to socket "));
    Serial.println(socket);
    if (myLTE.socketConnect(socket, HOLOGRAM_URL, HOLOGRAM_PORT) == LTE_SHIELD_SUCCESS) {
      // Send our message to the server:
      Serial.print(F("Sending message "));
      Serial.println(hologramMessage);
      if (myLTE.socketWrite(socket, hologramMessage) == LTE_SHIELD_SUCCESS)
      {
        Serial.println(F("LTE success"));
        // On succesful write, close the socket.
        if (myLTE.socketClose(socket) == LTE_SHIELD_SUCCESS) {
          Serial.println(F("Socket closed"));
        }
        return true;
      } else {
        Serial.println(F("Failed to write"));
        return false;
      }
    } else {
      Serial.println(F("Failed to connect"));
      return false;
    }
  } else {
    Serial.println(F("Failed to create socket"));
    return false;

  }

}

void toggleLTEPower() {
  Serial.println(F("Toggling LTE power"));
  pinMode(SARA_POWER_PIN, OUTPUT);
  digitalWrite(SARA_POWER_PIN, LOW);
  delay(2000);
  pinMode(SARA_POWER_PIN, INPUT); // Return to high-impedance, rely on SARA module internal pull-up
  delay(2000);
}

void wakeBoardB() {
  pinMode(BOARD_B_PIN, OUTPUT);
  digitalWrite(BOARD_B_PIN, LOW);
  delay(2000);
  pinMode(BOARD_B_PIN, INPUT); // Return to high-impedance, rely on board B internal pull-up
}

void printRTCDateTime() {
  myRTC.getTime();
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
          myRTC.year, myRTC.month, myRTC.dayOfMonth,
          myRTC.hour, myRTC.minute, myRTC.seconds, myRTC.hundredths);
  Serial.println(dateTimeBuffer);
}


void reconnectLTE() {
  int opsAvailable = 0;
  struct operator_stats ops[MAX_OPERATORS];
  for (int i = 0; i < 1; i++) {
    Serial.println(F("Setting mobile-network operator"));
    if (myLTE.setNetwork(MNO_SW_DEFAULT)) {
      Serial.print(F("Set mobile network operator to "));
      Serial.println(F("Default"));
    } else {
      Serial.println(F("Error setting MNO"));
      break;
    }
    Serial.println(F("Setting APN..."));
    if (myLTE.setAPN(APN) == LTE_SHIELD_SUCCESS) {
      Serial.println(F("APN successfully set.\r\n"));
    } else {
      Serial.println(F("Error setting APN"));
      break;
    }
    Serial.println(F("Scanning for operators...this may take up to 3 minutes\r\n"));
    opsAvailable = myLTE.getOperators(ops, MAX_OPERATORS); // This will block for up to 3 minutes
    if (opsAvailable > 0) {
      Serial.println("Found " + String(opsAvailable) + " operators:");
      printOperators(ops, opsAvailable);
      for (int i = 0; i < opsAvailable; i++) {
        if (ops[i].stat == 1) {
          Serial.println("Connecting to option " + String(i + 1));
          if (myLTE.registerOperator(ops[i]) == LTE_SHIELD_SUCCESS) {
            Serial.println("Network " + ops[i].longOp + " registered\r\n");
            break;
          } else {
            Serial.println(F("Error connecting to operator"));
            break;
          }
        }
      }
    } else {
      Serial.println(F("Did not find an operator"));
    }
  }
}


void printOperators(struct operator_stats * ops, int operatorsAvailable) {
  for (int i = 0; i < operatorsAvailable; i++) {
    Serial.print(String(i + 1) + ": ");
    Serial.print(ops[i].longOp + " (" + String(ops[i].numOp) + ") - ");
    switch (ops[i].stat) {
      case 0:
        Serial.println(F("UNKNOWN"));
        break;
      case 1:
        Serial.println(F("AVAILABLE"));
        break;
      case 2:
        Serial.println(F("CURRENT"));
        break;
      case 3:
        Serial.println(F("FORBIDDEN"));
        break;
    }
  }
  Serial.println();
}


void goToSleep() {
  // Disable UART
  Serial.end();

  // Disable ADC
  power_adc_disable();

  // Force the peripherals off
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_UART1);

  // Disable all pads
  for (int x = 0 ; x < 50 ; x++)
    am_hal_gpio_pinconfig(x, g_AM_HAL_GPIO_DISABLE);

  //Power down Flash, SRAM, cache
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE); // Turn off CACHE
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_512K); // Turn off everything but lower 512k
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_64K_DTCM); // Turn off everything but lower 64k
  //am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_ALL); //Turn off all memory (doesn't recover)

  // Keep the 32kHz clock running for RTC
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);

  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); // Sleep forever

  // And we're back!
  wakeUp();
}

// Power up gracefully
void wakeUp() {
  // Power up SRAM, turn on entire Flash
  am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_MAX);

  // Go back to using the main clock
  am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
  am_hal_stimer_config(AM_HAL_STIMER_HFRC_3MHZ);

  // Enable ADC
  ap3_adc_setup();

  // Enable Serial
  Serial.begin(115200);

  // Reenable peripherals
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM0);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM1);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM2);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM3);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM4);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_IOM5);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_ADC);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_UART0);
  am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_UART1);

  // Reinitialize everything
  setup();
  //  Serial.println(F("\nBuoy Test 8a"));
  //
  //  // I2C
  //  Wire.begin();                         // Start the I2C Bus as Master
  //
  //  // SD Card
  //  if (!SD.begin(CHIP_SELECT)) {
  //    Serial.println(F("SD init failed"));
  //    bypassSD = true;
  //  } else {
  //    Serial.println(F("SD init success"));
  //  }
  //
  //  // GPS
  //  myGNSS.begin();                       // Connect to the u-blox module using Wire port
  //  myGNSS.setI2COutput(COM_TYPE_UBX);    // Set the I2C port to output UBX only (turn off NMEA noise)
  //  myGNSS.powerSaveMode();               // Turn on power saving (defaults to true)
  //  pinMode(BATT_VOLTAGE_PIN, INPUT);
  //  // RTC
  //  myRTC.setTime(0, 0, 0, 0, 1, 1, 20);  // 00:00:00.000, Jan 1st, 2020 (hour, minutes, seconds, hundredths, day, month, year)
  //  myRTC.setAlarmMode(5);                // Set the RTC alarm to automatically trigger on hours rollover
  //  myRTC.attachInterrupt();
}

// Interrupt handler for the RTC
extern "C" void am_rtc_isr(void) {
  // Clear the RTC alarm interrupt
  am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
}
