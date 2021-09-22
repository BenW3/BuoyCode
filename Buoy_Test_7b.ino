/*
Buoy Test 7b
Collects and replies with data upon request from Arduino a. Data read from SDI-12 sensor.
Board sleeps between data collection periods, wakes up when INTERRUPTPIN pulled to GND.
Arduino Uno/SparkFun RedBoard
07/19/2021 - Owen Li
*/

#include <Wire.h>                         // Needed for SDI-12
#include <SDI12.h>                        // Sensor interface
#include <avr/sleep.h>                    // AVR library which contains the methods that controls the sleep modes

// Pins
#define SONDE_DATA_PIN 7                  // Data pin of the SDI-12 bus
#define INTERRUPT_PIN 2                   // Pin used to wake up the Arduino

// Variable Initializations
#define SONDE_DATA_LENGTH 90              // Must match value in sketch for Arduino b
char sondeData[SONDE_DATA_LENGTH] = "\0"; // 50 chars + 1 null
double field[8];
char charField[8][10];
byte packetNum = 0;
bool transmissionComplete = false;

// SDI-12
#define SONDE_ADDR 0                      // SDI device address
SDI12 mySDI12(SONDE_DATA_PIN);


//////////////////////////////////////////
////////////    void setup    ////////////
//////////////////////////////////////////
void setup() {
  
  // Serial monitor for troubleshooting and debugging only
  Serial.begin(9600);
  Serial.println(F("Buoy Test 7b"));

  // I2C
  Wire.begin(9); 
  Wire.onRequest(requestEvent);

  // SDI-12
  mySDI12.begin();
  delay(500);  // allow things to settle

  // Sleep
  pinMode(INTERRUPT_PIN,INPUT_PULLUP);     //Set pin D2 to input using the built-in pullup resistor
}


////////////////////////////////////////////////
////////////    SDI-12 Functions    ////////////
////////////////////////////////////////////////
// for converting SDI-12 addresses
char decToChar(byte i) {
  if (i < 10) return i + '0';
  if ((i >= 10) && (i < 36)) return i + 'a' - 10;
  if ((i >= 36) && (i <= 62))
    return i + 'A' - 36;
  else
    return i;
}
bool takeMeasurement(char i, String meas_type = "") {
  mySDI12.clearBuffer();
  String command = "";
  command += i;
  command += "M";
  command += meas_type;
  command += "!";  // SDI-12 measurement command format  [address]['M'][!]
  mySDI12.sendCommand(command);
  delay(100);

  // wait for acknowlegement with format [address][ttt (3 char, seconds)][number of
  // measurments available, 0-9]
  String sdiResponse = mySDI12.readStringUntil('\n');
  sdiResponse.trim();

  // find out how long we have to wait (in seconds).
  uint8_t wait = sdiResponse.substring(1, 4).toInt();
  //Serial.print("Wait: ");
  //Serial.println(wait);

  // Set up the number of results to expect
  int numResults = sdiResponse.substring(4).toInt();
  //Serial.print("Number of results: ");
  //Serial.println(numResults);

  unsigned long timerStart = millis();
  while ((millis() - timerStart) < (1000 * (wait + 1))) {
    if (mySDI12.available())  // sensor can interrupt us to let us know it is done early
    {
      mySDI12.clearBuffer();
      break;
    }
  }
  // Wait for anything else and clear it out
  delay(30);
  mySDI12.clearBuffer();

  if (numResults > 0) { return getResults(i, numResults); }

  return true;
}

bool getResults(char i, int resultsExpected) {
  uint8_t resultsReceived = 0;
  uint8_t cmd_number      = 0;
  while (resultsReceived < resultsExpected && cmd_number <= 9) {
    String command = "";
    command = "";
    command += i;
    command += "D";
    command += cmd_number;
    command += "!";  // SDI-12 command to get data [address][D][dataOption][!]
    mySDI12.sendCommand(command);

    uint32_t start = millis();
    while (mySDI12.available() < 3 && (millis() - start) < 1500) {}
    mySDI12.read();           // ignore the repeated SDI12 address
    char c = mySDI12.peek();  // check if there's a '+' and toss if so
    if (c == '+') { mySDI12.read(); }

    while (mySDI12.available()) {
      char c = mySDI12.peek();
      if (c == '-' || (c >= '0' && c <= '9') || c == '.') {
        field[resultsReceived] = mySDI12.parseFloat(SKIP_NONE);
        resultsReceived++;
      } else {
        // discard unneeded bytes such as '+'
        mySDI12.read();
      }
      delay(10);  // 1 character ~ 7.5ms
    }
    cmd_number++;
  }
  mySDI12.clearBuffer();
  return resultsReceived == resultsExpected;
}


///////////////////////////////////////////////
////////////     I2C Functions     ////////////
///////////////////////////////////////////////
void requestEvent() {
  Serial.println(F("Request received"));
  
  // Reply with message
  for (int i = packetNum*32; i<(packetNum+1)*32; i++){
    if (i<SONDE_DATA_LENGTH){
      Wire.write(sondeData[i]);
      Serial.print(sondeData[i]);
    } else {
      transmissionComplete = true;
    }
  }
  packetNum = (packetNum+1) % (SONDE_DATA_LENGTH/32 + 1);
  Serial.print("\n");
}


///////////////////////////////////////////////
////////////    Sleep Functions    ////////////
///////////////////////////////////////////////
void goToSleep(){
  Serial.println("Sleeping");
  delay(500);                           // Allow serial message to send
  sleep_enable();                       // Enabling sleep mode
  attachInterrupt(0, wakeUp, LOW);      // Attaching a interrupt to pin D2
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Setting the sleep mode, in our case full sleep
  sleep_cpu();                          // Activating sleep mode
}

void wakeUp(){
  Serial.println("Waking");             // Print message to serial monitor
  sleep_disable();                      // Disable sleep mode
  detachInterrupt(0);                   // Removes the interrupt from pin 2;
}


/////////////////////////////////////////
////////////    void loop    ////////////
/////////////////////////////////////////
void loop() {

  // Wakes up here

  if(transmissionComplete) {
    transmissionComplete = false;
    goToSleep();
  }

  // Collect sonde data
  String meas_type[] = {"", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
  char addr = decToChar(SONDE_ADDR);
  memset(field, 0, sizeof field);
  takeMeasurement(addr, meas_type[0]);     

  // Assemble message
  dtostrf(field[0],4,4,charField[0]);   // sprintf does not support floats
  dtostrf(field[1],4,4,charField[1]);
  dtostrf(field[2],4,1,charField[2]);
  dtostrf(field[3],1,0,charField[3]);
  dtostrf(field[4],1,0,charField[4]);
  dtostrf(field[5],1,0,charField[5]);
  dtostrf(field[6],1,0,charField[6]);
  dtostrf(field[7],1,0,charField[7]);
  sondeData[0] = '\0';
  strcat(sondeData, "&field1=");
  strcat(sondeData, charField[0]);
  strcat(sondeData, "&field2=");
  strcat(sondeData, charField[1]);
  strcat(sondeData, "&field3=");
  strcat(sondeData, charField[2]);
  strcat(sondeData, "&field4=");
  strcat(sondeData, charField[3]);
  strcat(sondeData, "&field5=");
  strcat(sondeData, charField[4]);
  strcat(sondeData, "&field6=");
  strcat(sondeData, charField[5]);
  strcat(sondeData, "&field7=");
  strcat(sondeData, charField[6]);
  strcat(sondeData, "&field8=");
  strcat(sondeData, charField[7]);

  Serial.println(sondeData);

  delay(5000L);
}
