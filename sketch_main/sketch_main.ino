#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AM2302-Sensor.h>
#include "ADebouncer.h"
#include <PID_v1.h>

// SCREEN
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     2 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensor
AM2302::AM2302_Sensor tempSensor{8};

// Keys
#define NUM_KEYS (4)
typedef enum {clearKey, plusKey, minusKey, infoKey} Keys;
const int keyPins[NUM_KEYS] = {4,6,5,7};
ADebouncer *keyDebouncers[NUM_KEYS];
bool keyStates[NUM_KEYS];
bool lastKeyStates[NUM_KEYS];
long keyMillis;
const long KEY_REPEAT_DELAY = 400;

// State
double temperature;
double humidity;
bool temperatureError;
long temperatureReadMillis;

double desiredTemperature = 20;
const long TEMP_READ_DELAY = 5000;

typedef enum {settingScreen, detailsScreen, graphsScreen, runningScreen} ScreenMode;
ScreenMode screenMode = detailsScreen;
long screenModeMillis;
const long SCREEN_MODE_SLEEP = 60000;

// Controller
int fanInPercentage = 50;
int fanOutPercentage = 75;
double peltierPercentage = 0;

// Fans
const int outFanPin = 9;
const int inFanPin = 10;
const int outFanMin = 168;
const int outFanMax = 190;
const int inFanMin = 145;
const int inFanMax = 175;
// PID
//Define Variables we'll be connecting to

//Specify the links and initial tuning parameters
double Kp=1, Ki=0.1, Kd=0.6;
PID myPID(&temperature, &peltierPercentage, &desiredTemperature, Kp, Ki, Kd, DIRECT);

void setup() {
  // Debugging
  Serial.begin(9600);

  TCCR1B = 0x01;
  // Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.cp437(true);
  display.clearDisplay();
  display.display();

  bootMessage("Booting...");
  // Temperature Sensor
  temperatureError = !tempSensor.begin();
  temperatureReadMillis = millis(); 
  temperature =desiredTemperature; 

  for ( int i = 0; i< NUM_KEYS; i++ ) {
    pinMode(keyPins[i], INPUT_PULLUP);
    keyDebouncers[i] = new ADebouncer();
    keyDebouncers[i]->mode(INSTANT, 10, true);
    keyStates[i] = false;
    lastKeyStates[i] = false;
  }

  screenModeMillis = millis();
  keyMillis = millis();

  // Fans
  pinMode(inFanPin, OUTPUT);
  pinMode(outFanPin, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-99, 99);
}

void bootMessage(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.write(msg);
  display.display();
}


void loop() {
  updateKeys();
  updateTemperature();
  updateDisplay();
  //val = digitalRead(clearPin);
  controlPeltier();

  delay(100);
}

void controlPeltier() {
  //float delta = desiredTemperature - temperature;
  myPID.Compute();


  float actualPeltier = peltierPercentage;

  if ( actualPeltier > 3) {
    double fanPercentage = abs(actualPeltier * 3); // At 33% Peltier power fans are running 100%
    fanPercentage = max(fanPercentage, 0);
    fanPercentage = min(fanPercentage, 100);
    fanOutPercentage = (int)fanPercentage;
    fanInPercentage = (int)fanPercentage;

  }
  else {
    fanInPercentage = 0;
    fanOutPercentage = 0;
  }

  setFanSpeed(outFanPin, outFanMin, outFanMax, fanOutPercentage);
  setFanSpeed(inFanPin, inFanMin, inFanMax, fanInPercentage);
}

void setFanSpeed(int pin, float fanMin, float fanMax, int percentage) {
  if ( percentage < 2) {
      digitalWrite(pin, 0);
  }
  else {
    int analogOut = ((fanMax-fanMin)*percentage)/100.0 + fanMin;
    analogWrite(pin, analogOut);
  }
}

void updateKeys() {
  for ( int i = 0; i < NUM_KEYS; i++ ) {
    auto state = !keyDebouncers[i]->debounce(digitalRead(keyPins[i]));
    if ( !lastKeyStates[i] && state) {
      lastKeyStates[i] = keyStates[i] = state;
    }
    else {
      lastKeyStates[i] = state;
      keyStates[i] = false;
    }   
  }

  if ( keyStates[infoKey] ) {
     setScreenMode(getNextScreenMode(screenMode)); 
  }

  if ( lastKeyStates[plusKey] || lastKeyStates[minusKey] ) {
    if ( screenMode == settingScreen ) {
      if ( millis() > keyMillis + KEY_REPEAT_DELAY ) {
        
        keyMillis = millis();
        if ( lastKeyStates[plusKey] ) {
            desiredTemperature += 1;
        }
        if ( lastKeyStates[minusKey] ) {
            desiredTemperature -= 1;
        }
        if ( desiredTemperature < 0 ) desiredTemperature = 0;
        if ( desiredTemperature > 40 ) desiredTemperature = 40;
      } 
    }
    setScreenMode(settingScreen);
  }

  if ( millis() > screenModeMillis + SCREEN_MODE_SLEEP ) {
    setScreenMode(runningScreen);
  }
}

void setScreenMode ( ScreenMode mode ) {
  screenMode = mode;
  screenModeMillis = millis();
}


void updateTemperature() {  
    if ( temperatureReadMillis > millis() ) {
      temperatureReadMillis = 0;
      return;
    }

    if ( temperatureReadMillis + TEMP_READ_DELAY > millis()) return;

    temperatureReadMillis = millis();
    auto status = tempSensor.read();
    switch ( status ) {
      case AM2302::AM2302_ERROR_CHECKSUM:
        Serial.println(F("AM2302: Checksum error"));
        break;
      case AM2302::AM2302_ERROR_READ_FREQ:
        Serial.println(F("AM2302: Read Frequency error"));
        break;
      case AM2302::AM2302_ERROR_TIMEOUT:
        Serial.println(F("AM2302: Timeout error"));
        break;
      case AM2302::AM2302_READ_OK:
        temperatureError = false;
        temperature = tempSensor.get_Temperature();
        humidity = tempSensor.get_Humidity();        
        return;
    } 
    temperatureError = true;
}

ScreenMode getNextScreenMode(ScreenMode currentMode) {
  switch ( currentMode ) {
    case detailsScreen:
      return graphsScreen;
    case graphsScreen:
    default:
      return detailsScreen;
  }
}

void updateDisplay() {
  switch ( screenMode )  {
    case runningScreen:
      displayRunningScreen();
      break;
    case detailsScreen:
      displayDetailScreen();
      break;
    case settingScreen:
      displaySettingScreen();
      break;
    case graphsScreen:
      displayGraphScreen();
      break;
  }  
}


const uint8_t PROGMEM waterIcon [8] = {
  0b00100000,
  0b00100000,
  0b01010000,
  0b01010000,
  0b10001000,
  0b10001000,
  0b01110000,
  0b00000000,
};


void displayDetailScreen() {
  int line = 3;
  int x = 13;
  const int lineHeight = 9;

  display.clearDisplay();
  display.drawRoundRect(7, 0, 58, 32, 4, SSD1306_WHITE);

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.drawBitmap(x,line,waterIcon,8,8,SSD1306_WHITE);
  display.setCursor(x+12,line);  
  displayPercentage(humidity);
  line += lineHeight;
  display.setCursor(x,line);  
  display.write("T ");
  line += lineHeight;
  displayTemperature(temperature);
  display.setCursor(x,line);  
  display.write(0x1a);
  display.write(' ');
  displayTemperature(desiredTemperature);

  line = 3;
  x = 80;

  display.drawRoundRect(73, 0, 47, 32, 4, SSD1306_WHITE);
  display.setCursor(x, line);
  display.write("I ");
  displayPercentage(fanInPercentage);
  line+= lineHeight;
  display.setCursor(x, line);
  display.write("O ");
  displayPercentage(fanOutPercentage);
  line+= lineHeight;
  display.setCursor(x, line);
  display.write(0xf0);
  if ( peltierPercentage >= 0 ) {
    display.write('+');
  }
  else {
    display.write('-');
  }
  displayPercentage(abs(peltierPercentage));

  display.display();
}

void displaySettingScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.write("Set temperature:");
  display.setTextSize(2);
  display.setCursor(32, 15);
  display.drawRoundRect(16, 10, 96, 22, 4, SSD1306_WHITE);
  displayTemperature(desiredTemperature); 
  display.display();
}

void displayGraphScreen() {
  bootMessage("Graphs");
}

void displayRunningScreen() {
  display.clearDisplay();
  display.display();
}

void displayTemperature(float temperature) {
  display.write(((int)(temperature/10)%10)+0x30);
  display.write(((int)(temperature)%10)+0x30);
  display.write('.');
  display.write(((int)(temperature*10)%10)+0x30);
  display.write(0xf8);
  display.write('C');
}

void displayPercentage(float humidity) {
  if ( humidity / 100 > 1.0) {
    display.write(((int)(humidity/100)%10)+0x30);
  }
  else {
    display.write(' ');
  }
  if ( humidity / 10 > 1.0) {
    display.write(((int)(humidity/10)%10)+0x30);
  }
  else {
    display.write(' ');
  }
  display.write(((int)(humidity)%10)+0x30);
  display.write('%');
}


void displayError() {
  const char PROGMEM error[] = "ERROR";
  display.write(error);
}


