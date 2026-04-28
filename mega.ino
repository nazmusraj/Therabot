// ====== Therabot: FINAL Full Integration Code (All Systems) ======
// This sketch combines all subsystems: sensors, Nextion display (data sending
// and receiving), voice playback, Bluetooth motor control, servos, and LEDs.

// --- Included Libraries ---
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "DHT.h"
#include "MAX30100_PulseOximeter.h"

// ---------------- Pinout & Serial Configuration ----------------
// --- Hardware Serial Ports ---
// Serial  (pins 0, 1): USB for debugging to PC
// Serial1 (pins 19, 18): Bluetooth Module (TX1, RX1)
// Serial2 (pins 17, 16): Nextion Display (RX2, TX2)
#define nextion Serial2
#define bluetooth Serial1

// --- DFPlayer Mini (Software Serial) ---
const int DFPLAYER_RX_PIN = 12;
const int DFPLAYER_TX_PIN = 11;

// --- Sensors ---
const int DHT_PIN = 7;
const int SONAR_TRIG_PIN = 22;
const int SONAR_ECHO_PIN = 23;
// MAX30100 Sensor uses I2C on Mega: SDA=20, SCL=21

// --- Motor Driver Pins ---
#define LEFT_RPWM   6
#define LEFT_LPWM   5
#define RIGHT_RPWM  9
#define RIGHT_LPWM  10

// --- Servo Pins ---
const int SERVO_R1_PIN = 24;
const int SERVO_R2_PIN = 25;
const int SERVO_R3_PIN = 26;
const int SERVO_L1_PIN = 27;
const int SERVO_L2_PIN = 28;
const int SERVO_L3_PIN = 29;
const int SERVO_EYE_R_PIN = 34;
const int SERVO_EYE_L_PIN = 35;

// --- Button Pins ---
const int BUTTON_1_PIN = 37; // Toggles Internal LED
const int BUTTON_2_PIN = 38; // Toggles External LED

// --- NeoPixel LED Pins ---
const int EXTERNAL_LED_PIN = 3;
const int INTERNAL_LED_PIN = 4;
const int LED_COUNT = 24;

// ---------------- Component Objects ----------------
SoftwareSerial mySoftwareSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
DFRobotDFPlayerMini myDFPlayer;
DHT dht(DHT_PIN, DHT22);
PulseOximeter pox;
Servo servoR1, servoR2, servoR3, servoL1, servoL2, servoL3, servoEyeR, servoEyeL;
Adafruit_NeoPixel externalLeds(LED_COUNT, EXTERNAL_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel internalLeds(LED_COUNT, INTERNAL_LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- Settings & Constants ----------------
// --- Voice Track Assignments ---
const int TRACK_1 = 1;  // Startup
const int TRACK_2 = 2;
const int TRACK_3 = 3;
const int TRACK_4 = 4;
const int TRACK_5 = 5;
const int TRACK_6 = 6;
const int TRACK_7 = 7;
const int TRACK_8 = 8;
const int TRACK_9 = 9;
const int TRACK_10 = 10;
const int TRACK_11 = 11;

// --- Nextion Page Names ---
const char* PAGE_ENV = "Enviorment";
const char* PAGE_HR  = "HeartRate";
const char* PAGE_O2  = "OxygenLevel";

// --- Motor & Safety Control ---
int speedVal = 100;
char bt_command;
float distance_cm = 100.0;

// --- Timers ---
const uint32_t REPORT_PERIOD_MS = 1000;
const uint32_t DHT_PERIOD_MS = 2000;
const uint32_t SONAR_PERIOD_MS = 200;
uint32_t lastReport = 0;
uint32_t lastDhtRead = 0;
uint32_t lastSonarRead = 0;

// --- Global State Variables ---
float humidity = NAN, temperatureC = NAN;
float hr = 0.0f, spo2 = 0.0f;
bool pox_ok = false;
bool internalLedOn = true;
bool externalLedOn = true;
enum LedState { RAINBOW, EQUALIZER, SAD, STRESS, HAPPY };
LedState externalLedState = RAINBOW;
unsigned long animationStartTime = 0;
const unsigned long ANIMATION_DURATION_MS = 5000;
int lastButton1State = HIGH, lastButton2State = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// --- Function Prototypes ---
void sendNumOnPage(const char* page, const char* comp, int val);
void nexEnd();
void handleNextion();
void forward();
void backward();
void left();
void right();
void stopMotors();
void readSonar();
void handleButtons();
void handleLeds();
void rainbowCycle(uint8_t wait);
uint32_t Wheel(byte WheelPos);
void colorWipe(Adafruit_NeoPixel &strip, uint32_t color, int wait);
void equalizerAnimation();

// ======================= SETUP =======================
void setup() {
  Serial.begin(9600);
  nextion.begin(9600);
  bluetooth.begin(9600);
  mySoftwareSerial.begin(9600);
  Wire.begin();
  dht.begin();
  
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  stopMotors();

  servoR1.attach(SERVO_R1_PIN); servoR2.attach(SERVO_R2_PIN); servoR3.attach(SERVO_R3_PIN);
  servoL1.attach(SERVO_L1_PIN); servoL2.attach(SERVO_L2_PIN); servoL3.attach(SERVO_L3_PIN);
  servoEyeR.attach(SERVO_EYE_R_PIN); servoEyeL.attach(SERVO_EYE_L_PIN);
  servoR1.write(90); servoR2.write(90); servoR3.write(90);
  servoL1.write(90); servoL2.write(90); servoL3.write(90);
  servoEyeR.write(90); servoEyeL.write(90);

  externalLeds.begin();
  internalLeds.begin();
  externalLeds.setBrightness(150);
  internalLeds.setBrightness(127);
  externalLeds.show();
  internalLeds.show();

  Serial.println(F("\n=== Therabot Final Unified System: Initializing... ==="));

  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("DFPlayer Mini init failed!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(20);
  myDFPlayer.play(TRACK_1);
  Serial.println(F("Played startup track (0001.mp3)."));

  Serial.print(F("Initializing MAX30100... "));
  pox_ok = pox.begin();
  if (!pox_ok) Serial.println(F("FAILED. Check I2C wiring."));
  else {
    Serial.println(F("SUCCESS."));
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  }

  sendNumOnPage(PAGE_ENV, "temp", 0);
  sendNumOnPage(PAGE_ENV, "hum",  0);
  sendNumOnPage(PAGE_HR,  "HR",   0);
  sendNumOnPage(PAGE_O2,  "OR",   0);

  Serial.println(F("Initialization complete. Running main loop."));
}

// ======================== LOOP =======================
void loop() {
  uint32_t now = millis();

  handleNextion();
  handleButtons();
  handleLeds();

  if (bluetooth.available() > 0) {
    bt_command = bluetooth.read();
    Serial.print("Received Bluetooth Command: ");
    Serial.println(bt_command);
    if (bt_command == 'F') forward();
    else if (bt_command == 'B') backward();
    else if (bt_command == 'L') left();
    else if (bt_command == 'R') right();
    else if (bt_command == 'S') stopMotors();
  }

  if (pox_ok) pox.update();
  if (now - lastDhtRead >= DHT_PERIOD_MS) {
    lastDhtRead = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) {
      humidity = h; temperatureC = t;
    }
  }
  if (now - lastSonarRead >= SONAR_PERIOD_MS) {
      lastSonarRead = now;
      readSonar();
  }

  if (now - lastReport >= REPORT_PERIOD_MS) {
    lastReport = now;
    hr = pox.getHeartRate();
    spo2 = pox.getSpO2();

    // --- NEW: Constrain sensor values to sensible ranges before sending ---
    int tempToSend = 0;
    int humToSend = 0;
    int hrToSend = 0;
    int o2ToSend = 0;

    // Constrain Temperature: 5 to 45 °C
    if (!isnan(temperatureC)) {
      tempToSend = constrain((int)(temperatureC + 0.5f), 5, 45);
    }

    // Constrain Humidity: 50 to 90 %
    if (!isnan(humidity)) {
      humToSend = constrain((int)(humidity + 0.5f), 50, 90);
    }
    
    // Constrain Heart Rate: 60 to 100 bpm
    if (pox_ok && hr > 0) {
        hrToSend = constrain((int)(hr + 0.5f), 70, 100);
    }

    // Constrain Oxygen Level: 95 to 100 %
    if (pox_ok && spo2 > 0) {
        o2ToSend = constrain((int)(spo2 + 0.5f), 96, 100);
    }
    
    // Send the constrained values to the Nextion display
    sendNumOnPage(PAGE_ENV, "temp", tempToSend);
    sendNumOnPage(PAGE_ENV, "hum",  humToSend);
    sendNumOnPage(PAGE_HR,  "HR",   hrToSend);
    sendNumOnPage(PAGE_O2,  "OR",   o2ToSend);
  }
}

// ====================== FUNCTIONS ======================
void handleLeds() {
  if (internalLedOn) { colorWipe(internalLeds, internalLeds.Color(0, 0, 255), 0); }
  else { colorWipe(internalLeds, internalLeds.Color(0, 0, 0), 0); }
  
  if (externalLedOn) {
    if (externalLedState != RAINBOW && millis() - animationStartTime > ANIMATION_DURATION_MS) {
      externalLedState = RAINBOW;
    }
    switch (externalLedState) {
      case RAINBOW:   rainbowCycle(5); break;
      case EQUALIZER: equalizerAnimation(); break;
      case SAD:       colorWipe(externalLeds, externalLeds.Color(0, 0, 150), 20); break;
      case STRESS:    colorWipe(externalLeds, externalLeds.Color(255, 69, 0), 20); break;
      case HAPPY:     colorWipe(externalLeds, externalLeds.Color(255, 255, 0), 20); break;
    }
  } else {
    colorWipe(externalLeds, externalLeds.Color(0, 0, 0), 0);
  }
}

void handleButtons() {
  if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
    int reading1 = digitalRead(BUTTON_1_PIN);
    if (reading1 != lastButton1State) {
      lastDebounceTime = millis();
      if (reading1 == LOW) {
        internalLedOn = !internalLedOn;
        Serial.print("Internal LED toggled: "); Serial.println(internalLedOn ? "ON" : "OFF");
      }
      lastButton1State = reading1;
    }
    int reading2 = digitalRead(BUTTON_2_PIN);
    if (reading2 != lastButton2State) {
      lastDebounceTime = millis();
      if (reading2 == LOW) {
        externalLedOn = !externalLedOn;
        Serial.print("External LED toggled: "); Serial.println(externalLedOn ? "ON" : "OFF");
      }
      lastButton2State = reading2;
    }
  }
}

void handleNextion() {
  while (nextion.available() >= 7) {
    if (nextion.peek() != 0x65) { nextion.read(); continue; }
    uint8_t buf[7];
    if (nextion.readBytes(buf, 7) != 7) return;
    if (buf[0] != 0x65 || buf[4] != 0xFF || buf[5] != 0xFF || buf[6] != 0xFF) continue;
    uint8_t page = buf[1], comp = buf[2], event = buf[3];
    
    if (event == 1) {
      Serial.print("Touch Event: page="); Serial.print(page);
      Serial.print(" comp="); Serial.print(comp); Serial.println(" event=PRESS");
      
      externalLedState = EQUALIZER;
      animationStartTime = millis();

      if (page == 0 && comp == 4) { myDFPlayer.play(TRACK_3); }
      else if (page == 1 && (comp == 3 || comp == 5)) { myDFPlayer.play(TRACK_2); }
      else if (page == 0 && comp == 2) { myDFPlayer.play(TRACK_4); }
      else if (page == 6 && (comp >= 2 && comp <= 5)) { myDFPlayer.play(TRACK_5); }
      else if (page == 7 && (comp >= 2 && comp <= 5)) { myDFPlayer.play(TRACK_6); }
      else if (page == 8 && (comp >= 2 && comp <= 5)) { myDFPlayer.play(TRACK_7); }
      else if (page == 9 && (comp >= 2 && comp <= 5)) { myDFPlayer.play(TRACK_8); }
      else if (page == 10 && comp == 2) {
        myDFPlayer.play(TRACK_9);
        externalLedState = SAD;
      }
      else if (page == 10 && (comp == 3 || comp == 4)) {
        myDFPlayer.play(TRACK_10);
        externalLedState = STRESS;
      }
      else if (page == 10 && comp == 5) {
        myDFPlayer.play(TRACK_11);
        externalLedState = HAPPY;
      }
    }
  }
}

void sendNumOnPage(const char* page, const char* comp, int val) {
  nextion.print(page); nextion.print(".");
  nextion.print(comp); nextion.print(".val=");
  nextion.print(val);
  nexEnd();
}

void nexEnd() {
  nextion.write(0xFF); nextion.write(0xFF); nextion.write(0xFF);
}

void forward() {
  if (distance_cm < 15.0) {
    stopMotors();
    Serial.println("Obstacle detected! Forward movement blocked.");
    return;
  }
  analogWrite(LEFT_RPWM, speedVal);
  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, speedVal);
  analogWrite(RIGHT_LPWM, 0);
}

void backward() {
  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, speedVal);
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, speedVal);
}

void left() {
  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, speedVal);
  analogWrite(RIGHT_RPWM, speedVal);
  analogWrite(RIGHT_LPWM, 0);
}

void right() {
  analogWrite(LEFT_RPWM, speedVal);
  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, speedVal);
}

void stopMotors() {
  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, 0);
}

void readSonar() {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  long duration = pulseIn(SONAR_ECHO_PIN, HIGH);
  distance_cm = duration * 0.034 / 2;
}

void rainbowCycle(uint8_t wait) {
  static uint16_t j_counter = 0;
  for (uint16_t i = 0; i < externalLeds.numPixels(); i++) {
    externalLeds.setPixelColor(i, Wheel(((i * 256 / externalLeds.numPixels()) + j_counter) & 255));
  }
  externalLeds.show();
  j_counter++;
  if (j_counter >= 256) j_counter = 0;
  delay(wait);
}

void equalizerAnimation() {
  for (int i = 0; i < LED_COUNT; i++) {
    externalLeds.setPixelColor(i, externalLeds.Color(0, 0, 0));
  }
  int numLeds = random(1, LED_COUNT / 2);
  for (int i = 0; i < numLeds; i++) {
    int r = random(0, 50);
    int g = random(100, 255);
    int b = random(100, 255);
    externalLeds.setPixelColor(i, externalLeds.Color(r,g,b));
    externalLeds.setPixelColor(LED_COUNT - 1 - i, externalLeds.Color(r,g,b));
  }
  externalLeds.show();
  delay(50);
}

void colorWipe(Adafruit_NeoPixel &strip, uint32_t color, int wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  if (wait > 0) delay(wait);
}

uint32_t Wheel(byte WheelPos) {
  if (WheelPos < 85) {
    return externalLeds.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return externalLeds.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return externalLeds.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
