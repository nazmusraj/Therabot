// ====== Therabot: Arduino Nano Subsystem Controller ======
// This code manages access control, local display, and a sensor array.
// VERSION UPDATE: Gate servo now starts at 180 degrees and toggles on each scan.

// --- Included Libraries ---
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <Servo.h>

// --- I2C Devices ---
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- PN532 (SPI) ---
#define PN532_SS 10
Adafruit_PN532 nfc(PN532_SS);

// --- Servos ---
#define SERVO_PIN 9         // Gate servo
#define SERVO2_PIN 6        // IR-triggered servo
#define CHOCOLATE_SERVO_PIN 5 // Chocolate dispenser servo
Servo gateServo;
Servo irServo;
Servo chocolateServo;

// --- Target RFID Cards ---
const uint8_t TARGET_UID_1[] = { 0x13, 0x5E, 0xBA, 0x02 };
const uint8_t TARGET_UID_2[] = { 0x13, 0x34, 0x11, 0xA5 };
const uint8_t TARGET_UID_3[] = { 0x04, 0x19, 0x2A, 0xF2, 0xD4, 0x13, 0x91 };
const uint8_t TARGET_UID_4[] = { 0x04, 0x7C, 0x21, 0xF2, 0xD4, 0x13, 0x90 };
const uint8_t TARGET_LEN_4 = 4;
const uint8_t TARGET_LEN_7 = 7;

// --- Relay + Button ---
#define EMERGENCY_BUTTON_PIN 4
#define RELAY_PIN  7
const bool RELAY_ACTIVE_HIGH = true;
bool relayState = false;

// --- Digital IR sensor ---
#define IR_PIN 8
#define IR_ACTIVE_LOW true

// --- State Variables ---
bool irPrevDetected   = false;
bool irActionActive   = false;
unsigned long irActionStart = 0;
const unsigned long IR_HOLD_MS = 2000;
bool irAtZero         = false;
bool gateIsOpen  = true; // Gate starts in the open state
bool matchedPrev = false;
unsigned long buttonPressStartTime = 0;
bool buttonIsBeingHeld = false;
bool longPressActionDone = false;
const unsigned long LONG_PRESS_MS = 3000;
bool chocolateServoActive = false;
unsigned long chocolateServoStartTime = 0;
const unsigned long CHOCOLATE_OPEN_MS = 10000;

// ======================= HELPER FUNCTIONS =======================
void printUid(const uint8_t *uid, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) Serial.print('0');
    Serial.print(uid[i], HEX);
    if (i < len - 1) Serial.print(':');
  }
}

bool uidEquals(const uint8_t *uid, uint8_t len, const uint8_t *ref, uint8_t refLen) {
  if (len != refLen) return false;
  for (uint8_t i = 0; i < len; i++) if (uid[i] != ref[i]) return false;
  return true;
}

void applyRelayOutput() {
  bool pinLevel = relayState ? RELAY_ACTIVE_HIGH : !RELAY_ACTIVE_HIGH;
  digitalWrite(RELAY_PIN, pinLevel ? HIGH : LOW);
  lcd.setCursor(11, 0);
  lcd.print("RY:");
  lcd.print(relayState ? "ON " : "OFF");
}

void smoothMove(Servo& s, int from, int to, int stepDelayMs = 8) {
  if (to > from) {
    for (int p = from; p <= to; p++) { s.write(p); delay(stepDelayMs); }
  } else if (to < from) {
    for (int p = from; p >= to; p--) { s.write(p); delay(stepDelayMs); }
  } else {
    s.write(to);
  }
}

void moveGateTo(int targetDeg) {
  int current = gateServo.read();
  smoothMove(gateServo, current, targetDeg, 8);
  gateIsOpen = (targetDeg == 180);

  lcd.setCursor(0, 0);
  if (gateIsOpen) { lcd.print("GATE OPEN      "); }
  else            { lcd.print("GATE CLOSED    "); }
}

void moveIRServoTo(int targetDeg) {
  int current = irAtZero ? 0 : 90;
  smoothMove(irServo, current, targetDeg, 8);
  irAtZero = (targetDeg == 0);
}

// =========================== SETUP ===========================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Therabot Ready");
  lcd.setCursor(0, 1); lcd.print("Init sensors...");

  if (!mlx.begin()) {
    Serial.println("MLX90614 not found!");
    lcd.clear(); lcd.print("MLX90614 ERROR");
  }

  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    Serial.println("PN532 not found");
    lcd.clear(); lcd.print("PN532 ERROR");
  }
  nfc.SAMConfig();

  // --- UPDATED: Initialize Servos ---
  gateServo.attach(SERVO_PIN);
  gateServo.write(180); // Start in the OPEN position
  gateIsOpen = true;

  irServo.attach(SERVO2_PIN);
  irServo.write(60);
  irAtZero = false;

  chocolateServo.attach(CHOCOLATE_SERVO_PIN);
  chocolateServo.write(90);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);
  applyRelayOutput();
  pinMode(IR_PIN, INPUT);

  delay(800);
  lcd.clear();
}

// ============================ LOOP ===========================
void loop() {
  handleEmergencyButton();
  handleChocolateServoTimer();

  float objectC = mlx.readObjectTempC();
  lcd.setCursor(0, 1);
  lcd.print("Obj:");
  lcd.print(objectC, 1);
  lcd.write(byte(223));
  lcd.print("C   ");

  uint8_t uid[7];
  uint8_t uidLen = 0;
  bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 120);
  
  bool matchedNow = false;
  if (found) {
    bool match1 = uidEquals(uid, uidLen, TARGET_UID_1, TARGET_LEN_4);
    bool match2 = uidEquals(uid, uidLen, TARGET_UID_2, TARGET_LEN_4);
    bool match3 = uidEquals(uid, uidLen, TARGET_UID_3, TARGET_LEN_7);
    bool match4 = uidEquals(uid, uidLen, TARGET_UID_4, TARGET_LEN_7);
    matchedNow = match1 || match2 || match3 || match4;
  }
  
  // --- UPDATED: RFID Toggle Logic ---
  if (matchedNow && !matchedPrev) {
    if (gateIsOpen) {
      // If gate is open, close it
      moveGateTo(90);
    } else {
      // If gate is closed, open it
      moveGateTo(180);
    }
  }
  matchedPrev = matchedNow;
  
  bool irDetected = (digitalRead(IR_PIN) == LOW);
  if (irDetected && !irPrevDetected && !irActionActive) {
    moveIRServoTo(90);
    irActionActive = true;
    irActionStart = millis();
  }
  if (irActionActive && (millis() - irActionStart >= IR_HOLD_MS)) {
    moveIRServoTo(60);
    irActionActive = false;
  }
  irPrevDetected = irDetected;

  delay(50);
}

// ====================== HELPER/HANDLER FUNCTIONS ======================
void handleEmergencyButton() {
  int reading = digitalRead(EMERGENCY_BUTTON_PIN);

  if (reading == LOW && !buttonIsBeingHeld) {
    buttonIsBeingHeld = true;
    longPressActionDone = false;
    buttonPressStartTime = millis();
  }

  if (reading == HIGH && buttonIsBeingHeld) {
    buttonIsBeingHeld = false;
    if (!longPressActionDone) {
      Serial.println("Short Press: Opening chocolate dispenser.");
      chocolateServo.write(180);
      chocolateServoActive = true;
      chocolateServoStartTime = millis();
    }
  }

  if (buttonIsBeingHeld && !longPressActionDone) {
    if (millis() - buttonPressStartTime > LONG_PRESS_MS) {
      Serial.println("Long Press: Toggling relay.");
      relayState = !relayState;
      applyRelayOutput();
      longPressActionDone = true;
    }
  }
}

void handleChocolateServoTimer() {
  if (chocolateServoActive) {
    if (millis() - chocolateServoStartTime >= CHOCOLATE_OPEN_MS) {
      Serial.println("Timer complete. Closing chocolate dispenser.");
      chocolateServo.write(90);
      chocolateServoActive = false;
    }
  }
}
