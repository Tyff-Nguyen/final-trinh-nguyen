#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//=== Motor Control Pins ===
// #define PIN_PWM 16
// #define PIN_DIR_A 17
// #define PIN_DIR_B 5

// // === Encoder Pins ===
// #define PIN_ENC_A 13
// #define PIN_ENC_B 14

// === Motor and PID Constants ===
const float PULSES_PER_ROT = 920.0;
const int MAX_PWM_VALUE = 255;
const unsigned long SPEED_UPDATE_MS = 200;

float kpGain = 20.0;
float kiGain = 70.0;
float kdGain = 5.0;

// === BLE UUIDs ===
#define MOTOR_SVC_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RPM_TARGET_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RPM_ACTUAL_UUID     "ca73b3ba-39f6-4ab3-91ae-186dc9577d99"
#define PID_KP_UUID         "d5e4b2a1-3f8c-4e5b-9a2d-7b3e8c1f0a2b"
#define PID_KI_UUID         "e6f5c3b2-4f9d-5f6c-ab3e-8c4f9d2e1b3c"
#define PID_KD_UUID         "f7e6d4c3-5fae-4e7d-9c4f-9d5e0e3f2c4d"

// === BLE Characteristics ===
BLECharacteristic *charTargetRpm;
BLECharacteristic *charActualRpm;
BLECharacteristic *charKpGain;
BLECharacteristic *charKiGain;
BLECharacteristic *charKdGain;

// === Motor Control State ===
volatile int32_t encPosition = 0;
int32_t lastEncPosition = 0;
unsigned long lastSpeedTick = 0;
unsigned long lastPidTick = 0;

float setpointRpm = 0;
float currentRpm = 0;
float pidError = 0;
float prevPidError = 0;
float integralTerm = 0;
float derivativeTerm = 0;
float pwmSignal = 0;

// Configure encoder interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encAInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encBInterrupt, CHANGE);

  // Set up BLE
  BLEDevice::init("MotorBLE");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ConnectionHandler());
  BLEService *service = server->createService(MOTOR_SVC_UUID);

// === Encoder ISR Handlers ===
void IRAM_ATTR encAInterrupt() {
  encPosition += (digitalRead(PIN_ENC_A) == digitalRead(PIN_ENC_B)) ? 1 : -1;
}
void IRAM_ATTR encBInterrupt() {
  encPosition += (digitalRead(PIN_ENC_A) != digitalRead(PIN_ENC_B)) ? 1 : -1;
}

// === PWM via LEDC for ESP32 ===
void setupPWM() {
  ledcSetup(0, 20000, 8); // Channel 0, 20kHz, 8-bit resolution
  ledcAttachPin(PIN_PWM, 0);
}

void applyPwm(float pwm) {
  int pwmAbs = abs(pwm);
  digitalWrite(PIN_DIR_A, pwm > 0 ? HIGH : LOW);
  digitalWrite(PIN_DIR_B, pwm < 0 ? HIGH : LOW);
  ledcWrite(0, pwmAbs); // LEDC for PWM
}

// === BLE Write Handlers ===
class TargetRpmHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String data = c->getValue().c_str();
    setpointRpm = data.toFloat();
    integralTerm = 0;
    Serial.printf("Setpoint updated: %.1f RPM\n", setpointRpm);
  }
};
class KpGainHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c)*
