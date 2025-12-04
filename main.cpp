// A4 for SDA, A5 for SCL, INT for 2

#include <Wire.h>
#include <LiquidCrystal.h>
#include "MPU6050_6Axis_MotionApps20.h"

// ===== MPU6050 =====
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;

// ===== Motor Pins =====
const int ENA = 11;   // Left PWM
const int IN1 = A1;
const int IN2 = A2;

const int ENB = 3;    // Right PWM
const int IN3 = A3;
const int IN4 = 12;

// ===== LCD Setup =====
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS, E, D4, D5, D6, D7

// ===== Motor Functions =====
void motorsStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void backward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void spinRight(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// ===== Get Yaw and Pitch =====
float getYaw() {
  uint8_t fifoBuffer[64];
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    float yawDeg = ypr[0] * 180.0 / M_PI;
    if (yawDeg < 0) yawDeg += 360;
    return yawDeg;
  }
  return NAN;
}

float getPitch() {
  uint8_t fifoBuffer[64];
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    return ypr[1] * 180.0 / M_PI; // pitch
  }
  return NAN;
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=A4, SCL=A5

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // LCD initialization
  lcd.begin(16, 2);
  lcd.print("Ramp Angle:");

  // MPU6050 initialization
  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    lcd.setCursor(0, 1);
    lcd.print("MPU Init Failed");
    Serial.println("MPU initialization failed!");
    while (1);
  }

  delay(1000);
}

// ===== Main Loop =====
void loop() {
  if (!dmpReady) return;

  // --- Step 1: Go UP ramp ---
  // Phase 1: Move slightly onto the ramp
  Serial.println("Going UP ramp (initial move)...");
  forward(200);
  delay(1000);   // Move only a short distance
  motorsStop();
  delay(300);   // Stabilize the MPU reading

  // Phase 2: Measure ramp angle clearly while stopped
  float rampAngle = getPitch();
  if (!isnan(rampAngle)) {
    lcd.setCursor(0, 1);
    lcd.print("Angle: ");
    lcd.print(rampAngle, 1);
    lcd.print(" deg   ");

    Serial.print("Measured Ramp Angle: ");
    Serial.println(rampAngle);
  }

  // Phase 3: Continue climbing until slope flattens
  Serial.println("Climbing ramp...");


  while (true) {
    forward(200);
    float pitch = getPitch();
    if (!isnan(pitch) && abs(pitch) < 3) {  // flat area detected
      break;
    }
  }

  // --- Gradual slowdown at top of ramp ---
  int speed = 120;

  while (speed > 0) {
      forward(speed);
      speed -= 20;    // reduce speed step-by-step
      delay(120);     // small delay for smooth slowdown
  }

  motorsStop(); 


  // --- Step 2: Wait 4 seconds ---
  Serial.println("Waiting on ramp...");
  delay(4000);

  // --- Step 3: Spin 360 using yaw (Code 1 logic) ---
  Serial.println("Spinning 360...");
  float lastYaw = getYaw();
  if (isnan(lastYaw)) return;

  float rotated = 0;
  spinRight(135);

  while (rotated < 360) {
    float currentYaw = getYaw();
    if (isnan(currentYaw)) continue;

    float delta = currentYaw - lastYaw;
    if (delta < -180) delta += 360;
    if (delta > 180) delta -= 360;

    rotated += abs(delta);
    lastYaw = currentYaw;
  }

  motorsStop();
  delay(500);

  // --- Step 4: Go DOWN ramp ---
  Serial.println("Going DOWN ramp...");
  forward(150);
  delay(2000);
  while (true) {
    float pitch = getPitch();
    if (!isnan(pitch) && pitch > -10) break;
  }
  motorsStop();

  // --- Done ---
  Serial.println("Finished ramp sequence.");

  // Print "Ramp Angle:" at the top row
  lcd.setCursor(0, 0);
  lcd.print("Angle:     "); // spaces clear leftover chars

  lcd.setCursor(8, 0);     // print angle starting at column 8
  lcd.print(rampAngle, 1);  // show 1 decimal place

  // Print "Finished" at the bottom row
  lcd.setCursor(0, 1);
  lcd.print("Finished       "); // extra spaces to clear old text

  while (1);
}
