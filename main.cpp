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

  float rampAngle = NAN; // declare ramp angle

  Serial.println("Going UP ramp...");

  // --- Step A: Move forward, detect ramp (>2 deg), accelerate, stop when angle stops changing ---
  Serial.println("Searching for ramp...");
  lcd.setCursor(0, 1);
  lcd.print("Searching Ramp ");

  forward(150); // always moving forward first

  bool rampStarted = false;
  float lastPitch = NAN;
  unsigned long noChangeStart = 0;

  while (true) {
      float pitch = getPitch();
      if (isnan(pitch)) {
          delay(20);
          continue;
      }

      // --- Show pitch on LCD ---
      lcd.setCursor(0, 0);
      lcd.print("Angle:          ");
      lcd.setCursor(7, 0);
      lcd.print(pitch, 1);

      // ===== 1. Ramp entry detection (pitch > 2°) =====
      if (!rampStarted && pitch > 2) {
          Serial.println("Ramp detected! Accelerating...");
          lcd.setCursor(0, 1);
          lcd.print("Ramp Up...      ");
          rampStarted = true;

          forward(150);       // accelerate
          lastPitch = pitch;  // store reference angle
          noChangeStart = millis();  // start timer for no-change detection
      }

      // ===== 2. Ramp already started → monitor angle change =====
      if (rampStarted) {
          forward(200); // keep accelerating

          // check if pitch changed significantly (less than 0.3° difference)
          if (abs(pitch - lastPitch) < 1.5) {
              // angle stable — check if 1 sec passed
              if (millis() - noChangeStart >= 20) {
                  Serial.println("Pitch stable 2 sec — stopping!");
                  motorsStop();
                  rampAngle = pitch;  // store angle
                  delay(300);
                  break;
              }
          } else {
              // angle changed → reset timer
              lastPitch = pitch;
              noChangeStart = millis();
          }
      }

      delay(20);
  }


  // --- Display angle once on LCD ---
  lcd.setCursor(0, 0);
  lcd.print("Angle:          ");
  lcd.setCursor(7, 0);
  if (isnan(rampAngle)) lcd.print("---");
  else lcd.print(rampAngle, 1);

  // --- Step B: Wait on ramp BEFORE going up ---
  Serial.println("Holding on ramp...");
  lcd.setCursor(0, 1);
  lcd.print("Holding...      ");
  delay(2000);   // <-- adjust this wait time if needed

  // --- Step C: Accelerate up ramp, then decelerate when reaching top ---
  Serial.println("Continuing up ramp...");

  while (true) {
      float pitch = getPitch();
      if (isnan(pitch)) {
          delay(40);
          continue;
      }

      // --- Accelerate while still climbing ---
      if (pitch >= 10) {  
          forward(200);   // strong climb
      }

      // --- Decelerate when nearing the top ---
      else {  
          Serial.println("Beginning deceleration...");

          int currentSpeed = 200;   // starting speed
          int targetSpeed  = 0;   // lowest speed

          // Smooth deceleration
          for (float sp = currentSpeed; sp >= targetSpeed; sp -= 5) {
              forward(sp);
              delay(40);

              float p = getPitch(); 
              if (!isnan(p)) pitch = p;

              // stop early if fully flat
              if (pitch < 0) {
                  Serial.println("Ramp top reached!");
                  break;
              }
          }

          break;  // leave the main while-loop
      }

      delay(40);
  }

  motorsStop();



  // --- Step 2: Wait 4 seconds ---
  Serial.println("Waiting on ramp...");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...      ");
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

    // optional: show yaw progress on LCD (bottom row)
    lcd.setCursor(0, 1);
    lcd.print("Rot: ");
    lcd.print((int)rotated);
    lcd.print("    ");

    delay(20);
  }

  motorsStop();
  delay(500);

  // --- Step 4: Go DOWN ramp ---
  Serial.println("Going DOWN ramp...");
  forward(150);
  delay(2000);
  while (true) {
    float downPitch = getPitch();
    // show on lcd
    lcd.setCursor(0, 0);
    lcd.print("Angle:          ");
    lcd.setCursor(7, 0);
    if (isnan(downPitch)) lcd.print("---");
    else lcd.print(downPitch, 1);

    if (!isnan(downPitch) && downPitch > -10) break;
    delay(50);
  }
  motorsStop();

  // --- Done ---
  Serial.println("Finished ramp sequence.");

  // Print final angle and finished
  lcd.setCursor(0, 0);
  lcd.print("Angle:          ");
  lcd.setCursor(7, 0);
  if (isnan(rampAngle)) lcd.print("---");
  else lcd.print(rampAngle, 1);

  lcd.setCursor(0, 1);
  lcd.print("Finished       ");

  while (1);
}

