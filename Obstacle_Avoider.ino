/*
============================================================
       ESP32-S3 OBSTACLE-AVOIDER ROBOT
============================================================

Components:
  1. Three HC-SR04 ultrasonic sensors
  2. TB6612FNG dual motor driver
  3. Two potentiometers simulating cliff sensors
  4. One potentiometer simulating battery voltage
  5. MPU-6050 gyroscope/accelerometer
  6. WS2812B NeoPixel status LED
  7. Buzzer
  8. Enable switch

IMPORTANT:
  - Potentiometers are simulation substitutes, not real sensors.
  - Hall encoder feedback is not implemented.
  - The gyroscope is monitored but does not control turns.
  - Battery ADC readings are simulated, not actual voltage.
============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// ==========================================================
// 1. PIN CONFIGURATION
// ==========================================================

// Front ultrasonic sensor
const int FRONT_TRIG = 7;
const int FRONT_ECHO = 15;

// Left ultrasonic sensor
const int LEFT_TRIG = 5;
const int LEFT_ECHO = 6;

// Right ultrasonic sensor
const int RIGHT_TRIG = 16;
const int RIGHT_ECHO = 17;

// TB6612FNG motor driver
// Motor A = left motor
// Motor B = right motor
const int PWMA = 11;
const int AIN1 = 12;
const int AIN2 = 13;

const int PWMB = 14;
const int BIN1 = 21;
const int BIN2 = 38;

const int STBY = 18;

// Potentiometers
const int LEFT_CLIFF_PIN  = 4;
const int RIGHT_CLIFF_PIN = 2;
const int BATTERY_PIN     = 1;

// MPU-6050 I2C
const int MPU_SDA = 8;
const int MPU_SCL = 9;
const uint8_t MPU_ADDRESS = 0x68;

// Other components
const int BUZZER_PIN = 47;
const int NEOPIXEL_PIN = 48;
const int SWITCH_PIN = 35;

// One WS2812B NeoPixel
Adafruit_NeoPixel pixel(
  1,
  NEOPIXEL_PIN,
  NEO_GRB + NEO_KHZ800
);

// ==========================================================
// 2. ADJUSTABLE PARAMETERS
// ==========================================================

// An obstacle closer than this distance triggers avoidance.
const int OBSTACLE_DISTANCE_CM = 25;

// Simulation ADC thresholds: ESP32-S3 ADC is configured for
// nominal 12-bit readings from 0 to 4095.
const int CLIFF_THRESHOLD = 1800;
const int LOW_BATTERY_THRESHOLD = 1200;

// Motor PWM values: 0 to 255
const int NORMAL_SPEED = 180;
const int TURN_SPEED = 150;

// Approximate pivot duration; actual rotation depends on motors.
const unsigned long TURN_TIME_MS = 300;

// Ultrasonic echo timeout
const unsigned long ECHO_TIMEOUT_US = 25000;

// ==========================================================
// 3. GLOBAL VARIABLES
// ==========================================================

bool mpuOK = false;
bool turning = false;

unsigned long turnStarted = 0;
unsigned long lastPrint = 0;

String robotState = "INITIALIZING";

// ==========================================================
// 4. NEOPIXEL STATUS INDICATOR
// ==========================================================

// Green  = path clear
// Blue   = turning left
// Orange = turning right
// Red    = stopped / blocked / disabled
// Purple = simulated cliff detected
// Yellow = simulated low battery

void setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  pixel.setPixelColor(0, pixel.Color(r, g, b));
  pixel.show();
}

// ==========================================================
// 5. ULTRASONIC DISTANCE MEASUREMENT
// ==========================================================

// Returns distance in centimetres.
// Returns -1 if no echo was received before timeout.
// A timeout is NOT interpreted as a real distance.

long readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(
    echoPin,
    HIGH,
    ECHO_TIMEOUT_US
  );

  if (duration == 0) {
    return -1;
  }

  return duration / 58;
}

// ==========================================================
// 6. MOTOR CONTROL
// ==========================================================

// Stop both motors and disable the driver.
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  digitalWrite(STBY, LOW);

  turning = false;
}

// Drive both motors forward.
void moveForward(int speedValue) {
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, constrain(speedValue, 0, 255));
  analogWrite(PWMB, constrain(speedValue, 0, 255));

  turning = false;
}

// Pivot left.
void turnLeft() {
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  analogWrite(PWMA, TURN_SPEED);
  analogWrite(PWMB, TURN_SPEED);

  turning = true;
}

// Pivot right.
void turnRight() {
  digitalWrite(STBY, HIGH);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  analogWrite(PWMA, TURN_SPEED);
  analogWrite(PWMB, TURN_SPEED);

  turning = true;
}

// ==========================================================
// 7. MPU-6050 INITIALIZATION
// ==========================================================

bool initializeMPU() {
  // Wake up the MPU-6050.
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Configure gyro range to +/-250 degrees/second.
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x00);

  return Wire.endTransmission() == 0;
}

// Read gyroscope Z-axis angular velocity.
float readGyroZ() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);

  if (Wire.endTransmission(false) != 0) {
    return 0.0;
  }

  if (Wire.requestFrom((int)MPU_ADDRESS, 2, true) != 2) {
    return 0.0;
  }

  int16_t rawValue =
    ((int16_t)Wire.read() << 8) | Wire.read();

  return rawValue / 131.0f;
}

// ==========================================================
// 8. FORMATTED SERIAL MONITOR OUTPUT
// ==========================================================

void printStatus(
  long front,
  long left,
  long right,
  int cliffL,
  int cliffR,
  int battery,
  float gyroZ
) {
  Serial.println();
  Serial.println("========================================");
  Serial.println("       OBSTACLE AVOIDER STATUS");
  Serial.println("========================================");

  Serial.printf(
    "Distance (cm) : Front=%-4s Left=%-4s Right=%-4s\n",
    front < 0 ? "N/E" : String(front).c_str(),
    left  < 0 ? "N/E" : String(left).c_str(),
    right < 0 ? "N/E" : String(right).c_str()
  );

  Serial.println("----------------------------------------");

  Serial.printf(
    "Cliff ADC     : Left=%4d  Right=%4d\n",
    cliffL,
    cliffR
  );

  Serial.printf(
    "Battery ADC   : %4d / 4095\n",
    battery
  );

  Serial.println("----------------------------------------");

  Serial.printf(
    "MPU-6050      : %s\n",
    mpuOK ? "CONNECTED" : "NOT DETECTED"
  );

  Serial.printf(
    "Gyro Z        : %7.2f deg/s\n",
    gyroZ
  );

  Serial.println("----------------------------------------");

  Serial.printf(
    "Robot State   : %s\n",
    robotState.c_str()
  );

  Serial.println("========================================");
}

// ==========================================================
// 9. SETUP
// ==========================================================

void setup() {
  Serial.begin(115200);

  // Configure ADC resolution.
  analogReadResolution(12);

  // Ultrasonic pins
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);

  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  digitalWrite(FRONT_TRIG, LOW);
  digitalWrite(LEFT_TRIG, LOW);
  digitalWrite(RIGHT_TRIG, LOW);

  // Motor driver pins
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  // Potentiometer inputs
  pinMode(LEFT_CLIFF_PIN, INPUT);
  pinMode(RIGHT_CLIFF_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);

  // Enable switch: HIGH = enabled
  pinMode(SWITCH_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize I2C and MPU-6050.
  Wire.begin(MPU_SDA, MPU_SCL);
  mpuOK = initializeMPU();

  // Initialize NeoPixel.
  pixel.begin();
  pixel.setBrightness(40);
  setStatusColor(0, 0, 100);

  // Ensure the motors start stopped.
  stopMotors();

  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESP32-S3 OBSTACLE AVOIDER ROBOT");
  Serial.println("========================================");
  Serial.println("Firmware started successfully.");
  Serial.printf(
    "MPU-6050 status : %s\n",
    mpuOK ? "CONNECTED" : "NOT DETECTED"
  );
  Serial.println("Serial baud rate: 115200");
  Serial.println("Enable switch   : HIGH = ON");
  Serial.println("========================================");
}

// ==========================================================
// 10. MAIN CONTROL LOOP
// ==========================================================

void loop() {
  // --------------------------------------------------------
  // A. Check enable switch.
  // --------------------------------------------------------

  if (digitalRead(SWITCH_PIN) != HIGH) {
    stopMotors();
    noTone(BUZZER_PIN);

    robotState = "DISABLED";
    setStatusColor(100, 0, 0);

    delay(100);
    return;
  }

  // --------------------------------------------------------
  // B. Read ultrasonic sensors one at a time.
  // The short delays reduce sensor cross-talk.
  // --------------------------------------------------------

  long front = readDistanceCm(FRONT_TRIG, FRONT_ECHO);
  delay(30);

  long left = readDistanceCm(LEFT_TRIG, LEFT_ECHO);
  delay(30);

  long right = readDistanceCm(RIGHT_TRIG, RIGHT_ECHO);

  // --------------------------------------------------------
  // C. Read simulated cliff and battery inputs.
  // --------------------------------------------------------

  int cliffL = analogRead(LEFT_CLIFF_PIN);
  int cliffR = analogRead(RIGHT_CLIFF_PIN);
  int battery = analogRead(BATTERY_PIN);

  // Read MPU-6050 gyroscope.
  float gz = mpuOK ? readGyroZ() : 0.0f;

  // --------------------------------------------------------
  // D. Determine warning conditions.
  // --------------------------------------------------------

  // For this simulation, ADC below the threshold means cliff.
  bool cliffDetected =
    (cliffL < CLIFF_THRESHOLD) ||
    (cliffR < CLIFF_THRESHOLD);

  // Low ADC represents low simulated battery.
  bool lowBattery = battery < LOW_BATTERY_THRESHOLD;

  // Treat a missing echo as unknown, not as a real distance.
  bool frontObstacle =
    (front >= 0 && front <= OBSTACLE_DISTANCE_CM);

  bool leftObstacle =
    (left >= 0 && left <= OBSTACLE_DISTANCE_CM);

  bool rightObstacle =
    (right >= 0 && right <= OBSTACLE_DISTANCE_CM);

  // --------------------------------------------------------
  // E. Priority 1: Cliff detected.
  // --------------------------------------------------------

  if (cliffDetected) {
    stopMotors();

    tone(BUZZER_PIN, 1800, 200);

    setStatusColor(100, 0, 100);

    robotState = "CLIFF DETECTED - STOP";

  }

  // --------------------------------------------------------
  // F. Priority 2: Low simulated battery.
  // --------------------------------------------------------

  else if (lowBattery) {
    stopMotors();

    tone(BUZZER_PIN, 900, 200);

    setStatusColor(100, 35, 0);

    robotState = "LOW BATTERY - STOP";

  }

  // --------------------------------------------------------
  // G. Continue a turn for the configured duration.
  // --------------------------------------------------------

  else if (turning && millis() - turnStarted < TURN_TIME_MS) {
    robotState = "AVOIDING OBSTACLE - TURNING";
  }

  // --------------------------------------------------------
  // H. Obstacle avoidance.
  // --------------------------------------------------------

  else {
    turning = false;

    // Front obstacle: choose the clearer side.
    if (frontObstacle) {
      tone(BUZZER_PIN, 1200, 120);

      bool leftClear = (left < 0) || (left > OBSTACLE_DISTANCE_CM);
      bool rightClear = (right < 0) || (right > OBSTACLE_DISTANCE_CM);

      if (leftClear && (!rightClear || left >= right)) {
        turnLeft();
        turnStarted = millis();

        setStatusColor(0, 0, 100);

        robotState = "OBSTACLE FRONT - TURN LEFT";
      }

      else if (rightClear) {
        turnRight();
        turnStarted = millis();

        setStatusColor(100, 50, 0);

        robotState = "OBSTACLE FRONT - TURN RIGHT";
      }

      else {
        stopMotors();

        setStatusColor(100, 0, 0);

        robotState = "PATH BLOCKED - STOP";
      }
    }

    // Obstacle on the left: steer right.
    else if (leftObstacle) {
      turnRight();
      turnStarted = millis();

      setStatusColor(100, 50, 0);

      robotState = "OBSTACLE LEFT - TURN RIGHT";
    }

    // Obstacle on the right: steer left.
    else if (rightObstacle) {
      turnLeft();
      turnStarted = millis();

      setStatusColor(0, 0, 100);

      robotState = "OBSTACLE RIGHT - TURN LEFT";
    }

    // No detected obstacle: drive forward.
    else {
      moveForward(NORMAL_SPEED);

      setStatusColor(0, 100, 0);

      robotState = "PATH CLEAR - MOVING FORWARD";
    }
  }

  // --------------------------------------------------------
  // I. Print a neat status report once per second.
  // --------------------------------------------------------

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    printStatus(
      front,
      left,
      right,
      cliffL,
      cliffR,
      battery,
      gz
    );
  }

  delay(40);
}