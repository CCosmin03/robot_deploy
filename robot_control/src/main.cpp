#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#include <ros.h>
#include <robot_msgs/BatteryStatus.h>
#include <robot_msgs/EncoderData.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

ros::NodeHandle nh;
Adafruit_INA219 ina219(0x45);
const int BATTERY_TABLE_SIZE = 16;

const float batteryVoltageTable[BATTERY_TABLE_SIZE] = {
  13.20f, 14.00f, 14.40f, 14.80f,
  15.00f, 15.16f, 15.28f, 15.40f,
  15.52f, 15.64f, 15.84f, 15.96f,
  16.12f, 16.32f, 16.44f, 16.80f
};

const float batteryPercentTable[BATTERY_TABLE_SIZE] = {
  0.0f, 10.0f, 20.0f, 30.0f,
  40.0f, 45.0f, 50.0f, 55.0f,
  60.0f, 65.0f, 70.0f, 75.0f,
  80.0f, 85.0f, 90.0f, 100.0f
};

const float BATTERY_SHUTDOWN_VOLTAGE = 14.4f;
const float BATTERY_SHUTDOWN_PERCENT = 20.0f;

float batteryVoltage = 0.0f;
float batteryCurrent_mA = 0.0f;
float batteryPower_mW = 0.0f;
float batteryPercent = 0.0f;

bool lowBatteryWarning = false;

// PINOUT
// BTS7960 stanga
const uint8_t L_RPWM = 6;
const uint8_t L_LPWM = 7;
const uint8_t L_REN  = 22;
const uint8_t L_LEN  = 23;

// BTS7960 dreapta
const uint8_t R_RPWM = 9;
const uint8_t R_LPWM = 8;
const uint8_t R_REN  = 25;
const uint8_t R_LEN  = 24;

// Current sense BTS7960
const uint8_t L_RIS = A0;
const uint8_t L_LIS = A1;
const uint8_t R_RIS = A2;
const uint8_t R_LIS = A3;

// Encodere
const uint8_t ENC_L_A = 2;
const uint8_t ENC_L_B = 3;
const uint8_t ENC_R_A = 18;
const uint8_t ENC_R_B = 19;

// Beeper
const uint8_t BEEPER_PIN = 31;

volatile long leftEncoderTicks  = 0;
volatile long rightEncoderTicks = 0;

float leftVelTarget    = 0.0f;
float rightVelTarget   = 0.0f;
float leftVelMeasured  = 0.0f;
float rightVelMeasured = 0.0f;
float leftIntegral     = 0.0f;
float rightIntegral    = 0.0f;
long  prevLeftTicks    = 0;
long  prevRightTicks   = 0;
unsigned long lastCtrlMs = 0;

const float WHEEL_BASE_M   = 0.2025f;
const float WHEEL_RADIUS_M = 0.03208f;
const int   ENCODER_CPR    = 11800;     
const float MAX_LINEAR_X  = 0.09f;  // 85% din max masurat (108 mm/s la PWM=255)
const float MAX_ANGULAR_Z = 2.0f * MAX_LINEAR_X / WHEEL_BASE_M;

// Test mode 1: PWM fix — calibrare FF_GAIN
//#define TEST_PWM_MODE
const int   TEST_PWM = 255;  // max PWM pt masurarea vitezei maxime reale

// Test mode 2: viteza fix prin controller — tuning Kp/Ki (comenteaza TEST_PWM_MODE)
//#define TEST_CTRL_MODE
const float TEST_VEL = 0.05f;  // m/s

// Test mode 3: verificare CPR — roteste roata o tura si citeste delta ticks
//#define TEST_CPR_MODE

// Controller FF + PI
const float FF_GAIN            = 2488.0f;  // medie (2451+2525)/2
const float CTRL_KP            = 1200.0f;
const float CTRL_KI            = 500.0f;  // start conservativ; creste spre 1000-2000 daca eroarea nu converge
const float CTRL_INTEGRAL_MAX  = 0.1f;   // Ki * max = 50 PWM units; creste proportional cu Ki
const unsigned long CTRL_INTERVAL_MS = 20;

unsigned long lastCmdVelMs = 0;
const unsigned long CMD_VEL_TIMEOUT_MS = 500;
bool teleopActive = false;

unsigned long lastBeepTime = 0;
const unsigned long BEEP_INTERVAL = 500;
const unsigned int BEEP_FREQUENCY = 3000;
bool beeperState = false;

robot_msgs::EncoderData encoder_msg;
ros::Publisher encoder_pub("encoder_data", &encoder_msg);

robot_msgs::BatteryStatus battery_msg;
ros::Publisher battery_pub("battery_status", &battery_msg);

unsigned long lastROSPublishTime = 0;
const unsigned long ROS_PUBLISH_INTERVAL = 50;

void change_left_a();
void change_left_b();
void change_right_a();
void change_right_b();

void readEncoderTicks(long &leftTicks, long &rightTicks);

void enableDrivers(bool on);
void stopMotors();
void setMotorLeft(int pwm);
void setMotorRight(int pwm);
void drive(int leftPwm, int rightPwm);

void beep(bool on);
float calculateBatteryPercent(float voltage);
void updateBatteryStatus();
void manageLowBattery(unsigned long now);
void sendROSData();

void computeWheelVelocities(float dt);
void runMotorControllers(float dt);
void cmdVelCb(const geometry_msgs::Twist& cmd);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCb);
void enableDrivers(bool on)
{
  digitalWrite(L_REN, on ? HIGH : LOW);
  digitalWrite(L_LEN, on ? HIGH : LOW);
  digitalWrite(R_REN, on ? HIGH : LOW);
  digitalWrite(R_LEN, on ? HIGH : LOW);
}

void stopMotors()
{
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
}

void setMotorLeft(int pwm)
{
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(L_RPWM, pwm);
    analogWrite(L_LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, -pwm);
  } else {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, 0);
  }
}

void setMotorRight(int pwm)
{
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(R_RPWM, pwm);
    analogWrite(R_LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, -pwm);
  } else {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, 0);
  }
}

void drive(int leftPwm, int rightPwm)
{
  setMotorLeft(leftPwm);
  setMotorRight(rightPwm);
}

// ================== BEEPER ==================
void beep(bool on)
{
  if (on) {
    tone(BEEPER_PIN, BEEP_FREQUENCY);
  } else {
    noTone(BEEPER_PIN);
  }
}

// ================== BATERIE ==================
float calculateBatteryPercent(float voltage)
{
  if (voltage <= batteryVoltageTable[0]) {
    return batteryPercentTable[0];
  }

  if (voltage >= batteryVoltageTable[BATTERY_TABLE_SIZE - 1]) {
    return batteryPercentTable[BATTERY_TABLE_SIZE - 1];
  }

  for (int i = 0; i < BATTERY_TABLE_SIZE - 1; i++) {
    float v1 = batteryVoltageTable[i];
    float v2 = batteryVoltageTable[i + 1];

    if (voltage >= v1 && voltage <= v2) {
      float p1 = batteryPercentTable[i];
      float p2 = batteryPercentTable[i + 1];

      float ratio = (voltage - v1) / (v2 - v1);
      return p1 + ratio * (p2 - p1);
    }
  }

  return 0.0f;
}

void updateBatteryStatus()
{
  float busVoltage = ina219.getBusVoltage_V();
  float shuntVoltage_mV = ina219.getShuntVoltage_mV();

  batteryVoltage = busVoltage + (shuntVoltage_mV / 1000.0f);
  batteryCurrent_mA = ina219.getCurrent_mA();
  batteryPower_mW = ina219.getPower_mW();

  batteryPercent = calculateBatteryPercent(batteryVoltage);

  lowBatteryWarning =
    (batteryVoltage <= BATTERY_SHUTDOWN_VOLTAGE) ||
    (batteryPercent <= BATTERY_SHUTDOWN_PERCENT);
}

void manageLowBattery(unsigned long now)
{
  if (!lowBatteryWarning) {
    beeperState = false;
    beep(false);
    return;
  }

  stopMotors();
  enableDrivers(false);

  if (now - lastBeepTime >= BEEP_INTERVAL) {
    lastBeepTime = now;
    beeperState = !beeperState;
    beep(beeperState);
  }
}

// ================== TELEOP ==================
void cmdVelCb(const geometry_msgs::Twist& cmd)
{
  if (lowBatteryWarning) {
    stopMotors();
    return;
  }

  enableDrivers(true);

  lastCmdVelMs = millis();
  teleopActive = true;

  float linear = constrain(cmd.linear.x, -MAX_LINEAR_X, MAX_LINEAR_X);
  float angular = constrain(cmd.angular.z, -MAX_ANGULAR_Z, MAX_ANGULAR_Z);

  leftVelTarget  = linear - angular * WHEEL_BASE_M / 2.0f;
  rightVelTarget = linear + angular * WHEEL_BASE_M / 2.0f;
}

void computeWheelVelocities(float dt)
{
  long leftTicks, rightTicks;
  readEncoderTicks(leftTicks, rightTicks);

  float ticksToMs = (2.0f * PI * WHEEL_RADIUS_M) / ((float)ENCODER_CPR * dt);
  leftVelMeasured  = (float)(leftTicks  - prevLeftTicks)  * ticksToMs;
  rightVelMeasured = (float)(rightTicks - prevRightTicks) * ticksToMs;

  prevLeftTicks  = leftTicks;
  prevRightTicks = rightTicks;
}

void runMotorControllers(float dt)
{
  // Left motor
  float errL = leftVelTarget - leftVelMeasured;
  if (fabs(leftVelTarget) < 0.01f) {
    leftIntegral = 0.0f;
  } else {
    leftIntegral = constrain(leftIntegral + errL * dt, -CTRL_INTEGRAL_MAX, CTRL_INTEGRAL_MAX);
  }
  float outL = FF_GAIN * leftVelTarget + CTRL_KP * errL + CTRL_KI * leftIntegral;
  setMotorLeft(constrain((int)outL, -255, 255));

  // Right motor
  float errR = rightVelTarget - rightVelMeasured;
  if (fabs(rightVelTarget) < 0.01f) {
    rightIntegral = 0.0f;
  } else {
    rightIntegral = constrain(rightIntegral + errR * dt, -CTRL_INTEGRAL_MAX, CTRL_INTEGRAL_MAX);
  }
  float outR = FF_GAIN * rightVelTarget + CTRL_KP * errR + CTRL_KI * rightIntegral;
  setMotorRight(constrain((int)outR, -255, 255));
}

// ================== ROS PUBLISH ==================
void sendROSData()
{
  unsigned long now = millis();

  if (now - lastROSPublishTime < ROS_PUBLISH_INTERVAL) {
    return;
  }

  lastROSPublishTime = now;

  long leftTicks, rightTicks;
  readEncoderTicks(leftTicks, rightTicks);

  encoder_msg.header.stamp = nh.now();
  encoder_msg.left_ticks  = leftTicks;
  encoder_msg.right_ticks = rightTicks;
  encoder_pub.publish(&encoder_msg);

  battery_msg.header.stamp  = nh.now();
  battery_msg.voltage_V        = batteryVoltage;
  battery_msg.current_A     = batteryCurrent_mA / 1000.0f;
  battery_msg.power_W       = batteryPower_mW   / 1000.0f;
  battery_msg.percent        = batteryPercent;
  battery_msg.low_warning    = lowBatteryWarning;
  battery_pub.publish(&battery_msg);
}

// ================== SETUP ==================
void setup()
{
  nh.getHardware()->setBaud(115200);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  pinMode(L_RIS, INPUT);
  pinMode(L_LIS, INPUT);
  pinMode(R_RIS, INPUT);
  pinMode(R_LIS, INPUT);

  pinMode(BEEPER_PIN, OUTPUT);
  beep(false);

  stopMotors();
  enableDrivers(true);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), change_right_b, CHANGE);

  Wire.begin();

  nh.initNode();

  nh.advertise(encoder_pub);
  nh.advertise(battery_pub);

  nh.subscribe(cmd_vel_sub);

  if (!ina219.begin()) {
    nh.logerror("INA219 not detected. Motors disabled.");
    stopMotors();
    enableDrivers(false);

    while (1) {
      nh.spinOnce();
      delay(100);
    }
  }

  lastCtrlMs = millis();
  nh.loginfo("Arduino base controller ready.");
}

// ================== LOOP ==================
void loop()
{
  unsigned long now = millis();

  updateBatteryStatus();
  manageLowBattery(now);

  if (lowBatteryWarning) {
    sendROSData();
    nh.spinOnce();
    delay(5);
    return;
  }

  if (teleopActive && (now - lastCmdVelMs > CMD_VEL_TIMEOUT_MS)) {
    leftVelTarget  = 0.0f;
    rightVelTarget = 0.0f;
    leftIntegral   = 0.0f;
    rightIntegral  = 0.0f;
    stopMotors();
    teleopActive = false;
  }

  if (now - lastCtrlMs >= CTRL_INTERVAL_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;
    computeWheelVelocities(dt);

#if defined(TEST_CPR_MODE)
    enableDrivers(true);
    drive(TEST_PWM, TEST_PWM);
    static unsigned long lastLogMs = 0;
    if (now - lastLogMs >= 500) {
      lastLogMs = now;
      static char logBuf[48];
      long lT, rT;
      readEncoderTicks(lT, rT);
      snprintf(logBuf, sizeof(logBuf), "L=%ld R=%ld", lT, rT);
      nh.loginfo(logBuf);
    }
#elif defined(TEST_PWM_MODE)
    enableDrivers(true);
    drive(TEST_PWM, TEST_PWM);
    static unsigned long lastLogMsPwm = 0;
    if (now - lastLogMsPwm >= 500) {
      lastLogMsPwm = now;
      static char logBuf[48];
      snprintf(logBuf, sizeof(logBuf), "L=%d R=%d mm/s",
               (int)(leftVelMeasured * 1000),
               (int)(rightVelMeasured * 1000));
      nh.loginfo(logBuf);
    }
#elif defined(TEST_CTRL_MODE)
    enableDrivers(true);
    leftVelTarget  = TEST_VEL;
    rightVelTarget = TEST_VEL;
    runMotorControllers(dt);
    static unsigned long lastLogMsCtrl = 0;
    if (now - lastLogMsCtrl >= 500) {
      lastLogMsCtrl = now;
      static char logBuf[80];
      snprintf(logBuf, sizeof(logBuf), "T=%d L=%d R=%d eL=%d eR=%d mm/s",
               (int)(TEST_VEL * 1000),
               (int)(leftVelMeasured * 1000),
               (int)(rightVelMeasured * 1000),
               (int)((leftVelTarget - leftVelMeasured) * 1000),
               (int)((rightVelTarget - rightVelMeasured) * 1000));
      nh.loginfo(logBuf);
    }
#else
    if (teleopActive) {
      runMotorControllers(dt);
    }
#endif
  }

  sendROSData();
  nh.spinOnce();

  delay(5);
}

// ================== ENCODERE ==================
void readEncoderTicks(long &leftTicks, long &rightTicks)
{
  noInterrupts();
  leftTicks = leftEncoderTicks;
  rightTicks = rightEncoderTicks;
  interrupts();
}

void change_left_a()
{
  if (digitalRead(ENC_L_A) == HIGH) {
    if (digitalRead(ENC_L_B) == LOW) {
      leftEncoderTicks++;
    } else {
      leftEncoderTicks--;
    }
  } else {
    if (digitalRead(ENC_L_B) == HIGH) {
      leftEncoderTicks++;
    } else {
      leftEncoderTicks--;
    }
  }
}

void change_left_b()
{
  if (digitalRead(ENC_L_B) == HIGH) {
    if (digitalRead(ENC_L_A) == HIGH) {
      leftEncoderTicks++;
    } else {
      leftEncoderTicks--;
    }
  } else {
    if (digitalRead(ENC_L_A) == LOW) {
      leftEncoderTicks++;
    } else {
      leftEncoderTicks--;
    }
  }
}

void change_right_a()
{
  if (digitalRead(ENC_R_A) == HIGH) {
    if (digitalRead(ENC_R_B) == LOW) {
      rightEncoderTicks--;
    } else {
      rightEncoderTicks++;
    }
  } else {
    if (digitalRead(ENC_R_B) == HIGH) {
      rightEncoderTicks--;
    } else {
      rightEncoderTicks++;
    }
  }
}

void change_right_b()
{
  if (digitalRead(ENC_R_B) == HIGH) {
    if (digitalRead(ENC_R_A) == HIGH) {
      rightEncoderTicks--;
    } else {
      rightEncoderTicks++;
    }
  } else {
    if (digitalRead(ENC_R_A) == LOW) {
      rightEncoderTicks--;
    } else {
      rightEncoderTicks++;
    }
  }
}