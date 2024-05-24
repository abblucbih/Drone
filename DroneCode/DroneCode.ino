#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <math.h>
// you can enable debug logging to Serial at 115200
// #define REMOTEXY__DEBUGLOG

// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT

#include <ESP8266WiFi.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

#include <RemoteXY.h>

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] = // 54 bytes
    {255, 5, 0, 0, 0, 47, 0, 17, 0, 0, 0, 31, 1, 200, 80, 1, 1, 3, 0, 2,
     77, 37, 44, 22, 0, 2, 26, 31, 31, 79, 78, 0, 79, 70, 70, 0, 5, 139, 8, 60,
     60, 0, 2, 26, 31, 5, 3, 8, 60, 60, 0, 2, 26, 31};

// this structure defines all the variables and events of your control interface
struct
{

  // input variables
  uint8_t switch_01;    // =1 if switch ON and =0 if OFF
  int8_t joystick_01_x; // from -100 to 100
  int8_t joystick_01_y; // from -100 to 100
  int8_t joystick_02_x; // from -100 to 100
  int8_t joystick_02_y; // from -100 to 100

  // other variable
  uint8_t connect_flag; // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

// Servo objects for motors
Servo ESC1; // Left front motor (counter-clockwise) D8
Servo ESC2; // Right front motor (clockwise) D7
Servo ESC3; // Left back motor (clockwise) D6
Servo ESC4; // Right back motor (counter-clockwise) D5

MPU6050 mpu; // Create an MPU6050 object

// PID constants
float Kp = 0.5;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.01; // Derivative gain

// Variables to store angle data
float roll = 0.0, pitch = 0.0;
float rollErrorSum = 0.0, pitchErrorSum = 0.0;
float lastRollError = 0.0, lastPitchError = 0.0;
unsigned long lastTime = 0;

// Offset values for calibration
float ax_offset = 0.0, ay_offset = 0.0, az_offset = 0.0;
float gx_offset = 0.0, gy_offset = 0.0, gz_offset = 0.0;

void setup()
{
  Serial.begin(9600);
  RemoteXY_Init();
  // Initialize I2C communication
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();

  // Verify connection
  if (mpu.testConnection())
  {
    Serial.println("MPU6050 connection successful");
  }
  else
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ; // Stop here if connection failed
  }

  // Attach ESC signal wires to pins with min and max pulse width
  ESC1.attach(D7, 1000, 2000); // Left front motor
  ESC2.attach(D5, 1000, 2000); // Right front motor
  ESC3.attach(D8, 1000, 2000); // Left back motor
  ESC4.attach(D6, 1000, 2000); // Right back motor

  // Set throttle to minimum to arm ESCs
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  delay(2000);

  // Set throttle to low
  ESC1.write(10);
  ESC2.write(10);
  ESC3.write(10);
  ESC4.write(10);
  delay(2000);

  // Calibrate the MPU6050 by calculating offsets
  calibrateMPU6050();
}

void loop()
{
  unsigned long currentTime = millis();
  RemoteXY_Handler();
  float dt = (currentTime - lastTime) / 1000.0; // Delta time in seconds
  lastTime = currentTime;

  // Read gyro and accelerometer data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply offsets
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // Convert raw data to Gs and degrees/s
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  float gx_dps = gx / 131.0;
  float gy_dps = gy / 131.0;
  float gz_dps = gz / 131.0;

  // Calculate roll and pitch from accelerometer
  float accel_roll = atan2(ay_g, az_g) * 180 / PI;
  float accel_pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 180 / PI;

  // Integrate gyroscope data to calculate roll and pitch
  roll += gx_dps * dt;
  pitch += gy_dps * dt;

  // Complementary filter
  float alpha = 0.98;
  roll = alpha * roll + (1 - alpha) * accel_roll;
  pitch = alpha * pitch + (1 - alpha) * accel_pitch;

  // PID calculations for roll
  float rollError = 0 - roll; // Desired roll is 0 (level)
  rollErrorSum += rollError * dt;
  float rollErrorRate = (rollError - lastRollError) / dt;
  float rollOutput = Kp * rollError + Ki * rollErrorSum + Kd * rollErrorRate;
  lastRollError = rollError;

  // PID calculations for pitch
  float pitchError = 0 - pitch; // Desired pitch is 0 (level)
  pitchErrorSum += pitchError * dt;
  float pitchErrorRate = (pitchError - lastPitchError) / dt;
  float pitchOutput = Kp * pitchError + Ki * pitchErrorSum + Kd * pitchErrorRate;
  lastPitchError = pitchError;
  int joystickInput = map(RemoteXY.joystick_01_y, 0, 100, 0, 180);

  // Adjust motor speeds based on PID output
  int motorSpeed1 = constrain(joystickInput + rollOutput - pitchOutput, 0, 100); // Left front motor (counter-clockwise)
  int motorSpeed2 = constrain(joystickInput - rollOutput - pitchOutput, 0, 100); // Right front motor (clockwise)
  int motorSpeed3 = constrain(joystickInput + rollOutput + pitchOutput, 0, 100); // Left back motor (clockwise)
  int motorSpeed4 = constrain(joystickInput - rollOutput + pitchOutput, 0, 100); // Right back motor (counter-clockwise)

  // Set motor speeds also only if switch is on
  if (RemoteXY.switch_01 == 1)
  {
    ESC1.write(motorSpeed1);
    ESC2.write(motorSpeed2);
    ESC3.write(motorSpeed3);
    ESC4.write(motorSpeed4);
  }
  else
  {
    ESC1.write(0);
    ESC2.write(0);
    ESC3.write(0);
    ESC4.write(0);
  }

  // Print angles and motor speeds for debugging
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Motor Speeds: ");
  Serial.print(motorSpeed1);
  Serial.print(", ");
  Serial.print(motorSpeed2);
  Serial.print(", ");
  Serial.print(motorSpeed3);
  Serial.print(", ");
  Serial.println(motorSpeed4);
  Serial.println(RemoteXY.joystick_01_y);

  RemoteXY_delay(20); // Short delay for stability
}

void calibrateMPU6050()
{
  const int numReadings = 1000;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < numReadings; i++)
  {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    RemoteXY_delay(2);
  }

  ax_offset = ax_sum / numReadings;
  ay_offset = ay_sum / numReadings;
  az_offset = az_sum / numReadings;
  gx_offset = gx_sum / numReadings;
  gy_offset = gy_sum / numReadings;
  gz_offset = gz_sum / numReadings;

  // Adjust accelerometer offset to account for gravity (assuming the sensor is flat)
  az_offset -= 16384;
}
