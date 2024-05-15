#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

Servo ESC1;
Servo ESC2; // Create servo objects to control the ESCs

MPU6050 mpu; // Create an MPU6050 object

void setup() {
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();

  // Verify connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1); // Stop here if connection failed
  }

  ESC1.attach(D8, 1000, 2000); // Attach ESC1 signal wire to pin D8 with min and max pulse width
  ESC2.attach(D7, 1000, 2000); // Attach ESC2 signal wire to pin D7 with min and max pulse width

  // Set throttle to minimum to arm ESCs
  ESC1.write(100); 
  ESC2.write(100); 
  delay(2000);
  
  // Set throttle to low
  ESC1.write(10);
  ESC2.write(10); 
  delay(2000);
}

void loop() {
  // Read gyro and accelerometer data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print sensor data
  Serial.print("Accelerometer: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay);
  Serial.print(", ");
  Serial.println(az);

  Serial.print("Gyroscope: ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);

  // Set throttle for ESCs
  ESC1.write(50); // Example throttle setting, adjust as needed
  ESC2.write(50); // Example throttle setting, adjust as needed
  
  delay(200); // Delay for stability
}
