#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

Servo ESC1; // Medurs1 D8
Servo ESC2; // Medurs2 D7
Servo ESC3; // Moturs1 D6
Servo ESC4; // Moturs2 D5

MPU6050 mpu; // Create an MPU6050 object

void setup()
{
    Serial.begin(9600);

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

    ESC1.attach(D8, 1000, 2000); // Attach ESC1 signal wire to pin D8 with min and max pulse width
    ESC2.attach(D7, 1000, 2000); // Attach ESC2 signal wire to pin D7 with min and max pulse width
    ESC3.attach(D6, 1000, 2000); // Attach ESC3 signal wire to pin D6 with min and max pulse width
    ESC4.attach(D5, 1000, 2000); // Attach ESC4 signal wire to pin D5 with min and max pulse width
    // Set throttle to minimum to arm ESCs
    ESC1.write(100);
    ESC2.write(100);
    ESC3.write(100);
    ESC4.write(100);
    delay(2000);

    // Set throttle to low
    ESC1.write(10);
    ESC2.write(10);
    ESC3.write(10);
    ESC4.write(10);
    delay(2000);
}

void loop()
{
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
    // Medurs 1 & 2
    ESC1.write(50);
    ESC2.write(50);
    // Moturs 1& 2
    ESC3.write(50);
    ESC4.write(50);

    delay(200); // Delay for stability
}
