// This is the 3D maze game for the color block 2.0

#include "neopixel_leds.h"
#include <XBee.h>
#include <SoftwareSerial.h>
#include "pitches.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// hardware connections
#define SPEAKER 3
#define VIBR_MOTOR 5

NeopixelLeds neopixelleds;
MPU6050 accelgyro;

unsigned long timer_last, timer_now;
const int PATTERN_PERIOD = 20000;

void setup() {
    // mpu6050
    Serial.begin(9600);  // print out test message
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // vibration motor
    // fade in from min to max in increments of 5 points:
    for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
        // sets the value (range from 0 to 255):
        analogWrite(VIBR_MOTOR, fadeValue);
        delay(30);
    }
    // fade out from max to min in increments of 5 points:
    for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
        // sets the value (range from 0 to 255):
        analogWrite(VIBR_MOTOR, fadeValue);
        delay(30);
    }
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
    // delay before next reading:
    delay(100);

}




