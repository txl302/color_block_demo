// This is the 3D maze game for the color block 2.0

#include "neopixel_leds.h"
#include "pitches.h"
#include "Wire.h"
#include "mpu6050lib.h"

// hardware connections
#define SPEAKER 3
#define VIBR_MOTOR 5

NeopixelLeds neopixelleds;





unsigned long timer_last, timer_now;
const int PATTERN_PERIOD = 20000;

void setup() {
    Serial.begin(9600);  // print test message
    // mpu6050
    Wire.begin();  // join I2C bus
    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu6050.initialize();
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
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
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
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




