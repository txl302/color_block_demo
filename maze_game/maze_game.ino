// This is the 3D maze game for the color block 2.0
// Author: Yang Liu, 04/20/2018.

// info about hardware orientation and indexing
// Cap is face 6, also the top of the block.
// The top surface of the circuit is facing face 3, so x of mpu6050 points to face 2, y
// points to face 6(top), z points to face 3.
// In the program, the face index is in range of [0,5].

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
#include "Adafruit_NeoPixel.h"

// hardware connections
#define SPEAKER 3
#define VIBR_MOTOR 5
#define MPU6050_ADDRESS 0x68
#define PIXEL_PIN 2
#define PIXEL_NUM 150

// MPU6050 variables
MPU6050 mpu6050;
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
float ax, ay, az;       // Stores the real accel value in g's
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
float gx, gy, gz;       // Stores the real gyro value in degrees per seconds
float aRes = 2.0/32768.0;
float gRes = 250.0/32768.0;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
float roll, pitch, yaw;  // in degrees
float accel_b[3];  // acceleration in body frame, gravity-compensated
// for quaternion update
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;
float p_vect[3];

// Neopixel variables
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(PIXEL_NUM, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t color_bright = pixels.Color(255,255,255);
uint32_t color_dark = pixels.Color(0,0,0);
// maze design: '1' lights up for wall, '0' is dark for free space
uint8_t MAZE[6][5][5] = {
    {{1,0,1,1,1},{1,0,1,0,1},{1,0,1,0,0},{0,0,1,0,1},{1,1,1,0,0}},
    {{0,0,0,0,0},{0,1,1,1,0},{0,0,0,1,0},{0,1,0,1,0},{0,1,0,1,0}},
    {{1,1,1,0,1},{1,0,1,0,1},{0,0,1,0,1},{1,0,0,0,1},{1,1,1,1,1}},
    {{0,1,0,0,0},{1,1,1,1,1},{0,0,0,0,0},{1,1,0,1,1},{0,1,0,0,0}},
    {{1,0,0,0,1},{1,0,1,1,1},{0,0,0,1,0},{1,1,1,1,1},{0,1,0,0,0}},
    {{1,0,0,0,0},{1,1,1,0,1},{1,0,0,0,1},{1,0,1,1,1},{1,0,0,0,0}}};
// positions on the block, indexed as {face, row, column}
uint8_t pos_player[3] = {5,2,2};  // also starting pos, at the middle of the top face
uint32_t color_player = pixels.Color(0,255,0);
uint8_t pos_des[3] = {4,2,2};  // destination is at the middle of the bottom face
uint32_t color_des = pixels.Color(255,0,0);

// block variables
uint8_t bottom_face[6] = {2,3,0,1,5,4};  // index is top face
uint8_t face_axis[6] = {2,0,2,0,1,1};  // axis that goes through the face
uint8_t last_valid = 0;  // whether last recorded shake acceleration value is valid
uint8_t last_top_face = 0;  // last top face index
float last_shake_accel;  // last shake acceleration value
uint8_t move_dir[6][6] = {{0,2,0,4,1,3},
                          {4,0,2,0,1,3},
                          {0,4,0,2,1,3},
                          {2,0,4,0,1,3},
                          {3,2,1,4,0,0},
                          {1,2,3,4,0,0}};
    // move direction of the player on that surface
    // first index is the face play is at, second index is the top index
    // '1' for moving up, '2' for right, '3' for down, '4' for left; '0' for invalid
uint8_t face_next[6][4] = {{5,3,4,1},
                           {5,0,4,2},
                           {5,1,4,3},
                           {5,2,4,0},
                           {0,3,2,1},
                           {2,3,0,1}};
    // next face when going the four direction from one face
uint8_t face_rot[6][6] = {{0,1,0,1,1,1},
                          {1,0,1,0,3,2},
                          {0,1,0,1,4,4},
                          {1,0,1,0,2,3},
                          {1,2,4,3,0,0},
                          {1,3,4,2,0,0}};
    // Whether the face needs to rotate to align the direction, when goes from one face
    // to the other. '0' for invalid, '1' for no rotation, '2' for rotate left 90 deg,
    // '3' for rotate right 90 deg, '4' for ratate 180 deg.
float shake_thres = 0.5;  // empirical threshold for shake detection

// flow control
unsigned long time_last, time_now;  // microsecond
unsigned long period = 20000;  // period for main loop
uint8_t vib_on = 0;  // vibration motor state
uint8_t vib_count_total = 10;  // duration is 10 times the loop period
uint8_t vib_count = 0;
uint8_t buz_on = 0;  // speaker state
uint8_t buz_count_total = 10;
uint8_t buz_count = 0;
uint8_t det_off = 0;  // whether shake detection is disabled
uint8_t det_count_total = 30;
uint8_t det_count = 0;
uint8_t game_win = 0;  // whether game is won
uint8_t win_count_total = 250;
uint8_t win_count = 0;
int rainbow_index = 0;
// debug print control
unsigned long debug_period = 100000;
uint8_t debug_freq = (uint8_t)((float)debug_period/period);
uint8_t debug_count = 0;

void setup() {
    Serial.begin(9600);  // print test message

    // mpu6050
    Wire.begin();  // join I2C bus
    calibrateGyro();
    mpu6050.initialize();

    // vibration motor, indicate block is powered on
    analogWrite(VIBR_MOTOR, 100);
    delay(200);
    analogWrite(VIBR_MOTOR, 0);

    // neopixels, light up the maze
    pixels.begin();
    int serial_index;
    uint8_t pos[3];
    for (uint8_t face=0; face<6; face++) {
        for (uint8_t row=0; row<5; row++) {
            for (uint8_t column=0; column<5; column++) {
                pos[0] = face; pos[1] = row; pos[2] = column;
                serial_index = pixel_indexing(pos);
                if (MAZE[face][row][column] == 1) {
                    pixels.setPixelColor(serial_index, color_bright);
                }
                else {
                    pixels.setPixelColor(serial_index, color_dark);
                }
            }
        }
    }
    serial_index = pixel_indexing(pos_player);
    pixels.setPixelColor(serial_index, color_player);
    serial_index = pixel_indexing(pos_des);
    pixels.setPixelColor(serial_index, color_des);
    pixels.show();

    time_last = micros();
}

void loop() {
    time_now = micros();
    if ((time_now - time_last) > period) {
        deltat = (time_now - time_last)/1000000.0f;
        time_last = time_now;

        // check if game is won and show the rainbow
        if (!game_win) {

            // check shake detection state
            if (det_off) {
                det_count = det_count + 1;
                if (det_count == det_count_total) {
                    det_off = 0;
                }
            }

            // check state of vibration motor
            if (vib_on) {
                vib_count = vib_count + 1;
                if (vib_count == vib_count_total) {
                    analogWrite(VIBR_MOTOR, 0);
                    vib_on = 0;
                }
            }

            // check state of speaker
            if (buz_on) {
                buz_count = buz_count + 1;
                if (buz_count == buz_count_total) {
                    noTone(SPEAKER);
                    buz_on = 0;
                }
            }

            // read raw accel/gyro measurements from device
            mpu6050.getMotion6(&accelCount[0], &accelCount[1], &accelCount[2],
                &gyroCount[0], &gyroCount[1], &gyroCount[2]);
            ax = (float)accelCount[0] * aRes;  // get actual g value
            ay = (float)accelCount[1] * aRes;
            az = (float)accelCount[2] * aRes;
            gx = (float)gyroCount[0] * gRes;  // get actual gyro value
            gy = (float)gyroCount[1] * gRes;
            gz = (float)gyroCount[2] * gRes;

            // quaternion update
            MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.f, gy*PI/180.f, gz*PI/180.f);
            // roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            // pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            // yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
            // roll *= 180.0f / PI;
            // pitch *= 180.0f / PI;
            // yaw *= 180.0f / PI; 

            // projection of pointing-up unit vector in inertial frame to body frame
            p_vect[0] = 2*(q[1]*q[3] - q[0]*q[2]);
            p_vect[1] = 2*(q[0]*q[1] + q[2]*q[3]);
            p_vect[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

            // the projection vector is also used to compensate gravity in body frame accel
            accel_b[0] = ax - p_vect[0];
            accel_b[1] = ay - p_vect[1];
            accel_b[2] = az - p_vect[2];

            // shake detection
            uint8_t shake_detected = 0;
            int8_t top_face = top_face_indexing(p_vect);
            if (!det_off) {
                uint8_t current_valid = 0;  // whether current shake acceleration is valid
                if ((top_face != -1) && (top_face != pos_player[0])
                    && (top_face != bottom_face[pos_player[0]])) current_valid = 1;
                if (last_valid && !current_valid) last_valid = 0;
                else if (!last_valid && current_valid) {
                    // record current shake acceleration as last values
                    last_valid = 1;
                    last_top_face = top_face;
                    last_shake_accel = accel_b[face_axis[pos_player[0]]];
                }
                else if (last_valid && current_valid) {
                    // usually top_face and last_top_face are equal if here, but just in case
                    if (top_face != last_top_face) {
                        // copy current values as last
                        last_valid = 1;
                        last_top_face = top_face;
                        last_shake_accel = accel_b[face_axis[pos_player[0]]];
                    }
                    else {
                        // compare shake acceleration in order to detect shake
                        float shake_accel = accel_b[face_axis[pos_player[0]]];
                        if (abs(shake_accel - last_shake_accel) > shake_thres) {
                            shake_detected = 1;  // a shake is detected
                            last_valid = 0;
                            det_off = 1;
                            det_count = 0;
                        }
                    }
                }
            }

            // either the player moves one step down, or hit the wall
            if (shake_detected) {
                uint8_t move_dir_curr = move_dir[pos_player[0]][top_face];
                if (move_dir_curr) {  // should never be 0
                    // find next pos for the player
                    uint8_t next_pos[3];
                    cal_next_pos(pos_player, move_dir_curr, next_pos);
                    Serial.print(move_dir_curr); Serial.print("\t");
                    Serial.print(next_pos[0]); Serial.print("\t");
                    Serial.print(next_pos[1]); Serial.print("\t");
                    Serial.println(next_pos[2]);
                    if (MAZE[next_pos[0]][next_pos[1]][next_pos[2]]) {
                        // hit a wall, vibrate for a short time
                        analogWrite(VIBR_MOTOR, 100);
                        vib_on = 1;
                        vib_count = 0;
                    }
                    else {
                        // next pos is free space, update the neopixel display
                        int serial_index;
                        serial_index = pixel_indexing(pos_player);
                        pixels.setPixelColor(serial_index, color_dark);
                        // update the new position
                        pos_player[0] = next_pos[0];
                        pos_player[1] = next_pos[1];
                        pos_player[2] = next_pos[2];
                        serial_index = pixel_indexing(pos_player);
                        pixels.setPixelColor(serial_index, color_player);
                        pixels.show();
                        // check if game is won
                        if ((pos_player[0] == pos_des[0]) && (pos_player[1] == pos_des[1])
                            && (pos_player[2] == pos_des[2])) {
                            game_win = 1;
                            win_count = 0;
                        }
                        else {
                            // buzz for a short time for noticing
                            tone(SPEAKER, 262);
                            buz_on = 1;
                            buz_count = 0;
                            }

                    }
                }
            }

    //        // debug print
    //        debug_count = debug_count + 1;
    //        if (debug_count > debug_freq) {
    //            debug_count = 0;
    //            // Serial.print(ax); Serial.print("\t");
    //            // Serial.print(ay); Serial.print("\t");
    //            // Serial.print(az); Serial.print("\t");
    //            // Serial.print(gx); Serial.print("\t");
    //            // Serial.print(gy); Serial.print("\t");
    //            // Serial.println(gz);
    //            // Serial.print(accel_b[0]); Serial.print("\t");
    //            // Serial.print(accel_b[1]); Serial.print("\t");
    //            // Serial.print(accel_b[2]); Serial.print("\t");
    //            // Serial.print(roll); Serial.print("\t");
    //            // Serial.print(pitch); Serial.print("\t");
    //            // Serial.println(yaw);
    //            // Serial.print(top_face); Serial.print("\t");
    //            // Serial.print(p_vect[0]); Serial.print("\t");
    //            // Serial.print(p_vect[1]); Serial.print("\t");
    //            // Serial.println(p_vect[2]);
    //        }
        }
        else {
            win_count = win_count + 1;
            if (win_count == win_count_total) {
                // start a new game
                game_win = 0;
                int serial_index;
                uint8_t pos[3];
                for (uint8_t face=0; face<6; face++) {
                    for (uint8_t row=0; row<5; row++) {
                        for (uint8_t column=0; column<5; column++) {
                            pos[0] = face; pos[1] = row; pos[2] = column;
                            serial_index = pixel_indexing(pos);
                            if (MAZE[face][row][column] == 1) {
                                pixels.setPixelColor(serial_index, color_bright);
                            }
                            else {
                                pixels.setPixelColor(serial_index, color_dark);
                            }
                        }
                    }
                }
                pos_player[0] = 5; pos_player[1] = 2; pos_player[2] = 2;
                serial_index = pixel_indexing(pos_player);
                pixels.setPixelColor(serial_index, color_player);
                serial_index = pixel_indexing(pos_des);
                pixels.setPixelColor(serial_index, color_des);
                pixels.show();
            }
            else {
                // show the rainbow
                int light_index = 0;
                int serial_index;
                uint8_t pos[3];
                for (uint8_t face=0; face<6; face++) {
                    for (uint8_t row=0; row<5; row++) {
                        for (uint8_t column=0; column<5; column++) {
                            pos[0] = face; pos[1] = row; pos[2] = column;
                            serial_index = pixel_indexing(pos);
                            if (MAZE[face][row][column] == 1) {
                                pixels.setPixelColor(serial_index,
                                    pixel_wheel(((light_index * 256 /25) + rainbow_index) & 255));
                                light_index = (light_index + 1)%25;
                            }
                            else {
                                pixels.setPixelColor(serial_index, color_dark);
                            }
                        }
                    }
                }
                serial_index = pixel_indexing(pos_des);
                pixels.setPixelColor(serial_index, color_player);
                pixels.show();
                rainbow_index = rainbow_index + 5;
            }
        }
    }
}

// return the index of current top surface(0~5), return -1 if unambiguous
int8_t top_face_indexing(float *p_vect) {
    uint8_t max_index;  // index of p_vect that has largest absolute value
    if (abs(p_vect[0]) > abs(p_vect[1])) {
        if (abs(p_vect[0]) > abs(p_vect[2])) max_index = 0;
        else max_index = 2;
    }
    else {
        if (abs(p_vect[1]) > abs(p_vect[2])) max_index = 1;
        else max_index = 2;
    }
    if (abs(p_vect[max_index]) > 0.8) {  // empirical threshold 
        switch (max_index) {
            case 0:
                if (p_vect[max_index] > 0) return 1;
                else return 3;
            case 1:
                if (p_vect[max_index] > 0) return 5;
                else return 4;
            case 2:
                if (p_vect[max_index] > 0) return 2;
                else return 0;
        }
    }
    else return -1;
}

// find next pos of the player, given current pos and move direction
void cal_next_pos(uint8_t *current_pos, uint8_t direction, uint8_t *next_pos) {
    uint8_t face = current_pos[0];
    uint8_t row = current_pos[1];
    uint8_t column = current_pos[2];
    // copy current pos to next pos
    next_pos[0] = current_pos[0];
    next_pos[1] = current_pos[1];
    next_pos[2] = current_pos[2];
    if ((row == 0) && (direction == 1)) {
        // player position goes across the top
        uint8_t t_face_next = face_next[face][direction-1];  // t for transition
        uint8_t t_face_rot = face_rot[face][t_face_next];
        next_pos[0] = t_face_next;
        next_pos[1] = 4;
        pos_face_rot(next_pos, t_face_rot);
    }
    else if ((row == 4) && (direction == 3)) {
        // player position goes across the bottom
        uint8_t t_face_next = face_next[face][direction-1];
        uint8_t t_face_rot = face_rot[face][t_face_next];
        next_pos[0] = t_face_next;
        next_pos[1] = 0;
        pos_face_rot(next_pos, t_face_rot);
    }
    else if ((column == 0) && (direction == 4)) {
        // player position goes across the left
        uint8_t t_face_next = face_next[face][direction-1];
        uint8_t t_face_rot = face_rot[face][t_face_next];
        next_pos[0] = t_face_next;
        next_pos[2] = 4;
        pos_face_rot(next_pos, t_face_rot);
    }
    else if ((column == 4) && (direction == 2)) {
        // player position goes across the right
        uint8_t t_face_next = face_next[face][direction-1];
        uint8_t t_face_rot = face_rot[face][t_face_next];
        next_pos[0] = t_face_next;
        next_pos[2] = 0;
        pos_face_rot(next_pos, t_face_rot);
    }
    else {
        // player moves inside the face
        switch (direction) {
            case 1:  // move one step up
                next_pos[1] = next_pos[1] - 1;
                break;
            case 2:  // move one step right
                next_pos[2] = next_pos[2] + 1;
                break;
            case 3:  // move one step down
                next_pos[1] = next_pos[1] + 1;
                break;
            case 4:  // move one step left
                next_pos[2] = next_pos[2] - 1;
                break;
        }
    }
}

// pos on the face after the face rotation
void pos_face_rot(uint8_t *pos, uint8_t face_rot) {
    uint8_t old_row = pos[1];
    uint8_t old_column = pos[2];
    switch (face_rot) {
        case 1:  // no rotation
            break;
        case 2:  // rotate left
            // the map(coordiates) rotate left, the position rotate right
            pos[1] = old_column;
            pos[2] = 4 - old_row;
            break;
        case 3:  // rotate right
            // the map(coordiates) rotate right, the position rotate left
            pos[1] = 4 - old_column;
            pos[2] = old_row;
            break;
        case 4:  // rotate 180
            pos[1] = 4 - old_row;
            pos[2] = 4 - old_column;
            break;
    }
}

// all the pixels are rearranged by this function
// input is a index in a 6x5x5 matrix
// output is the sequence in the neopixel serial connection, from 0 to NUMPIXELS-1
// return -1 if there is index error, not used
int pixel_indexing(uint8_t *pos) {
    // pos[0] is face index(0~5), total 6 faces;
    // pos[1] is row index(0~4), total 5 rows;
    // pos[2] is row index(0~4), total 5 columns; 
    int serial_index;  // for return
    if (pos[0] == 4 || pos[0] == 5) {
        // the bottom or top surfaces
        serial_index = 25*pos[0] + 5*pos[1] + pos[2];
        return serial_index;
    }
    else {
        // the side surfaces
        // all the led on the side surfaces need to be rotate ccw 90 degrees
        uint8_t row_temp = 4 - pos[2];
        uint8_t column_temp = pos[1];
        serial_index  = 25*pos[0] + 5*row_temp + column_temp;
        return serial_index;
    }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t pixel_wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)
    {
        return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    else if(WheelPos < 170)
    {
        WheelPos -= 85;
        return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    else
    {
        WheelPos -= 170;
        return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data     
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address 
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

// calibrate only gyro in mpu6050, by accumulating readings and taking average
// the device should be static along the time
// reason not calibrate accelerometer is that device orientation is not upright
void calibrateGyro()
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);  

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00); 
    delay(200);

    // Configure device for bias calculation
    writeByte(MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU6050_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU6050_ADDRESS, MPU6050_RA_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU6050_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, 0x40);   // Enable FIFO  
    writeByte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU6050_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU6050_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        // int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        int16_t gyro_temp[3] = {0, 0, 0};
        readBytes(MPU6050_ADDRESS, MPU6050_RA_FIFO_R_W, 12, &data[0]); // read data for averaging
        // accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        // accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        // accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        // accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        // accel_bias[1] += (int32_t) accel_temp[1];
        // accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }
    // accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    // accel_bias[1] /= (int32_t) packet_count;
    // accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    // if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    // else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    writeByte(MPU6050_ADDRESS, MPU6050_RA_XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, data[5]);

    // dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    // dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    // dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // // the accelerometer biases calculated above must be divided by 8.

    // int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    // readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    // accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    // readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    // accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    // readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    // accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    // uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    // uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    // for(ii = 0; ii < 3; ii++) {
    //     if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    // }

    // // Construct total accelerometer bias, including calculated average accelerometer bias from above
    // accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    // accel_bias_reg[1] -= (accel_bias[1]/8);
    // accel_bias_reg[2] -= (accel_bias[2]/8);

    // data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    // data[1] = (accel_bias_reg[0])      & 0xFF;
    // data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    // data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    // data[3] = (accel_bias_reg[1])      & 0xFF;
    // data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    // data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    // data[5] = (accel_bias_reg[2])      & 0xFF;
    // data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // // Push accelerometer biases to hardware registers
    // writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    // writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
    // writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
    // writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);  
    // writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
    // writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

    // // Output scaled accelerometer biases for manual subtraction in the main program
    // dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
    // dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    // dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


