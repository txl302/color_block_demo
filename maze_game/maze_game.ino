// This is the 3D maze game for the color block 2.0

// info about hardware orientation and indexing
// Cap is face 6, also the top of the block.
// The top of the circuit is facing face 3, so x of mpu6050 points to face 2, y points to
// face 6(top), z points to face 3.

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
#include "pitches.h"
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
float roll_rad, pitch_rad, yaw_rad;
float roll_deg, pitch_deg, yaw_deg;
float ax_c, ay_c, az_c;  // gravity-compensated acceleration on body frame
// for quaternion update
float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;

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

// flow control
unsigned long time_last, time_now;  // microsecond
unsigned long period = 20000;  // period for main loop
// debug print control
unsigned long debug_period = 100000;
uint8_t debug_freq = (uint8_t)((float)debug_period/period);
uint8_t debug_count = 0;

// declare functions
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void calibrateGyro();
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);

void setup() {
    Serial.begin(9600);  // print test message

    // mpu6050
    Wire.begin();  // join I2C bus
    calibrateGyro();
    mpu6050.initialize();

    // vibration motor, indicate block is powered on
    analogWrite(VIBR_MOTOR, 100);
    delay(500);
    analogWrite(VIBR_MOTOR, 0);

    // // neopixels, light up the maze
    // pixels.begin();
    // int serial_index;
    // uint8_t pos[3];
    // for (uint8_t face=0; face<6; face++) {
    //     for (uint8_t row=0; row<5; row++) {
    //         for (uint8_t column=0; column<5; column++) {
    //             pos[0] = face; pos[1] = row; pos[2] = column;
    //             serial_index = pixel_indexing(pos);
    //             if (MAZE[face][row][column] == 1) {
    //                 pixels.setPixelColor(serial_index, color_bright);
    //             }
    //             else {
    //                 pixels.setPixelColor(serial_index, color_dark);
    //             }
    //         }
    //     }
    // }
    // serial_index = pixel_indexing(pos_player);
    // pixels.setPixelColor(serial_index, color_player);
    // serial_index = pixel_indexing(pos_des);
    // pixels.setPixelColor(serial_index, color_des);
    // pixels.show();

    time_last = micros();
}

void loop() {
    time_now = micros();
    if ((time_now - time_last) > period) {
        deltat = (time_now - time_last)/1000000.0f;
        time_last = time_now;

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
        roll_rad = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch_rad = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        yaw_rad = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        roll_deg = roll_rad * 180.0f / PI;
        pitch_deg = pitch_rad * 180.0f / PI;
        yaw_deg = yaw_rad * 180.0f / PI; 

        // gravity-compensated acceleration
        ax_c = ax - 2*(q[1]*q[3] - q[0]*q[2]);
        ay_c = ay - 2*(q[0]*q[1] + q[2]*q[3]);
        az_c = az - (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

        // debug print
        debug_count = debug_count + 1;
        if (debug_count > debug_freq) {
            debug_count = 0;
            // // display tab-separated accel/gyro x/y/z values
            // Serial.print("a/g:\t");
            // Serial.print(ax); Serial.print("\t");
            // Serial.print(ay); Serial.print("\t");
            // Serial.print(az); Serial.print("\t");
            // Serial.print(gx); Serial.print("\t");
            // Serial.print(gy); Serial.print("\t");
            // Serial.println(gz);
            Serial.print("accel/rpy:\t");
            Serial.print(ax_c); Serial.print("\t");
            Serial.print(ay_c); Serial.print("\t");
            Serial.print(az_c); Serial.print("\t");
            Serial.print(roll_deg); Serial.print("\t");
            Serial.print(pitch_deg); Serial.print("\t");
            Serial.println(yaw_deg);
        }
    }
}

// return the index of current top surface


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


