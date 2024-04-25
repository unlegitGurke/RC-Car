#include <Wire.h>           // I2C Arduino Library
#include <DFRobot_BMI160.h> // DFRobot BMI160 Library
#include <movingAvg.h>      // Moving Average Library

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

movingAvg avgGyroX(2);  // Define moving average objects for each axis
movingAvg avgGyroY(2);
movingAvg avgGyroZ(2);

movingAvg avgAccelX(2); // Define moving average objects for each axis
movingAvg avgAccelY(2);
movingAvg avgAccelZ(2);


#define MAG_ADDR 0x0D   // I2C address of the QMC5883L


#define Mode_Standby    0b00000000    // Values for the QMC5883 control register 1
#define Mode_Continuous 0b00000001
#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100
#define RNG_2G          0b00000000
#define RNG_8G          0b00010000
#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

  // Quaternion variables
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Initialize quaternion
float beta = 0.1f;  // Beta parameter for sensor fusion algorithms
float deltat = 0.01f;  // Time interval between sensor updates (in seconds)
float Kp = 2.0f; // Proportional gain for Mahony filter
float Ki = 0.005f; // Integral gain for Mahony filter
float eInt[3] = {0.0f, 0.0f, 0.0f}; // Integral error

// Function prototypes
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

// Function to write data into a register on QMC5883L
void IMUwriteRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAG_ADDR); // Start talking
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Function to read results from QMC5883L
void IMUreadData(uint16_t * x, uint16_t * y, uint16_t * z) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDR, 6);
  *x = Wire.read(); // LSB  x
  *x |= Wire.read() << 8; // MSB  x
  *y = Wire.read(); // LSB  z
  *y |= Wire.read() << 8; // MSB z
  *z = Wire.read(); // LSB y
  *z |= Wire.read() << 8; // MSB y
}

// Function to set the control register 1 on QMC5883L
void setCtrlRegister(uint8_t overSampling, uint8_t range, uint8_t dataRate, uint8_t mode) {
  IMUwriteRegister(9, overSampling | range | dataRate | mode);
}

// Function to reset QMC5883L
void softReset() {
  IMUwriteRegister(0x0a, 0x80);
  IMUwriteRegister(0x0b, 0x01);
}

// Prepare hardware
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Start");
  softReset();
  setCtrlRegister(OSR_128, RNG_2G, ODR_100Hz, Mode_Continuous);
  Serial.println("Init done");

  if (bmi160.softReset() != BMI160_OK){
    Serial.println("BMI160 reset failed");
    while(1);
  }

  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("BMI160 init failed");
    while(1);
  }


  avgGyroX.begin(); // Initialize moving averages for each axis for the Gyro
  avgGyroY.begin();
  avgGyroZ.begin();

  avgAccelX.begin(); // Initialize moving averages for each axis for the Accel
  avgAccelY.begin();
  avgAccelZ.begin();
}

void loop() {
  int gxRaw, gyRaw, gzRaw; // Raw gyro values
  int axRaw, ayRaw, azRaw; // Raw accelerometer values
  uint16_t mx, my, mz; // Raw magnetometer values

  // Read raw gyro measurements from device
  int16_t gyroRaw[3];
  bmi160.getGyroData(gyroRaw);
  gxRaw = gyroRaw[0];
  gyRaw = gyroRaw[1];
  gzRaw = gyroRaw[2];

  // Read raw accelerometer measurements from device
  int16_t accelRaw[3];
  bmi160.getAccelData(accelRaw);
  axRaw = accelRaw[0];
  ayRaw = accelRaw[1];
  azRaw = accelRaw[2];

  // Read raw magnetometer measurements from device
  IMUreadData(&mx, &my, &mz);

  // Convert raw sensor data to appropriate units
  float ax = axRaw / 2048.0; // Convert accelerometer raw values to g
  float ay = ayRaw / 2048.0;
  float az = azRaw / 2048.0;
  
  float gx = gxRaw * 0.007629; // Convert gyroscope raw values to degrees/second
  float gy = gyRaw * 0.007629;
  float gz = gzRaw * 0.007629;
  
  // Update moving averages Gyro
  float avgGX = avgGyroX.reading(gx);
  float avgGY = avgGyroY.reading(gy);
  float avgGZ = avgGyroZ.reading(gz);

  // Update moving averages Accel
  float avgAX = avgAccelX.reading(ax);
  float avgAY = avgAccelY.reading(ay);
  float avgAZ = avgAccelZ.reading(az);

  
  // Update quaternion orientation using Madgwick or Mahony algorithm
  MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz);
  //MahonyQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz);

  // Send quaternion orientation data over serial for visualization or further processing
  Serial.print("Quaternion: ");
  Serial.print(q[0]);
  Serial.print("\t");
  Serial.print(q[1]);
  Serial.print("\t");
  Serial.print(q[2]);
  Serial.print("\t");
  Serial.println(q[3]);

  // Send gyro x/y/z values over serial
  Serial.print("Gyro:");
  Serial.print(avgGX);
  Serial.print("\t");
  Serial.print(avgGY);
  Serial.print("\t");
  Serial.print(avgGZ);
  Serial.println();

  // Send accelerometer x/y/z values over serial
  Serial.print("Accel:");
  Serial.print(avgAX);
  Serial.print("\t");
  Serial.print(avgAY);
  Serial.print("\t");
  Serial.print(avgAZ);
  Serial.println();

  // Convert raw magnetometer data to appropriate units
  float mxScaled = mx / 32768.0; // Assuming magnetometer range is ±2 Gauss
  float myScaled = my / 32768.0;
  float mzScaled = mz / 32768.0;

  // Send magnetometer x/y/z values over serial
  Serial.print("Magnetometer:");
  Serial.print(mxScaled);
  Serial.print("\t");
  Serial.print(myScaled);
  Serial.print("\t");
  Serial.println(mzScaled);
}



// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
// This code is directly copied from the https://github.com/kriswiner/MPU9250/blob/master/quaternionFilters.ino reposotory, written by Kris Winer.
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}
  
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {   // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and measured ones. 
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;   

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
      eInt[0] += ex;      // accumulate integral error
      eInt[1] += ey;
      eInt[2] += ez;
  }
  else
  {
      eInt[0] = 0.0f;     // prevent integral wind up
      eInt[1] = 0.0f;
      eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}
