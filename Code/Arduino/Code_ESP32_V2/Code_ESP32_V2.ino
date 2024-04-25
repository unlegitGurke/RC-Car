  //Create TaskHandles

  TaskHandle_t Task1;
  TaskHandle_t Task2;

  //Libraries used across everything

  #include <math.h>

  //General Values

  const int ADCRes = 12;  //Resolution of ADC in Bits (Arduino Nano: 10, ESP32: 12)
  const int ErrorCnt = 11;   //Amount of Error Variables 
  bool ErrorESP[ErrorCnt] = {0,0,0,0,0,0,0,0,0,0,0};   //Error Codes are saved in here:  0 - 6 : Temp Sensor Error, 
                                                    //7 - 11 : Max Voltage too HIGH for Microcontroller
  bool IsError = 0;    //True if theres any Error
  bool IsErrorPanda = 0;    //True, if theres an Error on the LattePanda
  
  ////////// C O R E 1 //////////  

  //Declare PWM Fan Controller

  const int Fan_Pin[4] = {17, 5, 18, 19};   //Pins, the PWM Pins of the fans are connected to

  const int PWMChannel[4] = {0, 2, 4, 6};   //Which PWM Channel of Microncontroller will be used

  const int Resolution[4] = {10, 10, 10, 10};   //ADC Resolution

  const int Frequency[4] = {25000, 25000, 25000, 25000};    //PWM Frequency of the Fans

  int FanSpeed[4] = {100, 100, 100, 100};   //Speed of the fans at the beginning of the program

  int DutyCycle[4] = {0, 0, 0, 0};    //Fanspeed converted to a value for the ADC

  //Declare Accelerometer, Gyroscope and Magnetometer

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
  float IMUq[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Initialize quaternion
  float beta = 0.1f;  // Beta parameter for sensor fusion algorithms
  float deltat = 0.01f;  // Time interval between sensor updates (in seconds)
  float Kp = 2.0f; // Proportional gain for Mahony filter
  float Ki = 0.005f; // Integral gain for Mahony filter
  float eInt[3] = {0.0f, 0.0f, 0.0f}; // Integral error  

  //Declare Ultrasonic Sensors

  #include "OctoSonar.h"

  #define SONAR_ADDR 0x20
  #define SONAR_INT 2
  #define ACTIVE_SONARS 0xFFFF

  const int ReadIntervallSonar = 200; //Time between Sensor Readingsd in millisceonds
  uint32_t last_print = 0;
  uint8_t SonarValues[16];

  OctoSonarX2 myOcto(SONAR_ADDR, SONAR_INT);

  //Declare Temperature Sensors

  const float temprangemin = 218.15;    //Temperature Range of thermistor in Kelvin
  const float temprangemax = 398.15;

  const int NrOfSensors = 6;              //Define Number of Sensors connected to the Analog Pins of the Arduino
  const int SensorPins[NrOfSensors] = {34,35,32,33,25,26}; //Define Pins of Sensors

  const int ErrorLedPin = 4; //Pin for Error LED
  
  int TempUnit = 2;   //Choose which Unit the Temperature will be printed in, 1 for Kelvin, 2 for Celcius, 3 for Fahrenheit

  const unsigned int ReadIntervallTemp = 200; //Time between Sensor Readings in milliseconds
  unsigned long previousMillisTemp = 0;
  unsigned long currentMillisTemp;

  float temp[NrOfSensors + 1][3];

  //Declare Voltage Sensors

  const float R1[] = {100000, 100000, 100000, 100000, 100000};   //Voltage Divider Resistor Values
  const float R2[] = {4700, 4700, 4700, 4700, 4700};

  const int NOSVoltage = 4;   //Number of Sensors plugged in
  const int VoltagePin[] = {12, 13, 14, 27};  //Sensor Pins
  const float LogicLevel = 3.3;   //Logic Level of the microcontroller
  const float MaxVoltage = 80;  //Max Voltage that will be measured

  float Voltage[NOSVoltage];  //Voltage Data will be saved in here
  float InputVoltage[5] = {0,0,0,0};

  const unsigned int ReadIntervallVoltage = 10;   //Time between Sensor Readings in milliseconds
  unsigned long previousMillisVoltage = 0;
  unsigned long currentMillisVoltage;

  //Declare ESC-Power-Button

  #define ESC_BUTTON_PIN 4
  #define ESC_PIN 15

  bool ESCState = 0;
  bool ESCButtonState = 0;

  //Serial Communication
  
  #include "LattePandacomms.h"
  
  LattePandacomms LattePanda;
  
  #define startMarker 0x78   //Marks Beggging of Datastream   ASCII for x
  #define endMarker 0x71    //Marks End of Datastream   ASCII for q
  #define maxMessage 512   //Number of Bytes that can be transmitted in one message

  bool inProgress = false;
  bool allReceived = false;
  bool DataSent = false;
  
  byte bytesRecvd = 0;
  byte dataRecvCount = 0; 

  char tempBufferIn[maxMessage];
  char tempBufferOut[maxMessage];
  
  int nb = 0;

  ////////// C O R E 2 //////////

  //Lighting WS2812B using FastLED

  // Declare FastLED

  #include <FastLED.h>

  #define NUM_LEDS_BACK 45    //Declare Back LEDStrip
  #define LED_PIN_BACK 23

  #define NUM_LEDS_FRONT 34   //Declare Front LED-Strip
  #define LED_PIN_FRONT 0

  #define NUM_LEDS_INTERNAL 8
  #define LED_PIN_INTERNAL 16

  CRGB ledsback[NUM_LEDS_BACK];
  CRGB ledsfront[NUM_LEDS_FRONT];
  CRGB ledsinternal[NUM_LEDS_INTERNAL];

  unsigned int Effekt = 0;

  //Define LED Groups
  
  const int fr1length = 7;    //Define Length of each front LED Section
  const int fr2length = 6;
  const int fr3length = 4;
  const int fl1length = 7;
  const int fl2length = 6;
  const int fl3length = 4;
  
  int ledsfr1[fr1length] = {0,1,2,3,4,5,6};             //Group Front LEDs to match shape they're arragned in:   - - - - - - -  fr1       fl1 - - - - - - -
  int ledsfr2[fr2length] = {12,11,10,9,8,7};            //                                                         - - - - - -  fr2       fl2 - - - - - -
  int ledsfr3[fr3length] = {13,14,15,16};               //                                                             - - - -  fr3       fl3 - - - -
  int ledsfl1[fl1length] = {17,18,19,20,21,22,23};
  int ledsfl2[fl2length] = {29,28,27,26,25,24};
  int ledsfl3[fl3length] = {30,31,32,33};

  //Define Buttons

  //const int ButtonPins[6] = {13, 12, 14, 27, 15, 16};   //Pin of Buttons for Control

  bool IndicatorRightState = 0;   //Variables for button presses / Serial communication
  bool IndicatorLeftState = 0;
  bool BrakeLightState = 0;
  bool ReverseLightState = 0;
  bool HazardState = 0;

  //Define indicator
  float IndicatorSize = 0.25;   //Size of each LED-Strip Segment which has an Indicator

  //Define Reverse Light
  int endspace = 0;   //Distance from reverslights to end of strip
  int revlightsize = NUM_LEDS_BACK * IndicatorSize;    //Size of Reverslight on each side
  
  //Define FadeToColor
  int FadeToColorsteps = 0;
  int r1, r2, b1, b2, g1, g2;
  unsigned int d1, d2, d3;
  

  //Define Colors
  const unsigned long idlecolback = 0x400000;    //Hex Codes of each color
  const unsigned long brakecol = 0xff0000;
  const unsigned long indicatorcol = 0xff4000;
  const unsigned long reversecol = 0xffffff;
  const unsigned long frontcol = 0xffffff;
  const unsigned long frontcoldim = 0x202020;
  const unsigned long blackcol = 0x000000;
  const unsigned long idlecolfront = 0x202020;
  unsigned long EffektColor1;
  unsigned long EffektColor2;

  //Non-Blocking Delay Variables
  unsigned long currentMillisLED = 0;
  unsigned long previousMillisLED = 0;
  
  unsigned long previousMillisFadeToColor = 0;
  
  unsigned long previousMillisStartup[3] = {0, 0, 0};
  
  unsigned long previousMillisIndicator[6] = {0, 0, 0, 0, 0, 0,};

  int IndicatorAnimTime = 10; //Time between each Indicator LED turning on
  int IndicatorOnTime = 350;  //Time the whole indicator stays on
  int IndicatorOffTime = 500; //Time between eacht indicator animation

  int StartupAnimTime = 25; //Time between each LED activatiung at startup
  int StartupFadeTime = 1000; //Time for Fade at the end of startup
  
  //State Checks
  bool IsStartup = 0;
  int StartupState[6] = {10, 0, 0, 0, 0, 0};
  int FadeToColorState = 0;
  int IndicatorState[6] = {10, 0, 0, 10, 0, 0};
  bool IndicatorInProgress[2] = {0, 0};


void setup() {

  Serial.begin(115200);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1setup,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2setup,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500);

}

void loop() {

}

void Task1setup( void * pvParameters ) {

  //Serial.print("Task1 running on core ");
  //Serial.println(xPortGetCoreID());

  delay(3000);
  
  //MPU 

  Wire.begin();
  Serial.println("Start IMU Init:");
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

  //Octosonar

  myOcto.begin(ACTIVE_SONARS);   // initialize OctoSonar

  //Fan Control

  ledcSetup(PWMChannel[0], Frequency[0], Resolution[0]);
  ledcSetup(PWMChannel[1], Frequency[1], Resolution[1]);
  ledcSetup(PWMChannel[2], Frequency[2], Resolution[2]);
  ledcSetup(PWMChannel[3], Frequency[3], Resolution[3]);

  ledcAttachPin(Fan_Pin[0], PWMChannel[0]);
  ledcAttachPin(Fan_Pin[1], PWMChannel[1]);
  ledcAttachPin(Fan_Pin[2], PWMChannel[2]);
  ledcAttachPin(Fan_Pin[3], PWMChannel[3]);

  //Voltage Sensors

  pinMode(VoltagePin[0], INPUT);
  pinMode(VoltagePin[1], INPUT);
  pinMode(VoltagePin[2], INPUT);
  pinMode(VoltagePin[3], INPUT);
  pinMode(VoltagePin[4], INPUT);

  for(int i = 0; i < 5; i++) {
    InputVoltage[i] = MaxVoltage / ((R1[i] + R2[i])/R2[i]);     //Calculate Voltage which is applied to GPIO at MaxVoltage    
    if(InputVoltage[i] > LogicLevel) {   //If the Voltage applied to the GPIOS can be too high, set Error HIGH
      ErrorESP[7+i] = 1;
    }
    else {
      ErrorESP[7+i] = 0;
    }
  }

  for(;;){
    Task1loop();   
  }   
  
}

void Task1loop() {
  
  //CheckError();
  //ReadTemp();
  //ReadSonar();
  ReadIMU();
  Fan_Control();
  //VoltageSensor();
  //ESCPower();
  //ConvertVarToString();
  getSerialData();

  //Serial.println("Test");

  delay(1);

}

void ReadTemp() {

  currentMillisTemp = millis();
  if(currentMillisTemp - previousMillisTemp >= ReadIntervallTemp) {
    previousMillisTemp = currentMillisTemp;
  
    ErrorESP[6] = 0;
    
    for(int i = 0;i <= NrOfSensors - 1;i++){    //Read Temps from all sensors and prints them
      SaveTemp(SensorPins[i], i);
      //Serial.print("Temp");                   //DEBUG Only
      //Serial.print(i + 1);
      //Serial.print(": ");
      //Serial.print(temp[i][TempUnit]);
      //Serial.print(" C Error: ");
      //Serial.println(ErrorESP[i]);
  
      if(ErrorESP[i] == 1) {
        ErrorESP[6] = 1;              
      }
      
    }
  
    if(ErrorESP[6] == 1) {
      digitalWrite(ErrorLedPin, HIGH);
    }
    else {
      digitalWrite(ErrorLedPin, LOW);
    }
    
    //Serial.println(" ");
    
  }
  
}

void SaveTemp(int Pin, int Pos) {   //Pin of Sensor and Position of Value in temp Array
  
  int tempReading = analogRead(Pin);
  double tempK = log(10000.0 * ((pow(2,ADCRes) / tempReading - 1)));
  tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK )) * tempK );       //  Temp Kelvin
  float tempC = tempK - 273.15;            // Convert Kelvin to Celcius
  float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit

  if(tempK > temprangemin && tempK < temprangemax) {
    temp[Pos][1] = tempK;
    temp[Pos][2] = tempC;
    temp[Pos][3] = tempF;    
    ErrorESP[Pos] = 0;
  }
  else {
    temp[Pos][1] = 0;
    temp[Pos][2] = 0;
    temp[Pos][3] = 0;    
    ErrorESP[Pos] = 1;
  }
  
}

void ReadSonar() {
  OctoSonar::doSonar();  // call every cycle, OctoSonar handles the spacing

  if (last_print + ReadIntervallSonar < millis()) {   
    last_print = millis();
    for (uint8_t i = 0; i < 16; i++) {
      SonarValues[i] = myOcto.read(i);
      Serial.print(SonarValues[i]); Serial.print("   ");
      
    }
    Serial.println();
  }  
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);    //IMU// / Function prototypes
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);    //IMU// / Function prototypes

void IMUwriteRegister(uint8_t reg, uint8_t val) {    //IMU// / Function to write data into a register on QMC5883L
  Wire.beginTransmission(MAG_ADDR); // Start talking
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}


void IMUreadData(uint16_t * x, uint16_t * y, uint16_t * z) {   //IMU// / Function to read results from QMC5883L
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


void setCtrlRegister(uint8_t overSampling, uint8_t range, uint8_t dataRate, uint8_t mode) {   //IMU// / Function to set the control register 1 on QMC5883L
  IMUwriteRegister(9, overSampling | range | dataRate | mode);
}

void softReset() {    //IMU// / Function to reset QMC5883L 
  IMUwriteRegister(0x0a, 0x80);
  IMUwriteRegister(0x0b, 0x01);
}

void ReadIMU() {    //Reads IMU Sensor Data and filters it
  
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
  Serial.print(IMUq[0]);
  Serial.print("\t");
  Serial.print(IMUq[1]);
  Serial.print("\t");
  Serial.print(IMUq[2]);
  Serial.print("\t");
  Serial.println(IMUq[3]);

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
  float q1 = IMUq[0], q2 = IMUq[1], q3 = IMUq[2], q4 = IMUq[3];   // short name local variable for readability
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
  IMUq[0] = q1 * norm;
  IMUq[1] = q2 * norm;
  IMUq[2] = q3 * norm;
  IMUq[3] = q4 * norm;

}
  
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {   // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and measured ones. 
  float q1 = IMUq[0], q2 = IMUq[1], q3 = IMUq[2], q4 = IMUq[3];   // short name local variable for readability
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
  IMUq[0] = q1 * norm;
  IMUq[1] = q2 * norm;
  IMUq[2] = q3 * norm;
  IMUq[3] = q4 * norm;

}

void Fan_Control() {

  for(int i = 0;i < 4;i++) {
    
    FanSpeed[i] = LattePanda.Fan.Data[i];   //Read Data from LattePanda

    DutyCycle[i] = map(FanSpeed[i], 0, 100, 0, pow(2, Resolution[i]));    //Calculate DutyCycle from Speed value 

    ledcWrite(PWMChannel[i], DutyCycle[i]);     //Write PWM Signal to Pins

  }

}

void VoltageSensor() {
  
  currentMillisVoltage = millis();
  if(currentMillisVoltage - previousMillisVoltage >= ReadIntervallVoltage) {
    previousMillisVoltage = currentMillisVoltage;

    for(int i = 0;i < NOSVoltage;i++) {                         
      float value = analogRead(VoltagePin[i]);                              //Reads all the AnalogPins Values
      Voltage[i] = value * (LogicLevel/pow(2, ADCRes)) * ((R1[i] + R2[i])/R2[i]);    //Calculates the voltages from the sensorpins values
    }
    
  /*for(int i = 0;i < NOSVoltage;i++) {   //Prints all Voltages to Serial monitor DEBUG Only
      Serial.print(Voltage[0]);
      Serial.print(" ");
    }
  */}
}

void ESCPower() {   //Turns ESC on and off

  ESCButtonState = digitalRead(ESC_BUTTON_PIN);

  if(ESCButtonState == HIGH) {
    ESCState = true;
  }
  else{
    ESCState = false;
  }

  if(ESCState == HIGH) {
    digitalWrite(ESC_PIN, HIGH);
    //Serial.println("HIGH");     //DEBUG
  }
  else{
    digitalWrite(ESC_PIN, LOW);
    //Serial.println("LOW");      //DEBUG
  }

}

void ConvertVarToString() {   //Converts Data from Variables to a String to be sent

  //int NOSVoltage = 5;     //DEBUG ONLY
  //float Voltage[5] = {12.1, 56.4, 89.2, 23.5, 74.2};    //DEBUG ONLY
  
  char BufferSonar[128];
  char BufferIMU[64];
  char StringIMUTemp[7];
  char StringVoltage[5][5];
  char BufferVoltage[32];
  char StringTemp[6][6];
  char BufferTemp[32];
  
  
  for(int i = 0; i < NOSVoltage; i++) {           //Converts Voltage Floats to String
    dtostrf(Voltage[i], 3, 1, StringVoltage[i]);
  }
  
  for(int i = 0; i < NrOfSensors; i++) {           //Converts Temp Floats to String
    dtostrf(temp[i][TempUnit], 3, 1, StringTemp[i]);
  }
  
  sprintf(BufferSonar, "x%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,", SonarValues[0], SonarValues[1], SonarValues[2], SonarValues[3], SonarValues[4], SonarValues[5], SonarValues[6], SonarValues[7], SonarValues[8], SonarValues[9], SonarValues[10], SonarValues[11], SonarValues[12], SonarValues[13], SonarValues[14], SonarValues[15]);    //convert all Variables into substrings
  sprintf(BufferVoltage, "%s,%s,%s,%s,%s,", StringVoltage[0], StringVoltage[1], StringVoltage[2], StringVoltage[3], StringVoltage[4]);
  sprintf(BufferTemp, "%s,%s,%s,%s,%s,%s,%huq", StringTemp[0], StringTemp[1], StringTemp[2], StringTemp[3], StringTemp[4], StringTemp[5], IsError);
  sprintf(tempBufferOut, "%s%s%s%s", BufferSonar, BufferIMU, BufferVoltage, BufferTemp); //Combine all substrings into one
  //strcat(tempBufferOut, BufferSonar);
  //strcat(tempBufferOut, BufferIMU);
  //strcat(tempBufferOut, BufferVoltage);
  //strcat(tempBufferOut, BufferTemp);

  //Serial.println(tempBufferOut);

}

void CheckError() {   //Checks if there has been an Error

  IsError = false;  

  for (int i = 0; i < ErrorCnt; i++) {
    
    if (ErrorESP[i]) {  
      
      IsError = true;  
       
    }
    
  }

}

void getSerialData() { 	  //If printout = 1 sends back data, 0 doesnt send back data
  
  LattePanda.refresh(startMarker, endMarker);
  LattePanda.decode(startMarker, endMarker);

  /*
  Serial.print("Type: ");                       //DEBUG
  Serial.println(LattePanda.LED.Type);            
  Serial.print("nVal: ");
  Serial.println(LattePanda.LED.nVal);
  Serial.print("Access: ");
  Serial.println(LattePanda.LED.Access);
  for (int i = 0; i < LattePanda.LED.nVal; ++i) {
    Serial.print("Data"); Serial.print(i); Serial.print(": ");
    Serial.println(LattePanda.LED.Data[i]);
  }
  Serial.print("Error: ");
  Serial.println(LattePanda.LED.Error);
  Serial.println("");
  */
}

void Task2setup( void * pvParameters ) {
  
  FastLED.addLeds<WS2812B, LED_PIN_BACK, GRB>(ledsback, NUM_LEDS_BACK);  //Initlialize Back LEDStrip
  FastLED.addLeds<WS2812B, LED_PIN_FRONT, GRB>(ledsfront, NUM_LEDS_FRONT);  //Initialize Front LEDStrip  
  FastLED.addLeds<WS2812B, LED_PIN_INTERNAL, GRB>(ledsinternal, NUM_LEDS_INTERNAL);
    
  pinMode(LED_PIN_BACK, OUTPUT);              //Declare Pins Input/Output DEBUG Only
  pinMode(LED_PIN_FRONT, OUTPUT);
  pinMode(LED_PIN_INTERNAL, OUTPUT);
  //pinMode(ButtonPins[0], INPUT);
  //pinMode(ButtonPins[1], INPUT);
  //pinMode(ButtonPins[2], INPUT);
  //pinMode(ButtonPins[3], INPUT);
  //pinMode(ButtonPins[4], INPUT);
  //pinMode(ButtonPins[5], INPUT);  

  fill_solid(ledsback, NUM_LEDS_BACK, CRGB::Black);    //Turn off all LEDs at startup
  fill_solid(ledsfront, NUM_LEDS_FRONT, CRGB::Black);
  fill_solid(ledsinternal, NUM_LEDS_INTERNAL, CRGB::Red);
  FastLED.show();

  IsStartup = 1;    //Run Startup Animation

  for(;;){
    Task2loop();
  }
    
}

void Task2loop() {
  
  currentMillisLED = millis();
  
  if(IsStartup == 1) {
    startup();
  }

  else {
    
    ReadState();
    idle();
    
    if(IndicatorRightState == true) {
      Indicator(0);
      IndicatorInProgress[0] = 1;
    }
    
    if(IndicatorInProgress[0] == true && IndicatorInProgress[1] == false) {
      Indicator(0);
    }
    
    if(IndicatorLeftState == true) {
      Indicator(1);
      IndicatorInProgress[1] = 1;
    }
    
    if(IndicatorInProgress[0] == false && IndicatorInProgress[1] == true) {
      Indicator(1);
    }
    
    if(HazardState == true) {
      Indicator(2);
      IndicatorInProgress[0] = 1;
      IndicatorInProgress[1] = 1;
    }
    
    if(IndicatorInProgress[0] == true && IndicatorInProgress[1] == true) {
      Indicator(2);
    }
    
    if(BrakeLightState == true) {
      BrakeLight();
    }
    
    if(ReverseLightState == true) {
      ReverseLight();
    }
    
  }

  delay(1);

}

void ReadState() { 
  
  uint8_t StateIn = LattePanda.LED.Data[0];

  bool State[8] = {};

  for (int i = 7; i >= 0; i--) {
    State[i] = StateIn%2;
    StateIn=StateIn/2;
  }

  IndicatorRightState = State[0];
  IndicatorLeftState = State[1];
  BrakeLightState = State[2];
  ReverseLightState = State[3];
  HazardState = State[4];


  /*
  IndicatorRightState = digitalRead(ButtonPins[0]);   //Buttons for testing only
  IndicatorLeftState = digitalRead(ButtonPins[1]);
  BrakeLightState = digitalRead(ButtonPins[2]);
  ReverseLightState = digitalRead(ButtonPins[3]);
  HazardState = digitalRead(ButtonPins[4]);
  */

  //Serial.print(IndicatorRightState);    //DEBUG
  //Serial.print(IndicatorLeftState);
  //Serial.print(BrakeLightState);
  //Serial.print(ReverseLightState);
  //Serial.println(HazardState);
  
}

void startup() {

  switch(StartupState[0]) { //Controls State of Animation sequnce
   
    case 10:
    
      StartupState[1] = NUM_LEDS_BACK / 2;
      StartupState[2] = NUM_LEDS_BACK / 2;
      StartupState[3] = 0;
      StartupState[4] = 0;
      StartupState[5] = NUM_LEDS_BACK / 2;
          
      StartupState[0]++;
      
    break;
    
    case 11:
      
      if(StartupState[1] <= NUM_LEDS_BACK && currentMillisLED - previousMillisStartup[0] >= StartupAnimTime) {
         
        ledsback[StartupState[1]] = idlecolback;
        ledsback[StartupState[2]] = idlecolback;
        SwitchFrontLedColor(StartupState[3],frontcoldim,2);   

        if(StartupState[1] - (NUM_LEDS_BACK / 2) >= 4) {
          
          ledsback[StartupState[1] - 4] = blackcol;
          ledsback[StartupState[2] + 4] = blackcol;
          
        } 
        
        if(StartupState[3] >= 1) {
          
          ledsfront[ledsfr1[constrain(StartupState[3],0,fr1length - 1) - 1]] = blackcol;
          ledsfront[ledsfr2[constrain(StartupState[3],0,fr2length - 1) - 1]] = blackcol;
          ledsfront[ledsfr3[constrain(StartupState[3],0,fr3length - 1) - 1]] = blackcol;
          ledsfront[ledsfl1[constrain(StartupState[3],0,fl1length - 1) - 1]] = blackcol;
          ledsfront[ledsfl2[constrain(StartupState[3],0,fl2length - 1) - 1]] = blackcol;
          ledsfront[ledsfl3[constrain(StartupState[3],0,fl3length - 1) - 1]] = blackcol;
          
        } 
          
        FastLED.show();

        StartupState[2]--;
        StartupState[3]++;    
        StartupState[1]++;
        previousMillisStartup[0] = currentMillisLED;
        
      }
      
      if(StartupState[1] >= NUM_LEDS_BACK) {
        
        StartupState[0]++; 
        
      }
       

    break;
    
    case 12:
    
      StartupState[2] = NUM_LEDS_BACK;
      StartupState[3] = fr1length; 
      StartupState[0]++;
    
    break;
    
    case 13:

      if(StartupState[4] <= NUM_LEDS_BACK / 2 && currentMillisLED - previousMillisStartup[1] >= StartupAnimTime) {
        ledsback[StartupState[4]] = idlecolback;
        ledsback[StartupState[2]] = idlecolback;
        SwitchFrontLedColor(StartupState[3],frontcoldim,2);

        FastLED.show();
        
        StartupState[2]--;
        StartupState[3]--;
        StartupState[4]++;
        previousMillisStartup[1] = currentMillisLED;   
      } 
    
      if(StartupState[4] >= NUM_LEDS_BACK / 2) {
        
         StartupState[0]++;
        
      }    
     

    break;
    
    case 14:
    
      StartupState[2] = NUM_LEDS_BACK / 2;
      StartupState[3] = 0;
    
      StartupState[0]++;
      
    break;
    
    case 15:
      
      if(StartupState[5] <= NUM_LEDS_BACK && currentMillisLED - previousMillisStartup[2] >= StartupAnimTime) {
        ledsback[StartupState[5]] = brakecol; 
        ledsback[StartupState[2]] = brakecol;
        SwitchFrontLedColor(StartupState[3],frontcol,2);

        FastLED.show();
        
        StartupState[2]--;
        StartupState[3]++;
        StartupState[5]++;
        previousMillisStartup[2] = currentMillisLED;
      
      }
      
      if(StartupState[5] >= NUM_LEDS_BACK) {
        
        StartupState[0]++;
        
      }     

    break;
    
    case 16:
      
      if(FadeToColor(brakecol, idlecolback, StartupFadeTime, 0, NUM_LEDS_BACK) == true) {
        
        StartupState[0]++;
        
      }
    
    break;
    
    case 17:
    
      IsStartup = 0;
      
      StartupState[0] = 10;
    
    break;
    
  }
 
}

void idle() {
  
   //If BrakeLight isnt supposed to be turned on, turn it off
   if(BrakeLightState == false) {
    for(int i = NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = idlecolback;              
    }      
   }
    
    //Checks if Left Indicator is running, if not lights up left side of Strip
    if(IndicatorLeftState == false && ReverseLightState == false && HazardState == false) {
      for(int i = 0; i <= NUM_LEDS_BACK * IndicatorSize;i++) {
        ledsback[i] = idlecolback;
      }   
    }
    
    //Checks if Right Indicator is running, if not lights up right side of the Strip
    if(IndicatorRightState == false && ReverseLightState == false && HazardState == false) {
      for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize; i <= NUM_LEDS_BACK;i++) {
        ledsback[i] = idlecolback;
      }
    }
    
    //If indicators arent turned on front lights idle
    if(IndicatorRightState == false && IndicatorLeftState == false && HazardState == false) {
      for(int i = 0;i <= NUM_LEDS_FRONT;i++) {    
        SwitchFrontLedColor(i,frontcoldim,2);
      }
    }
    
    FastLED.show();
  
}

void BrakeLight() {
  
  if(BrakeLightState == true) {
    
    //Lights up Center of Strip
    for(int i = NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i++) {
      ledsback[i] = brakecol;
    }
    
    //Checks if Left Indicator is running, if not lights up left side of Strip
    if(IndicatorLeftState == false && ReverseLightState == false && HazardState == false && BrakeLightState == true) {
      for(int i = 0; i <= NUM_LEDS_BACK * IndicatorSize;i++) {
        ledsback[i] = brakecol;
      }
      
    }
    
    //Checks if Right Indicator is running, if not lights up right side of the Strip
    if(IndicatorRightState == false && ReverseLightState == false && HazardState == false && BrakeLightState == true) {
      for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize; i <= NUM_LEDS_BACK;i++) {
        ledsback[i] = brakecol;
      }
    
    }
    
    FastLED.show();
    
  }
  
}

void ReverseLight() {  

  if(IndicatorRightState == false && HazardState == false && ReverseLightState == true) {  
    for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <=NUM_LEDS_BACK;i++) {
      ledsback[i] = reversecol;
    }
  }

  if(IndicatorLeftState == false && HazardState == false && ReverseLightState == true) {
    for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0 ;i--) {
      ledsback[i] = reversecol;
    }
  }

  FastLED.show();

}

void Indicator(int dir) {    //dir = 1 for left, dir = 0 for right

  if(dir == 1 || dir == 2) {    //Left Indicator
    
    switch(IndicatorState[0]) {
      
      case 10:
        
        IndicatorState[1] = 0;
        IndicatorState[2] = NUM_LEDS_BACK * IndicatorSize;
        
        IndicatorState[0]++;
        
      break;
      
      case 11:
      
        if(IndicatorState[2] >= 0 && currentMillisLED - previousMillisIndicator[0] >= IndicatorAnimTime) {   //Animation 
        
          ledsback[IndicatorState[2]] = indicatorcol; 
          SwitchFrontLedColor(IndicatorState[1],indicatorcol,1);
          FastLED.show();   
          
          IndicatorState[2]--;
          IndicatorState[1]++;
          previousMillisIndicator[0] = currentMillisLED; 
          
        }
        
        if(IndicatorState[2] < 0) {
          
          IndicatorState[0]++;
          previousMillisIndicator[1] = currentMillisLED;
          
        }
      
      break;
      
      case 12:

        if(currentMillisLED - previousMillisIndicator[1] >= IndicatorOnTime) {
          
          IndicatorState[0]++;
          IndicatorState[1] = 0;
          
        }
      
      break;
      
      case 13:
      
        for(int i = NUM_LEDS_BACK * IndicatorSize;i >= 0;i--) {
          
          ledsback[i] = idlecolback;
          SwitchFrontLedColor(IndicatorState[1],idlecolfront,1);
          
          IndicatorState[1]++; 
                        
        }
        
        FastLED.show();
        
        IndicatorState[0]++;
        previousMillisIndicator[2] = currentMillisLED;
      
      break;
      
      case 14:
      
        if(currentMillisLED - previousMillisIndicator[2] >= IndicatorOffTime) {
          
          IndicatorState[0] = 10;
          
          IndicatorInProgress[1] = 0;
          
        }
      
      break;
      
    }
    
  }

  if(dir == 0 || dir == 2) {    //Right Indicator 
  
    switch(IndicatorState[3]) {
      
      case 10:
      
        IndicatorState[3]++;
        IndicatorState[4] = 0;
        IndicatorState[5] = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;
      
      break;
      
      case 11:
      
        if(IndicatorState[5] <= NUM_LEDS_BACK && currentMillisLED - previousMillisIndicator[3] >= IndicatorAnimTime) {    //Animation
        
          ledsback[IndicatorState[5]] = indicatorcol;
          SwitchFrontLedColor(IndicatorState[4],indicatorcol,0);
          
          FastLED.show();
          
          IndicatorState[4]++;
          IndicatorState[5]++;

          previousMillisIndicator[3] = currentMillisLED;       
          
        } 
        
        if(IndicatorState[5] > NUM_LEDS_BACK) {
          
          IndicatorState[3]++;
          previousMillisIndicator[4] = currentMillisLED;
          
        }
      
      break;
      
      case 12:
      
        if(currentMillisLED - previousMillisIndicator[4] >= IndicatorOnTime) {
          
          IndicatorState[3]++;
          IndicatorState[4] = 0;
          
        }
      
      break;
      
      case 13:
      
        for(int i = NUM_LEDS_BACK - NUM_LEDS_BACK * IndicatorSize;i <= NUM_LEDS_BACK;i++) {
          ledsback[i] = idlecolback;
          SwitchFrontLedColor(IndicatorState[4],idlecolfront,0);
          IndicatorState[4]++;
        }
        
        FastLED.show();
        
        IndicatorState[3]++;
        previousMillisIndicator[5] = currentMillisLED;
      
      break;
      
      case 14:
        
        if(currentMillisLED - previousMillisIndicator[5] >= IndicatorOffTime) {
          
          IndicatorState[3] = 10;
          
          IndicatorInProgress[0] = 0;
          
        }
      
      break;
    }
    
  }
  
}

bool FadeToColor(unsigned long Color1, unsigned long Color2, unsigned int timeinms, int ledbegin, int ledend) {     //This function fades all the LEDs from Color1 to Color 2 in given time "timeinms" Color1 and Color2 should be HEX Values format: "0x000000"

  //Convert Color1 from HEX to RGB this is done by splitting the HEX Value in 3 Parts and converting them do decimal
  if(FadeToColorState == 0) {
    r1 = Color1 >> 16;

    g1 = (Color1 & 0x00ff00) >> 8;

    b1 = (Color1 & 0x0000ff);

    //Convert Color2 from HEX to RGB
    r2 = Color2 >> 16;

    g2 = (Color2 & 0x00ff00) >> 8;

    b2 = (Color2 & 0x0000ff);
    
    //Calsculate Differences between Color1 and Color2 RGB Values
    
    d1 = abs(r1 -r2);
    
    d2 = abs(g1 - g2);
    
    d3 = abs(b1 -b2);
  
    //Determine how many steps it takes to Fade from Color1 to Color2, the biggest differnce of the RGB Values is taken
    
    if(d1 >= d2 || d1 >= d3) {
      FadeToColorsteps = d1;
    }
    
    else if(d2 >= d1 || d2 >= d3) {
      FadeToColorsteps = d2;
    }
    
    else if(d3 >= d1 || d3 >= d2) {
      FadeToColorsteps = d3;
    }
  }
  
  //Fade the colors
  
  if(FadeToColorState <= FadeToColorsteps /*&& FadeToColorsteps > 0*/ ) {    
    if(currentMillisLED - previousMillisFadeToColor >= (timeinms / FadeToColorsteps)) {
      //Checks if current of r1 value is under, over or equals the final value r2 and adjusts it one step towards r2
      if(r1 > r2) {
        r1--;
      }
      
      else if(r1 == r2) {}
      
      else if(r1 < r2) {
        r1++;
      }
      
      //Checks if current of g1 value is under, over or equals the final value g2 and adjusts it one step towards g2
      if(g1 > g2) {
        g1--;
      }
      
      else if(g1 == g2) {}
      
      else if(g1 < g2) {
        g1++;
      }
      
      //Checks if current of b1 value is under, over or equals the final value b2 and adjusts it one step towards b2
      if(b1 > b2) {
        b1--;
      }
      
      else if(b1 == b2) {}
      
      else if(b1 < b2) {
        b1++;
      }
      
      for(int i = ledbegin; i <= ledend;i++) {   //All the LEDS are set to the current RGB Values
        ledsback[i].setRGB(r1, g1, b1);  
      }
      
      FastLED.show();
      
      FadeToColorState++;
      
      previousMillisFadeToColor = currentMillisLED;
      
      return false;
      
      //delay(timeinms / FadeToColorsteps);
      
    }
  }
  
  else {
    
    FadeToColorState = 0;
    return true;
    
  }
  
}

void SwitchFrontLedColor(int nrofled, long color, int dir) {        //dir = 1 for left, dir  0 for right, dir = 2 for both, dir = 3 for both without the 3rd row of leds

  if(dir == 2) {  //Both Sides will be affected
    
    ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    ledsfront[ledsfr3[constrain(nrofled,0,fr3length - 1)]] = color;
    ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    ledsfront[ledsfl3[constrain(nrofled,0,fl3length - 1)]] = color;
  }  

  if(dir == 0) {  //Only the right side will be affected
    
    //ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    //ledsfront[ledsfr3[constrain(nrofled,0,fr3length - 1)]] = color;
    
  }

  if(dir == 1) {  //Only the left side will be affected
    
    //ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    //ledsfront[ledsfl3[constrain(nrofled,0,fl3length - 1)]] = color;
  }
  
  if(dir == 3) {  //Both sides will be affected except the bottom row of leds
    
    ledsfront[ledsfr1[constrain(nrofled,0,fr1length - 1)]] = color;
    ledsfront[ledsfr2[constrain(nrofled,0,fr2length - 1)]] = color;
    ledsfront[ledsfl1[constrain(nrofled,0,fl1length - 1)]] = color;
    ledsfront[ledsfl2[constrain(nrofled,0,fl2length - 1)]] = color;
    
  }
  
}