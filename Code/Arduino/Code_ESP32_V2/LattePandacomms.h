
#ifndef LattePandacomms_h
#define LattePandacomms_h
#include <Arduino.h> 

#define maxnDataVar 32

struct Component {    //Struktur des Datenspeicherorts
  uint8_t Type;
  uint8_t nVal;
  uint8_t Access;
  float Data[maxnDataVar];
  uint8_t Error;
};

class LattePandacomms {
public:
  
  LattePandacomms();
  bool refresh(char startMarker, char endMarker);
  
  bool decode(char startMarker, char endMarker);
  
  bool send();

  Component IsOk;
  Component IMU1;
  Component IMU2;
  Component IMU3;
  Component Octosonar;
  Component Voltage;
  Component Temp;
  Component Fan;
  Component LED;

private:

  byte _bytesRecvd = 0;
  int _nb = 0;
  bool _inProgress = false;
  
  const int _ExpectedAccess[9] = {
    1,    //IMU1 Request only
    1,    //IMU2 Request only
    1,    //Octosonar Request only
    1,    //Voltage Sensors Request only
    1,    //Temperature Sensors Request only
    2,    //PWM Fan Speed --> Write to ESP --> confirm to LattePanda
    2,    //LEDs --> Write Effect to ESP --> confirm to LattePanda
    1,    //Error --> Write to ESP --> Send back to LattePanda
    1     //IMU3 Quaternions Request only
  };
  
  uint8_t Queue = 0b10000000;   //Queue for sending Data to LattePanda
                                //0b10000000 IMU1
                                //0b01000000 IMU2
                                //0b00100000 Octosonar
                                //0b00010000 Voltage Sensors
                                //0b00001000 Temp Sensors
                                //0b00000100 Fans
                                //0b00000010 LEDs
                                //0b00000001 IMU3

  int _commsError = 0;

};
#endif


