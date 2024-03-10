
#ifndef LattePandacomms_h
#define LattePandacomms_h
#include <Arduino.h> 

/*struct Package {
  uint8_t Type;
  uint8_t nVal;
  uint8_t Access;
  uint8_t Data[32];
  uint8_t Error;
}*/

class LattePandacomms {
public:
  
  LattePandacomms();
  bool refresh(char startMarker, char endMarker);
  
  bool decode();
  
  bool send();
  
private:

  byte _bytesRecvd = 0;
  int _nb = 0;
  bool _inProgress = false;
  
  const int _ExpectedAccess[8] = {
    1,    //IMU1 Request only
    1,    //IMU2 Request only
    1,    //Octosonar Request only
    1,    //Voltage Sensors Request only
    1,    //Temperature Sensors Request only
    2,    //PWM Fan Speed --> Write to ESP --> confirm to LattePanda
    2,    //LEDs --> Write Effect to ESP --> confirm to LattePanda
    1,    //Error --> Write to ESP --> Send back to LattePanda
  }
  
  int _commsError = 0;

};
#endif


