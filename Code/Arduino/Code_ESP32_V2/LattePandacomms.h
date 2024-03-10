
#ifndef LattePandacomms_h
#define LattePandacomms_h
#include <Arduino.h> 

#define maxnDataVar 32

struct Component {    //Struktur des Datenspeicherorts
  uint8_t Type;
  uint8_t nVal;
  uint8_t Access;
  uint8_t Data[maxnDataVar];
  uint8_t Error;
};

enum ComponentType {
  Type,
  nVal,
  Access,
  Data,
  Error
};

class LattePandacomms {
public:
  
  LattePandacomms();
  bool refresh(char startMarker, char endMarker);
  
  bool decode();
  
  bool send();
  
  //IMU1 get Function
  
  uint8_t getIMU1(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _IMU1.Type;
      case nVal:
        return _IMU1.nVal;
      case Access:
        return _IMU1.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _IMU1.Data[index];
        }
        break;
      case Error:
        return _IMU1.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }
  
  //IMU2 get Function
  
  uint8_t getIMU2(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _IMU2.Type;
      case nVal:
        return _IMU2.nVal;
      case Access:
        return _IMU2.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _IMU2.Data[index];
        }
        break;
      case Error:
        return _IMU2.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }
  
  //Octosonar get Function
  
  uint8_t getOctoSonar(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _octosonar.Type;
      case nVal:
        return _octosonar.nVal;
      case Access:
        return _octosonar.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _octosonar.Data[index];
        }
        break;
      case Error:
        return _octosonar.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }  

  uint8_t getVoltage(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _voltage.Type;
      case nVal:
        return _voltage.nVal;
      case Access:
        return _voltage.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _voltage.Data[index];
        }
        break;
      case Error:
        return _voltage.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }
  
  uint8_t getTemp(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _temp.Type;
      case nVal:
        return _temp.nVal;
      case Access:
        return _temp.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _temp.Data[index];
        }
        break;
      case Error:
        return _temp.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }  
  
  uint8_t getFan(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _fan.Type;
      case nVal:
        return _fan.nVal;
      case Access:
        return _fan.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _fan.Data[index];
        }
        break;
      case Error:
        return _fan.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }   
  
  uint8_t getLED(ComponentType component, int index = 0) {
    switch (component) {
      case Type:
        return _LED.Type;
      case nVal:
        return _LED.nVal;
      case Access:
        return _LED.Access;
      case Data:
        if (index >= 0 && index < maxnDataVar) {
          return _LED.Data[index];
        }
        break;
      case Error:
        return _LED.Error;
    }
    return 0; // or handle out-of-bounds error as needed
  }  

private:

  Component _error;
  Component _IMU1;
  Component _IMU2;
  Component _octosonar;
  Component _voltage;
  Component _temp;
  Component _fan;
  Component _LED;

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
  };
  
  int _commsError = 0;

};
#endif


