#include <Arduino.h>
#include "LattePandacomms.h"

#define _maxMessage 256
//#define startMarker 'x'   //Marks Begining of Datastream   ASCII for x 0x78
//#define endMarker 'q'    //Marks End of Datastream   ASCII for q 0x71

char BufferIn[_maxMessage] = "x1,6,2,91,92,93,94,95,96,1q";    //Buffer for incoming messages
char BufferOut[_maxMessage];   //Buffer for outgoing messages

char decodingBuffer[_maxMessage];

LattePandacomms::LattePandacomms() {
  
  
  
}

bool LattePandacomms::refresh(char startMarker, char endMarker) {
  
  _inProgress = false;
  
  while (Serial.available() > 0) {    //Checks for Serial Data
    
    byte x = Serial.read();   //Reads Serial Data
    
    if (x == startMarker && _inProgress == false) {    //IF the first Byte = startMarker, start to recieve Data
       
      _bytesRecvd = 0;      //Zero bytes have been recieved so far
      _inProgress = true;   //New Message has started
      
      for(int i = 0; i < sizeof(BufferIn); i++) {   //Clear BufferIn Array to make space for new message
        BufferIn[i] = '\0';
      }
      
    }
    
    if (_inProgress ) {   //Has the message started?
          
      if (_bytesRecvd<_maxMessage) {    //Does the current length exceed the max Message limit
        
        BufferIn[_bytesRecvd++] = x;
        
      }
      
      if (x == endMarker) { 
        
        _inProgress = false;
        //allReceived = true;
        _nb = _bytesRecvd;
        return 1;
        
      }
      
    }
    
  }
  
  //Serial.println(BufferIn);   //DEBUG
  return 0;
  
}

bool removeMarkers(char* inputMessage, char* outputMessage) {   //Removes the "x" and "q" Markers from the message
  
  int startIndex = 0;
  int endIndex = 0;

  for (int i = 0; i < strlen(inputMessage); i++) {
      if (inputMessage[i] == 'x') {
        startIndex = i + 1;     // Skip startMarker
      } 
      else if (inputMessage[i] == 'q') {
         endIndex = i;
         break;    // Stop once endMarker is found
      }
  }
  
  if (endIndex > startIndex) {    // Extract the substring between 'x' and 'q' into the output message
    strncpy(outputMessage, inputMessage + startIndex, endIndex - startIndex);
    outputMessage[endIndex - startIndex] = '\0';    // Null-terminate the output message
    return true;
  } 
  
  else {
    outputMessage[0] = '\0'; // Empty output message if 'q' is not found after 'x'
    return false;
  }  
  
}

bool removeFirstValue(char* inputMessage, char* outputMessage) {    //Removes the first valie from a message
    
  int commaIndex = 0;
  
  while (inputMessage[commaIndex] != ',' && inputMessage[commaIndex] != '\0') {   //Search for first comma
    commaIndex++;
  }
  
  if (inputMessage[commaIndex] == ',') {    // If a comma is found, copy the substring after the first comma into the output message
    strcpy(outputMessage, inputMessage + commaIndex + 1);
    return true;
  } 
  
  else {
    return false;
  }
  
}

bool LattePandacomms::decode() {
  
  uint8_t Type = 0;
  uint8_t nVal = 0;
  uint8_t Access = 0;
  uint8_t Data[maxnDataVar] = {0};
  uint8_t Error = 0;
  
  removeMarkers(BufferIn, decodingBuffer);   //Removes the "x" and "q" Markers from the message
  
  sscanf(decodingBuffer, "%hhu,%hhu,%hhu", &Type, &nVal, &Access);    //Parse the first 3 variables from message: Type, nVal and Access
  
  for(int i = 0; i < 3; i++) {                          //Remove parsed values from buffer
    removeFirstValue(decodingBuffer, decodingBuffer);
  }
  
  for(int i = 0; i < nVal; i++) {                       //Parse Data Variables and remove them from buffer
    
    sscanf(decodingBuffer, "%u", &Data[i]);
    removeFirstValue(decodingBuffer, decodingBuffer);
    
  }
  
  sscanf(decodingBuffer, "%hhu", &Error);     //Parse the Error variable.
  
  //Write Data to Data structure
  
  if(Access == 0 || Access == 2) {
    
    switch(Type) {
      
      case 0:
      
        _error.Type = Type;
        _error.nVal = nVal;
        _error.Access = Access;
        for(int i = 0; i < _error.nVal; i++) {
          _error.Data[i] = Data[i];
        }
        _error.Error = Error;
      
      break;
      
      case 1:
      
        _IMU1.Type = Type;
        _IMU1.nVal = nVal;
        _IMU1.Access = Access;
        for(int i = 0; i < _IMU1.nVal; i++) {
          _IMU1.Data[i] = Data[i];
        }
        _IMU1.Error = Error;
      
      break;
      
      case 2:
      
        _IMU2.Type = Type;
        _IMU2.nVal = nVal;
        _IMU2.Access = Access;
        for(int i = 0; i < _IMU2.nVal; i++) {
          _IMU2.Data[i] = Data[i];
        }
        _IMU2.Error = Error;
      
      break;
      
      case 3: 
      
        _octosonar.Type = Type;
        _octosonar.nVal = nVal;
        _octosonar.Access = Access;
        for(int i = 0; i < _octosonar.nVal; i++) {
          _octosonar.Data[i] = Data[i];
        }
        _octosonar.Error = Error;
      
      break;
      
      case 4: 
      
        _voltage.Type = Type;
        _voltage.nVal = nVal;
        _voltage.Access = Access;
        for(int i = 0; i < _voltage.nVal; i++) {
          _voltage.Data[i] = Data[i];
        }
        _voltage.Error = Error;
      
      break;
      
      case 5:
      
        _temp.Type = Type;
        _temp.nVal = nVal;
        _temp.Access = Access;
        for(int i = 0; i < _temp.nVal; i++) {
          _temp.Data[i] = Data[i];
        }
        _temp.Error = Error;
      
      break;
      
      case 6:
      
        _fan.Type = Type;
        _fan.nVal = nVal;
        _fan.Access = Access;
        for(int i = 0; i < _fan.nVal; i++) {
          _fan.Data[i] = Data[i];
        }
        _fan.Error = Error;
      
      break;
      
      case 7:
      
        _LED.Type = Type;
        _LED.nVal = nVal;
        _LED.Access = Access;
        for(int i = 0; i < _LED.nVal; i++) {
          _LED.Data[i] = Data[i];
        }
        _LED.Error = Error;
      
      break;
      
    }  
    
  } 

  /*
  Serial.println(_IMU1.Type);             //DEBUG
  Serial.println(_IMU1.nVal);
  Serial.println(_IMU1.Access);
  for (int i = 0; i < _IMU1.nVal; ++i) {
    Serial.println(_IMU1.Data[i]);
  }
  Serial.println(_IMU1.Error);
  Serial.println("");
  */
  
  return 0;
  
}



bool LattePandacomms::send() {
  
  
  
  return 0;
  
}