#include <Arduino.h>
#include "LattePandacomms.h"

#define _maxMessage 256
//#define startMarker 'x'   //Marks Begining of Datastream   ASCII for x 0x78
//#define endMarker 'q'    //Marks End of Datastream   ASCII for q 0x71

char BufferIn[_maxMessage];    //Buffer for incoming messages
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

bool removeMarkers(char* inputMessage, char* outputMessage, char startMarker, char endMarker) {   //Removes the startMakrer and endMarker Markers from the message
  
  int startIndex = 0;
  int endIndex = 0;

  for (int i = 0; i < strlen(inputMessage); i++) {
      if (inputMessage[i] == startMarker) {
        startIndex = i + 1;     // Skip startMarker
      } 
      else if (inputMessage[i] == endMarker) {
         endIndex = i;
         break;    // Stop once endMarker is found
      }
  }
  
  if (endIndex > startIndex) {    // Extract the substring between startMarker and endMarker into the output message
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

bool LattePandacomms::decode(char startMarker, char endMarker) {
  
  uint8_t Type = 0;
  uint8_t nVal = 0;
  uint8_t Access = 0;
  uint8_t Data[maxnDataVar] = {0};
  uint8_t Error = 0;
  
  removeMarkers(BufferIn, decodingBuffer, startMarker, endMarker);   //Removes the "x" and "q" Markers from the message
  
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
      
        IsOk.Type = Type;
        IsOk.nVal = nVal;
        IsOk.Access = Access;
        for(int i = 0; i < IsOk.nVal; i++) {
          IsOk.Data[i] = Data[i];
        }
        IsOk.Error = Error;
      
      break;
      
      case 1:
      
        IMU1.Type = Type;
        IMU1.nVal = nVal;
        IMU1.Access = Access;
        for(int i = 0; i < IMU1.nVal; i++) {
          IMU1.Data[i] = Data[i];
        }
        IMU1.Error = Error;
      
      break;
      
      case 2:
      
        IMU2.Type = Type;
        IMU2.nVal = nVal;
        IMU2.Access = Access;
        for(int i = 0; i < IMU2.nVal; i++) {
          IMU2.Data[i] = Data[i];
        }
        IMU2.Error = Error;
      
      break;
      
      case 3: 
      
        Octosonar.Type = Type;
        Octosonar.nVal = nVal;
        Octosonar.Access = Access;
        for(int i = 0; i < Octosonar.nVal; i++) {
          Octosonar.Data[i] = Data[i];
        }
        Octosonar.Error = Error;
      
      break;
      
      case 4: 
      
        Voltage.Type = Type;
        Voltage.nVal = nVal;
        Voltage.Access = Access;
        for(int i = 0; i < Voltage.nVal; i++) {
          Voltage.Data[i] = Data[i];
        }
        Voltage.Error = Error;
      
      break;
      
      case 5:
      
        Temp.Type = Type;
        Temp.nVal = nVal;
        Temp.Access = Access;
        for(int i = 0; i < Temp.nVal; i++) {
          Temp.Data[i] = Data[i];
        }
        Temp.Error = Error;
      
      break;
      
      case 6:
      
        Fan.Type = Type;
        Fan.nVal = nVal;
        Fan.Access = Access;
        for(int i = 0; i < Fan.nVal; i++) {
          Fan.Data[i] = Data[i];
        }
        Fan.Error = Error;
      
      break;
      
      case 7:
      
        LED.Type = Type;
        LED.nVal = nVal;
        LED.Access = Access;
        for(int i = 0; i < LED.nVal; i++) {
          LED.Data[i] = Data[i];
        }
        LED.Error = Error;
      
      break;
      
    }  
    
  } 

  /*
  Serial.println(IMU1.Type);             //DEBUG
  Serial.println(IMU1.nVal);
  Serial.println(IMU1.Access);
  for (int i = 0; i < IMU1.nVal; ++i) {
    Serial.println(IMU1.Data[i]);
  }
  Serial.println(IMU1.Error);
  Serial.println("");
  */
  
  return 0;
  
}



bool LattePandacomms::send() {
  
  
  
  return 0;
  
}