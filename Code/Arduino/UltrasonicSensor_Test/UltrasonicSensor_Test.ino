#define echoPin 19 // attach pin GPIO18 to pin Echo of JSN-SR04
#define trigPin 18  // attach pin GPIO5 ESP32 to pin Trig of JSN-SR04                     




long duration; // Variable to store time taken to the pulse
// to reach the receiver


int distance; // Variable to store distance calculated using
// formula


void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT


  // Serial Communication is starting with 9600 of
  // baudrate speed
  Serial.begin(115200);


  // The text to be printed in the serial monitor
  Serial.println("Distance measurement using JSN-SR04T");
  delay(500);
}


void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); // wait for 2 ms to avoid
  // collision in the serial monitor


  digitalWrite(trigPin, HIGH); // turn on the Trigger to generate pulse
  delayMicroseconds(10); // keep the trigger "ON" for 10 ms to generate
  // pulse for 10 ms.


  digitalWrite(trigPin, LOW); // Turn off the pulse trigger to stop
  // pulse generation


  // If pulse reached the receiver echoPin
  // become high Then pulseIn() returns the
  // time taken by the pulse to reach the
  // receiver


  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.0344 / 2; // Expression to calculate
  // distance using time


  Serial.print("Distance: ");
  Serial.print(distance); // Print the output in serial monitor
  Serial.println(" cm");
  delay(100);
}