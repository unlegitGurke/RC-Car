int LED_White = 13;
int LED_Blinker_Rechts = 12;
int LED_Blinker_Links = 10;
int LED_Ruecklicht = 11;
int ESC_Pin = 9;
int Bremslicht = 8;

int Blinker_rechts_an = 0;
int Blinker_links_an = 1;
int Bremst;

void setup() {
  pinMode(LED_White, OUTPUT);
  pinMode(LED_Blinker_Rechts, OUTPUT);
  pinMode(LED_Blinker_Links, OUTPUT);
  pinMode(LED_Ruecklicht, OUTPUT);
  pinMode(ESC_Pin, INPUT_PULLUP);
  
  digitalWrite(LED_White, HIGH);
  digitalWrite(LED_Blinker_Rechts, LOW);
  digitalWrite(LED_Blinker_Links, LOW);
  digitalWrite(LED_Ruecklicht, HIGH);
  
}

void loop() {
  if(Blinker_rechts_an == 1) {
    Blinker_rechts();
  }
  else {
    
  }
  if(Blinker_links_an == 1) {
    Blinker_links();
  }
  else {
    
  }
  if(analogRead(ESC_Pin) == HIGH) {
    digitalWrite(Bremslicht, HIGH)
  }
  else {
    digitalWrite(Bremslicht, LOW)
  }
}

void Blinker_rechts() {
  digitalWrite(LED_Blinker_Rechts, HIGH);
  delay(200);
  digitalWrite(LED_Blinker_Rechts, LOW);
  delay(200);
}

void Blinker_links() {
  digitalWrite(LED_Blinker_Links, HIGH);
  delay(200);
  digitalWrite(LED_Blinker_Links, LOW);
  delay(200);
}
