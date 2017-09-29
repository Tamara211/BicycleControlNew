
int speed_pin = 5;
int next = '\x00';


void setup() {

  Serial.begin(115200);
  TCCR0B = (TCCR0B & 0b11111000) | 0x01;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW

  analogWrite(speed_pin, next);
}

void loop() {
  
  //Serial.print("\nwriting 10");
  //analogWrite(speed_pin,'\x49');

  //Serial.print("\nSpeed pin1: ");
  //analogWrite(speed_pin, next);

  if (Serial.available()){
    next = Serial.read();
    analogWrite(speed_pin,next);
    Serial.print("\nSpeed pin: ");
    Serial.print(next);
    
   
  }
}
