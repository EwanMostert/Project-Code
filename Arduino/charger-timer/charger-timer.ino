const int en_pin = 9;
const int start_time = millis();

void setup() {
  // put your setup code here, to run once:  
  pinMode(en_pin, OUTPUT);
  digitalWrite(en_pin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  int current_time = millis();
  int led_on = millis();
  if((current_time - start_time) >= 10800000){
    digitalWrite(en_pin, LOW);
  }
  if((led_on % 1000 == 0)){
    if()
  }
}
