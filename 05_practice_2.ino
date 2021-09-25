#define PIN7 7

void setup() {
  pinMode(PIN7, OUTPUT);
  digitalWrite(PIN7, LOW);//LED ON
  delay(1000);  //1초동안 유지

}

void loop() {
  for(int i=0;i<5;i++){
    //1초동안 LED 5회 깜빡이도록
      digitalWrite(PIN7, HIGH);
      delay(100);
      digitalWrite(PIN7, LOW);
      delay(100);
  }

  while(1){
    digitalWrite(PIN7, HIGH); //LED 끄고 무한루프상태
  }


}
