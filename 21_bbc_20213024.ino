#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN 1114
#define _DUTY_NEU 1476
#define _DUTY_MAX 1834

// global variables
Servo myservo;

int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(9600);

  a = 64;
  b = 436;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

  if(dist_cali < 255.0) {
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  else {
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  
  delay(1000);
}
