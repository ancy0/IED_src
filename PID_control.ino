#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 420

// Distance sensor
#define _DIST_ALPHA 0.25

// Servo range
#define _DUTY_MIN 1280
#define _DUTY_NEU 1527
#define _DUTY_MAX 1840

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 70

// Event periods
#define _INTERVAL_DIST 30
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 0.33
#define _KD 68.8
#define _KI 0.002

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target;
float dist_raw, dist_ema;
float alpha;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

int a, b; // unit: mm

// Median
float dist_median;
const int n = 4;
int readindex;
float arr[n];

void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  // initialize global variables
  dist_target = _DIST_TARGET;
  alpha = _DIST_ALPHA;
  dist_ema = 0;
  duty_curr = _DUTY_NEU;

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

  //initialize variables
  dist_median = 0.0;
  readindex = 0;

  //initialize arr[]
  for (int i = 0; i < n; i++) {
    arr[i] = 0;
  }

  // initialize serial port
  Serial.begin(57600);

  a = 69;
  b = 367;

  // convert angle speed into duty change per interval
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;
}

void loop() {
  /////////////////////
  // Event generator //
  /////////////////////

  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    event_dist = false;
    // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();

    // PID control logic
    error_curr = dist_target - dist_raw;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;

    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

    // Limit duty_target within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN)duty_target = _DUTY_MIN;
    if (duty_target > _DUTY_MAX)duty_target = _DUTY_MAX;

    // update error_prev
    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",D:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",I:");
    Serial.print(map(iterm, -1000, 1000, 510, 610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float dist_cali;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  dist_cali = 100 + 300.0 / (b - a) * (val - a);
  return dist_cali;
}

float ir_distance_filtered(void) { // return value unit: mm
  //float dist_prev;
  float dist_ir = ir_distance();

  // save dist_raw in the array
  arr[readindex] = dist_ir;

  // update readindex
  readindex = readindex + 1;

  if (readindex >= n) {
    readindex = 0;

    for (int i = 0; i < n; i++) { //sort
      for (int j = 0; j < n - i - 1; j++) {
        if (arr[i] > arr[i + 1]) {
          float temp = arr[i];
          arr[i] = arr[i + 1];
          arr[i + 1] = temp;
        }
      }
    }

    if (n % 2 == 1) { // return the median value
      int i = n / 2;
      dist_median = arr[i];
    }
    else {
      int i = n / 2;
      dist_median = (arr[i - 1] + arr[i]) / 2;
    }

  }
  dist_ema = _DIST_ALPHA * dist_median + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}
