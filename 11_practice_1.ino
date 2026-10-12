#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficent to convert duration to distance

#define _EMA_ALPHA 0.3    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// // Target Distance
// #define _TARGET_LOW  250.0
// #define _TARGET_HIGH 290.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
 
#define _DUTY_MIN 565 // servo full clockwise position (0 degree)
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2) // servo neutral position (90 degree)
#define _DUTY_MAX 2600 // servo full counterclockwise position (180 degree)

// global variables
float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX); 
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // EMA 초기값을 dist_prev로 설정
  dist_ema = dist_prev;

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float  dist_raw, dist_filtered;
  
  // wait until next sampling time.
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // the range filter
  // 범위 밖이면 LED 끄기(HIGH), 범위 안이면 LED 켜기(LOW)
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
      dist_filtered = dist_prev;
      digitalWrite(PIN_LED, HIGH);
  } else if (dist_raw < _DIST_MIN) {
      dist_filtered = dist_prev;
      digitalWrite(PIN_LED, HIGH);
  } else {    // In desired Range
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
      digitalWrite(PIN_LED, LOW);
  }

  // EMA 필터 적용
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0 - _EMA_ALPHA) * dist_ema;

  // 거리에 비례한 서보 제어
  // 18cm 이하: 0° / 18~36cm: 거리에 비례해서 0~180° / 36cm 이상: 180°
  float duty;
  if (dist_ema <= _DIST_MIN) {
    duty = _DUTY_MIN;
  } else if (dist_ema >= _DIST_MAX) {
    duty = _DUTY_MAX;
  } else {
    duty = _DUTY_MIN + (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN) * (_DUTY_MAX - _DUTY_MIN);
  }
  myservo.writeMicroseconds(duty);

  // output the distance to the serial port
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
