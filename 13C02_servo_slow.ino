#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10

// configurable parameters for Servo
#define _DUTY_MIN 540  // servo full clock-wise position (0 degree)
#define _DUTY_NEU ((_DUTY_MIN + _DUTY_MAX) / 2) // servo neutral position (90 degree)
#define _DUTY_MAX 2550 // servo full counter-clockwise position (180 degree)

// ---- 실험 설정: 실험마다 아래 두 줄만 바꿔서 업로드 ----
// 실험 1: _SERVO_SPEED 3   , _MOVE_ANGLE 180  (60초 동안 180도 이동)
// 실험 2: _SERVO_SPEED 0.3 , _MOVE_ANGLE 90   (300초 동안 90도 이동)
#define _SERVO_SPEED 3   // servo angular speed (unit: degree/sec)
#define _MOVE_ANGLE  180 // total moving angle (unit: degree)

#define _POS_START _DUTY_MIN
#define _POS_END   (_DUTY_MIN + (_DUTY_MAX - _DUTY_MIN) * (_MOVE_ANGLE / 180.0))

// Loop Interval
#define INTERVAL 14 // servo update interval (unit: msec)

// global variables
unsigned long last_sampling_time; // unit: msec
unsigned long move_start_time;    // unit: msec

Servo myservo;

float duty_change_per_interval; // duty change per interval (소수점이 필요해서 float)
float duty_target;              // target duty time
float duty_curr;                // current duty time
bool  arrived = false;          // reached the target?

void setup() {
  myservo.attach(PIN_SERVO, _DUTY_MIN, _DUTY_MAX);

  duty_target = _POS_END;
  duty_curr = _POS_START;
  myservo.writeMicroseconds((int)duty_curr);

  Serial.begin(57600);

  // convert angular velocity into duty change per interval.
  duty_change_per_interval =
    (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);

  // 서보가 시작 위치로 가는 시간을 기다린 뒤 시작
  delay(2000);
  last_sampling_time = millis();
  move_start_time = millis();
}

void loop() {
  // wait until next sampling time.
  if (millis() < (last_sampling_time + INTERVAL))
    return;
  last_sampling_time += INTERVAL;

  // move duty_curr toward duty_target
  if (!arrived) {
    duty_curr += duty_change_per_interval;
    if (duty_curr >= duty_target) {
      duty_curr = duty_target;
      arrived = true;
      Serial.print("arrived! elapsed(sec): ");
      Serial.println((millis() - move_start_time) / 1000.0);
    }
  }

  // update servo position
  myservo.writeMicroseconds((int)(duty_curr + 0.5));

  // output the values to the serial port
  Serial.print("Min:");          Serial.print(_DUTY_MIN);
  Serial.print(",duty_curr:");   Serial.print(duty_curr);
  Serial.print(",duty_target:"); Serial.print(duty_target);
  Serial.print(",Max:");         Serial.println(_DUTY_MAX);
}
