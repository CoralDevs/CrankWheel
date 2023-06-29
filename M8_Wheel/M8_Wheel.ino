#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Bounce2.h>

#define MOTOR_ENCODER_A 2
#define MOTOR_ENCODER_B 3

#define ENCODER_CONSTANT 2
#define WHEEL_DIAMETER_CM 6.5
#define WHEEL_DISTANCE_CM 14.5

#define BUTTON_PIN 4
Bounce debouncer = Bounce();

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;

const float defaultEncoderPosValue = (8600 - 5000) * ENCODER_CONSTANT;
volatile long encoder0pos = defaultEncoderPosValue;

boolean doCounterForce = true;
boolean doingSpin = false;

float outputValue = 0;

int force = map(defaultEncoderPosValue, 100, 0, 50, 215);

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_ENCODER_A, INPUT);
  digitalWrite(MOTOR_ENCODER_A, HIGH);
  pinMode(MOTOR_ENCODER_B, INPUT);
  digitalWrite(MOTOR_ENCODER_B, HIGH);

  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), setEncoderValue, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_B), setEncoderValue, CHANGE);

  pinMode(9, OUTPUT); // PWM
  pinMode(10, OUTPUT); // P1
  pinMode(11, OUTPUT); // P2

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(1000);

  setMotor(1, 100, 9, 10, 11);
  delay(250);
  setMotor(-1, 100, 9, 10, 11);
  delay(250);
  setMotor(0, 0, 9, 10, 11);
}

void loop() {
  if (!doingSpin) setEncoderValue(); // Update encoder value
  outputValue = constrain(map((encoder0pos / ENCODER_CONSTANT), -5000, 5000, 0, 10000), 0, 10000)/100.0;
  force = map(outputValue, 100, 0, 50, 215);

  debouncer.update();


  // Check for manual rotation
  static unsigned long lastUpdateTime = millis();
  static bool forceApplied = false;

  if (doCounterForce) {
    int dir = checkManualRotation();
    if (dir < 0 && !forceApplied) { // clockwise
      setMotor(1, force   , 9, 10, 11); // Apply counter force.
      forceApplied = true;
      //delay(100);
    } else {
      setMotor(0, 0, 9, 10, 11);
      forceApplied = false;
    }
    lastUpdateTime = millis();
  }

  // Check for serial command to trigger spin
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 's') {
      spinClockwiseReset();
    }
  }

  // Print encoder values
  // Serial.print("Encoder: ");
  Serial.print(int(outputValue*100));
  Serial.print('\t');
  Serial.print(1 - debouncer.read());
  Serial.print('\n');
}

void setEncoderValue() {
  pinAState = digitalRead(MOTOR_ENCODER_A);
  pinBState = digitalRead(MOTOR_ENCODER_B);

  if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0)  // forward
      encoder0pos++;
    if (pinAStateOld == 0 && pinBStateOld == 1)  // reverse
      encoder0pos--;
  }
  if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0)  // forward
      encoder0pos++;
    if (pinAStateOld == 1 && pinBStateOld == 1)  // reverse
      encoder0pos--;
  }
  if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1)  // forward
      encoder0pos++;
    if (pinAStateOld == 1 && pinBStateOld == 0)  // reverse
      encoder0pos--;
  }

  if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1)  // forward
      encoder0pos++;
    if (pinAStateOld == 0 && pinBStateOld == 0)  // reverse
      encoder0pos--;
  }

  pinAStateOld = pinAState;
  pinBStateOld = pinBState;

  if (encoder0pos - 300 >= (5000 * ENCODER_CONSTANT)) {
    encoder0pos = 5000 * ENCODER_CONSTANT;  // Set to maximum value
  } else if (encoder0pos + 300 <= (-5000 * ENCODER_CONSTANT)) {
    encoder0pos = -5000 * ENCODER_CONSTANT; // Set to minimum value
  }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (dir == -1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

int checkManualRotation() {
  static long lastPos = 0;
  long currentPos = encoder0pos;
  int dir = 0;

  if (currentPos < lastPos - 1) {
    dir = -1;
  }
  else if (currentPos > lastPos) {
    dir = 1;
  }

  lastPos = currentPos;
  return dir;
}

void spinClockwiseReset() {
  unsigned long startTime = millis();
  setMotor(1, 125, 9, 10, 11); // Spin clockwise
  doCounterForce = false;
  doingSpin = true;
  delay(1000);
  encoder0pos = defaultEncoderPosValue;
  doCounterForce = true;
  setMotor(0, 0, 9, 10, 11); // Stop the motor
  delay(1250);
  encoder0pos = defaultEncoderPosValue;
  doingSpin = false;
}
