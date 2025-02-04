#include <AccelStepper.h>
#include <Encoder.h>

#include "ultrasonic.h"
#include "potentiometer.h"
#include "servo.h"
#include "slot.h"
#include "dc_motor.h"

#define ULTRASONIC_PIN A0
#define POT_PIN A2
#define SLOT_PIN A5
//#define SERVO_PIN 11
// #define IR_PIN 7
#define BUTTON_PIN 3
#define STEPPER_DIR_PIN 6
#define STEPPER_STEP_PIN 5
#define ENCODER_PIN_A 2 
#define ENCODER_PIN_B 9

volatile int state = 0; // 8 total states
int prev_state;

volatile int prev_gui_state = 0; // keep track of state in case gui used
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 200;

// Current Motor Values
int current_stepper_angle = 0;
int dc_motor_speed = 0;
int dc_motor_position = 0;

// Current Sensor Values
float pot_output = 0;
float ultrasonic_distance_cm = 0;
bool slot_blocked = false; // false if unblocked, true if blocked 

bool gui_override = false;

double setpointPosition = 0,prevPosition = 0, inputPosition = 0;
unsigned long previousTime = 0, prevTime = 0;
double setpointSpeed = 0, currentSpeed = 0;
double integral_error = 0, prevError = 0;
double outputPosition = 0;

AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);

// s0 lets you change dc motor velocity, s1 lets you change dc motor position
// s2 lets you change stepper motor position, s3 lets you change servo motor position

// s4 allows for override

void change_state() {
  if (!gui_override && (millis() - lastDebounceTime) > debounceDelay) {
    // next state behavior (see diagram)
    prev_state = state;
    if (state == 0 || state == 1) {
      state = 2;
    } else if (state == 2) {
      state = 3;
    } else if (state == 3) {
      state = 0;
      dc_motor_position = 0;
      myEnc.readAndReset();
    }
    Serial.print("Current State: ");
    Serial.println(state);
    lastDebounceTime = millis();
  }
}

void s0() { // control motor velocity via pot
  // PID Gains for speed control
  unsigned long currTime = micros();

  if(currTime - prevTime > 100000){
    double Kp = 0.5, Ki = 0.8, Kd = 0.0;
    double setpointSpeed = map(pot_output, 0, 1023, -800, 800); // Adjust range as needed

    // Get current motor speed
    double dTime = ((currTime - prevTime) / (1.0e6)); // Convert to seconds
    dc_motor_speed =  (inputPosition - prevPosition) / (dTime);

    // Calculate PID output for speed control
    int motorpower = control_dc_motor(setpointSpeed, dc_motor_speed, Kp, Ki, Kd);
    prevTime = currTime;

    setMotorSpeed(motorpower);
  }
}

void s1() { // control motor position via pot
  int setpoint = map(pot_output, 0, 1023, -500, 500);
  dc_motor_position = myEnc.read();
  int motorpower = control_dc_motor(setpoint, dc_motor_position,  1.5, 0.1, 0.0);
  setMotorSpeed(motorpower);
}

void s2() { // control stepper speed (steps/s) via ultrasonic distance
  //TODO
  float capped_distance = (ultrasonic_distance_cm < 30) ? ultrasonic_distance_cm : 40;
  int angle = map(capped_distance, 10, 30, -45, 45);
  int steps = angle / 1.8 * 16;
  stepper.moveTo(steps);
  stepper.setSpeed(500);
  stepper.runSpeedToPosition();
  if (stepper.distanceToGo() == 0) current_stepper_angle = angle;
}

void s3() { // control servo via IR
  //TODO
}

void s4() { // control all motors via GUI
  //TODO

}

void setup() {
  Serial.begin(9600);
  setup_slot(SLOT_PIN);
  setup_potentiometer(POT_PIN);
  setup_ultrasonic(ULTRASONIC_PIN);
  //setup_servo(SERVO_PIN);
  setup_dc_motor();
  stepper.setMaxSpeed(1000);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), change_state, RISING);

}

void loop() {
  // read sensors
  pot_output = read_potentiometer(POT_PIN);
  ultrasonic_distance_cm = read_ultrasonic(ULTRASONIC_PIN);
  slot_blocked = read_slot(SLOT_PIN);

  // State selection behavior
  // Need to clean up how to get in and out of s4 based on if gui override is active
  // if (gui_state_selector != 0) { // if gui is actively trying to override, set to associated state
  //   state = gui_state_selector;
  //   gui_override = true;
  // } else if (gui_override && gui_state_selector == 0) { // if gui no longer trying to override, revert to state prior to gui
  //   state = prev_gui_state;
  //   gui_override = false;
  // }

  if (state == 0 && slot_blocked) {
    state = 1;
    dc_motor_position = 0;
    myEnc.readAndReset();
  }
  else if (state == 1 && !slot_blocked) state = 0;

  // state behavior
  switch (state) {
    case 0:
      s0();
      break;
    case 1:
      s1();
      break;
    case 2:
      s2();
      break;
    case 3:
      s3();
      break;
    case 4:
      s4(); // should we have one override state, that allows you to control all motors at the same time?
      break;
  }
  if (state <= 3) prev_gui_state = state; // store current state in case gui overrides
}