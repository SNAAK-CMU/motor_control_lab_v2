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
#define SERVO_PIN 10
#define IR_PIN 7
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
int ir_data = 1;

bool gui_override = false;

// double setpointPosition = 0,prevPosition = 0, inputPosition = 0;
// unsigned long previousTime = 0, prevTime = 0;
// double setpointSpeed = 0, currentSpeed = 0;
// double integral_error = 0, prevError = 0;
// double outputPosition = 0;
// int motorpower=0;

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
    // Serial.print("Current State: ");
    // Serial.println(state);
    lastDebounceTime = millis();
  }
}

void s0() { // control motor velocity via pot
  if (micros() - prevTime > 10000){

  inputPosition = myEnc.read();
  double Kp = 0.5, Ki = 0.8, Kd = 0.0;
  setpointSpeed = map(analogRead(POT_PIN), 0, 1023, -800, 800); // Adjust range as needed

  // Get current motor speed
  unsigned long currTime = micros();
  double dTime = ((currTime - prevTime) / (1.0e6)); // Convert to seconds
  currentSpeed =  (inputPosition - prevPosition) / (dTime);

  // Calculate PID output for speed control
  motorPower = control_dc_motor(setpointSpeed, currentSpeed, Kp, Ki, Kd);
  prevTime = currTime;

  prevPosition = inputPosition; 
  setMotorSpeed(motorPower);  


    // Debug prints for speed control
  Serial.print("Setpoint: ");
  Serial.print(setpointSpeed);
  Serial.print(" | Current Speed: ");
  Serial.print(currentSpeed);
  Serial.print(" | Output: ");
  Serial.println(motorPower);

  }
}

void s1() { // control motor position via pot
  inputPosition = myEnc.read();
  // PID Gains for position control
  double Kp = 1.5, Ki = 0.1, Kd = 0.0;
  setpointPosition = map(analogRead(POT_PIN), 0, 1023, -500, 500);

  // Calculate PID output for position control
  motorPower = control_dc_motor(setpointPosition, inputPosition, Kp, Ki, Kd);
  setMotorSpeed(motorPower);
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
  ir_data = read_irsensor();
  if (ir_data == 1){
    set_servo_angle(270);
  }
  else if (ir_data == 0){
    set_servo_angle(0);
  }
  else{
    Serial.print("cant read IR");
  }
  // set_servo_angle(0);
  // delay(1000);
  // 
  // delay(1000);
  // Serial.print();
  delay(1000);
  
}

void s4() { // control all motors via GUI
  //TODO

}

String read_serial_port() {
  String command = "";
  if (Serial.available() > 0) {
    // Wait a bit for the entire message to arrive
    delay(10);
    // Read all the available characters
    while (Serial.available() > 0) {
      char c = Serial.read();
      // Break if it's the end of the line
      if (c == '\n') break;
      // Otherwise, append the character to the command string
      command += c;
    }
    // Trim any leading or trailing whitespace
    command.trim();
  }
  return command;
}

bool responseReceived = false;

void ping_gui() {
  if (!responseReceived) {
    Serial.println("PING");
    delay(100);  // Wait a bit before checking for response

    String response = read_serial_port();
    if (response == "PONG") {
      responseReceived = true;
      Serial.println("GUI Connection Established!!");
    }
  }
}

void setup() {
  Serial.begin(9600);
  setup_slot(SLOT_PIN);
  setup_potentiometer(POT_PIN);
  setup_ultrasonic(ULTRASONIC_PIN);
  setup_servo(SERVO_PIN);
  setup_dc_motor();
  setup_irsensor(IR_PIN);
  stepper.setMaxSpeed(1000);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), change_state, RISING);

}

void loop() {
  responseReceived = true; //REMOVE
  if (!responseReceived) {
    ping_gui();
  } 
  else {
      // read sensors
    pot_output = read_potentiometer(POT_PIN);
    ultrasonic_distance_cm = read_ultrasonic(ULTRASONIC_PIN);
    slot_blocked = read_slot(SLOT_PIN);
    ir_data = 10 ;
    
    // Send sensor data
    // Format = "SENSOR_DATA:pot_output,ultrasonic_distance_cm,ir_data,slot_blocked"
    // Serial.print("SENSOR_DATA:");
    // Serial.print(pot_output);
    // Serial.print(",");
    // Serial.print(ultrasonic_distance_cm);
    // Serial.print(",");
    // Serial.print(ir_data);
    // Serial.print(",");
    // Serial.println(slot_blocked);

    // prevent flooding port
    // delay(100);

    
    // read serial port
    String command = read_serial_port();

    // delay(150); // prevent flooding port

        
    // Check if the command is "OVERRIDE:ON"
    if (command == "OVERRIDE:ON") {
      // change state to s4 (@oliver)
      Serial.println("Override ON, waiting for command");
    }
    if (command == "OVERRIDE:OFF"){
      // change back to prev_state (@oliver)
      Serial.println("Override OFF");
    }

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
  
}