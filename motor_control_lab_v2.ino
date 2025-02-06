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

unsigned long lastSensorTime = 0;
unsigned long sensorTransmitDelay = 1000;

bool dc_pos = false; // flag if doing position control

// Current Motor Values
int current_stepper_angle = 0;
int dc_motor_speed = 0;
int dc_motor_position = 0;
int servo_angle = 0;

// Current Sensor Values
float pot_output = 0;
float ultrasonic_distance_cm = 0;
float prev_ultrasonic_distance_cm = 0;
float alpha = 0.8;
int desired_stepper_angle = 0;
bool slot_blocked = false; // false if unblocked, true if blocked 
int ir_data = 1;

bool gui_override = false;


AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);
Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);

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
    lastDebounceTime = millis();
  }
}

void s0() { // control motor velocity via pot
  if (micros() - prevTime > 10000){

  inputPosition = myEnc.read();
  double Kp = 5, Ki = 0.0, Kd = 0.0;
  setpointSpeed = map(analogRead(POT_PIN), 0, 1023, -800, 800); // Adjust range as needed

  // Get current motor speed
  unsigned long currTime = micros();
  double dTime = ((currTime - prevTime) / (1.0e6)); // Convert to seconds
  currentSpeed =  (inputPosition - prevPosition) / (dTime);
  dc_motor_speed = currentSpeed * 60 / (210 * 2);
  // Calculate PID output for speed control
  motorPower = control_dc_motor(setpointSpeed, currentSpeed, Kp, Ki, Kd);
  prevTime = currTime;

  prevPosition = inputPosition; 
  setMotorSpeed(motorPower);  
  }
}

void s1() { // control motor position via pot
  inputPosition = myEnc.read();
  // PID Gains for position control
  double Kp = 5, Ki = 0, Kd = 0.;
  setpointPosition = map(analogRead(POT_PIN), 0, 1023, -90, 90);
  dc_motor_position = inputPosition / 1.7;
  // Calculate PID output for position control
  motorPower = control_dc_motor(setpointPosition * 1.7, inputPosition, Kp, Ki, Kd);
  setMotorSpeed(motorPower);
}

void move_stepper_to_pos(int angle) {
  int steps = angle / 1.8 * 16;
  stepper.moveTo(steps);
  stepper.setSpeed(500);
  stepper.runSpeedToPosition();
  if (stepper.distanceToGo() == 0) current_stepper_angle = angle;
}

void s2() { // control stepper speed (steps/s) via ultrasonic distance
  float capped_distance = (ultrasonic_distance_cm < 30) ? ultrasonic_distance_cm : 40;
  int angle = map(capped_distance, 10, 30, -45, 45);
  move_stepper_to_pos(angle);

}

void s3() { // control servo via IR
  int curr_ir_data = read_irsensor();
  if (curr_ir_data == 30){
    ir_data = curr_ir_data;
    set_servo_angle(270);
    servo_angle = 270;
  }
  else if (curr_ir_data == 45){
    ir_data = curr_ir_data;
    servo_angle = 0;
    set_servo_angle(0);
  }
  
}

int desired_speed = 0;
int desired_position = 0;

void s4(String command) { // control all motors via GUI
  
  if (command.startsWith("STEPPER:")) {
    
    desired_stepper_angle = command.substring(8).toInt();
  }
  if (command.startsWith("SERVO:")) {
    servo_angle = command.substring(6).toInt();
  }
  if (command.startsWith("DC_POS:")) {
    desired_position = command.substring(8).toInt();
    if (dc_pos == false) {
      dc_motor_position = 0;
      dc_motor_speed = 0;
      myEnc.readAndReset();
    }
    dc_pos = true;
  }
  if (command.startsWith("DC_SPEED:")) {
    desired_speed = command.substring(9).toInt() * 210 * 60 / 100;
    dc_pos = false;
  }

  move_stepper_to_pos(desired_stepper_angle);
  set_servo_angle(servo_angle);  
  if (dc_pos) { 
    inputPosition = myEnc.read();
    // PID Gains for position control
    double Kp = 5, Ki = 0, Kd = 0.0;
    dc_motor_position = inputPosition / 1.7;
    // Calculate PID output for position control
    motorPower = control_dc_motor(desired_position * 1.7 * 2, inputPosition, Kp, Ki, Kd);
    setMotorSpeed(motorPower);
  } else {
    inputPosition = myEnc.read();
    double Kp = 5, Ki = 0.0, Kd = 0.0;
    unsigned long currTime = micros();
    double dTime = ((currTime - prevTime) / (1.0e6)); // Convert to seconds
    currentSpeed =  (inputPosition - prevPosition) / (dTime);
    dc_motor_speed = currentSpeed * 60 / (210 * 2);
    // Calculate PID output for speed control
    motorPower = control_dc_motor(desired_speed, currentSpeed, Kp, Ki, Kd);
    prevTime = currTime;

    prevPosition = inputPosition; 
    setMotorSpeed(motorPower); 
  }
}

String read_serial_port() {
  String command = "";
  if (Serial.available() > 0) {
    delay(10);
    while (Serial.available() > 0) {
      char c = Serial.read();
      // Break if it's the end of the line
      if (c == '\n') break;
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
  myEnc.readAndReset();

}

void loop() {
  if (!responseReceived) {
    ping_gui();
  } else {
      // read sensors
    pot_output = read_potentiometer(POT_PIN);
    ultrasonic_distance_cm = read_ultrasonic(ULTRASONIC_PIN);
    slot_blocked = read_slot(SLOT_PIN);
    
    // Send sensor data
    // Format = "SENSOR_DATA:pot_output,ultrasonic_distance_cm,ir_data,slot_blocked"
    if ((millis() - lastSensorTime) > sensorTransmitDelay) {
      // TODO: Need to send motor data as well
      Serial.println("SENSOR_DATA:" + String(pot_output) + "," + String(ultrasonic_distance_cm) + "," + String(ir_data) + "," + String(slot_blocked) + "," +String(dc_motor_speed) + "," + String(dc_motor_position) + "," + String(current_stepper_angle) + "," + String(servo_angle));
      lastSensorTime = millis();
      delay(10);

    }

    String command = read_serial_port();

        
    if (command == "OVERRIDE:ON") {
      gui_override = true;
      prev_gui_state = state;
      state = 4;
      dc_motor_position = 0;
      myEnc.readAndReset();

      Serial.println("Override ON, waiting for command");
    } else if (command == "OVERRIDE:OFF"){
      gui_override = false;
      state = prev_gui_state;

      Serial.println("Override OFF");
      dc_motor_position = 0;
      myEnc.readAndReset();
    }


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
        s4(command);
        break;
    }
  }
}