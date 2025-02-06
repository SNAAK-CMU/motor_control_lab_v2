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

bool slot_blocked = false; // false if unblocked, true if blocked 
int ir_data = 1;

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
    lastDebounceTime = millis();
  }
}

void s0() { // control motor velocity via pot
  dc_motor_speed = map(pot_output, 0, 1023, -500, 500);
  setMotorSpeed(dc_motor_speed);
}

void s1() { // control motor position via pot
  int setpoint = map(pot_output, 0, 1023, -500, 500);
  dc_motor_position = myEnc.read();
  int motorpower = control_dc_motor(setpoint, dc_motor_position,  1.5, 0.1, 0.0);
  setMotorSpeed(motorpower);
}

void move_stepper_to_pos(int angle) {
  int steps = angle / 1.8 * 16;
  stepper.moveTo(steps);
  stepper.setSpeed(500);
  stepper.runSpeedToPosition();
  if (stepper.distanceToGo() == 0) current_stepper_angle = angle;
}

void s2() { // control stepper speed (steps/s) via ultrasonic distance
  //TODO
  float capped_distance = (ultrasonic_distance_cm < 30) ? ultrasonic_distance_cm : 40;
  int angle = map(capped_distance, 10, 30, -45, 45);
  move_stepper_to_pos(angle);
}

void s3() { // control servo via IR
  //TODO
  ir_data = read_irsensor();
  if (ir_data == 1){
    set_servo_angle(270);
    servo_angle = 270;
  }
  else if (ir_data == 0){
    servo_angle = 0;
    set_servo_angle(0);
  }
  else{
    Serial.print("cant read IR");
  }
  delay(100);
  
}

void s4(String command) { // control all motors via GUI
  //TODO
  bool dc_pos = false; // flag if doing position control
  int desired_stepper_angle = 0;
  //Serial.println(command);
  if (command.startsWith("STEPPER:")) {
    desired_stepper_angle = command.substring(8).toInt();
  }
  if (command.startsWith("SERVO:")) {
    servo_angle = command.substring(6).toInt();
  }
  if (command.startsWith("DC_POS:")) {
    dc_motor_position = command.substring(8).toInt();
    dc_pos = true;
  }
  if (command.startsWith("DC_SPEED:")) {
    dc_motor_speed = command.substring(9).toInt();
  }

  move_stepper_to_pos(desired_stepper_angle);
  set_servo_angle(servo_angle);
  
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
  if (!responseReceived) {
    ping_gui();
  } else {
      // read sensors
    pot_output = read_potentiometer(POT_PIN);
    ultrasonic_distance_cm = read_ultrasonic(ULTRASONIC_PIN);
    slot_blocked = read_slot(SLOT_PIN);
    ir_data = 10; //TODO - Why is this always set to 10?
    
    // Send sensor data
    // Format = "SENSOR_DATA:pot_output,ultrasonic_distance_cm,ir_data,slot_blocked"
    if ((millis() - lastSensorTime) > sensorTransmitDelay) {
      // TODO: Need to send motor data as well
      Serial.println("SENSOR_DATA:" + String(pot_output) + "," + String(ultrasonic_distance_cm) + "," + String(ir_data) + "," + String(slot_blocked) + "," +String(dc_motor_speed) + "," + String(dc_motor_position) + "," + String(current_stepper_angle) + "," + String(servo_angle));
      lastSensorTime = millis();
      delay(100);

    }

    // read serial port
    String command = read_serial_port();
    //delay(15); // prevent flooding port

        
    // Check if the command is "OVERRIDE:ON"
    if (command == "OVERRIDE:ON") {
      // change state to s4 (@oliver)
      gui_override = true;
      prev_gui_state = state;
      state = 4;
      Serial.println(state);

      Serial.println("Override ON, waiting for command");
    } else if (command == "OVERRIDE:OFF"){
      // change back to prev_state (@oliver)
      gui_override = false;
      state = prev_gui_state;
      //Serial.println("State: " + state);
      Serial.println(state);
      Serial.println("Override OFF");
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