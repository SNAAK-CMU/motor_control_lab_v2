#include <Encoder.h>
#include "dc_motor.h"

// Motor Driver Pins
#define ENA 11     // Enable Pin
#define IN1 12     // Motor Direction
#define IN2 13     // Motor Direction


// double setpointPosition = 0,prevPosition = 0, inputPosition = 0;
// unsigned long previousTime = 0, prevTime = 0;
// double setpointSpeed = 0, currentSpeed = 0;
// double integral_error = 0, prevError = 0;
// double outputPosition = 0;
// int motorPower = 0;

void setup_dc_motor() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
}

int control_dc_motor(double setpoint, double input, double Kp, double Ki, double Kd) {
    
    unsigned long currentTime = micros();
    double dt = ((currentTime - previousTime) / (1.0e6)); // Convert to seconds
    previousTime = currentTime;

    double error = setpoint - input;
    double derivative = (dt > 0) ? ((error - prevError) / (dt)) : 0;

    integral_error = integral_error + error * dt;
    prevError = error;

    double output = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
    int motorPower = constrain(output, -255, 255);
    
    return motorPower;
}


void setMotorSpeed(int speed) {
    if (speed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else if (speed < 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
    analogWrite(ENA, abs(speed));
}


