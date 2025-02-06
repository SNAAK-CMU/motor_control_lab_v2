#ifndef DCMOTOR_H
#define DCMOTOR_H

double setpointPosition = 0,prevPosition = 0, inputPosition = 0;
unsigned long previousTime = 0, prevTime = 0;
double setpointSpeed = 0, currentSpeed = 0;
double integral_error = 0, prevError = 0;
double outputPosition = 0;
int motorPower = 0;

void setup_dc_motor();
int control_dc_motor(double setpoint, double input, double Kp, double Ki, double Kd);
void setMotorSpeed(int speed);
#endif