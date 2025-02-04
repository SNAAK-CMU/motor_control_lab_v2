#ifndef ULTRASONIC_H
#define ULTRASONIC_H

void setup_dc_motor();
int control_dc_motor(double setpoint, double input, double Kp, double Ki, double Kd);
void setMotorSpeed(int speed);
#endif