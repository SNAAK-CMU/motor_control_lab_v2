// #include "dc_motor.h"

// #define POTENTIOMETER A2
// bool positionControl = 0;

// #define ENCODER_PIN_A 2 
// #define ENCODER_PIN_B 9

// void setup() {
//   Serial.begin(115200);
//   setup_dc_motor();

//   pinMode(POTENTIOMETER, INPUT);
// }

// Encoder myEnc(ENCODER_PIN_A, ENCODER_PIN_B);

// void loop() {

//     inputPosition = myEnc.read();
    
//     if (positionControl){
//       // PID Gains for position control
//       double Kp = 1.5, Ki = 0.1, Kd = 0.0;
//       setpointPosition = map(analogRead(POTENTIOMETER), 0, 1023, -500, 500);

//       // Calculate PID output for position control
//       motorPower = control_dc_motor(setpointPosition, inputPosition, Kp, Ki, Kd);

//     } else {
//       // PID Gains for speed control
//       double Kp = 0.5, Ki = 0.8, Kd = 0.0;
//       setpointSpeed = map(analogRead(POTENTIOMETER), 0, 1023, -800, 800); // Adjust range as needed

//       // Get current motor speed
//       unsigned long currTime = micros();
//       double dTime = ((currTime - prevTime) / (1.0e6)); // Convert to seconds
//       currentSpeed =  (inputPosition - prevPosition) / (dTime);

//       // Calculate PID output for speed control
//       motorPower = control_dc_motor(setpointSpeed, currentSpeed, Kp, Ki, Kd);
//       prevTime = currTime;
//     }

//     prevPosition = inputPosition;    
//     setMotorSpeed(motorPower);

//     delay(100);
//     // Debug prints for position control
//     // Serial.print("Setpoint: ");
//     // Serial.print(setpointPosition);
//     // Serial.print(" | Current Position: ");
//     // Serial.print(inputPosition);
//     // Serial.print(" | Speed: ");
//     // Serial.println(motorPower);

//     // Debug prints for speed control
//     Serial.print("Setpoint: ");
//     Serial.print(setpointSpeed);
//     Serial.print(" | Current Speed: ");
//     Serial.print(currentSpeed);
//     Serial.print(" | Output: ");
//     Serial.println(motorPower);

// }
