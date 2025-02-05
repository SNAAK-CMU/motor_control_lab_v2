import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QSlider, QLabel, QGridLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
import time

"""
To get the serial port name, run the following command in a python shell:
import serial.tools.list_ports

   ports = serial.tools.list_ports.comports()

   for port, desc, hwid in ports:
       print(f"Port: {port}, Description: {desc}, Hardware ID: {hwid}")

"""

class ArduinoGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensors and Motor Control Lab GUI - Team C")
        self.setGeometry(100, 100, 500, 500)

        try:
            # self.serial_port = serial.Serial('/dev/cu.usbmodem21301', 9600, timeout=1) # Open serial port by name
            self.serial_port = wait_for_ping(port='/dev/cu.usbmodem21301', baud_rate=9600)
            print(f"Opened serial port: {self.serial_port}")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.serial_port = None
        
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.setup_motor_controls()

        self.setup_IR_sensor_display()
        self.setup_ultrasonic_sensor_display()
        self.setup_potentiometer_sensor_display()
        self.setup_slot_sensor_display()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_sensor_data)
        self.timer.start(100)  # Update sensor data every 100 ms

    def setup_motor_controls(self):
        motor_layout = QGridLayout()
        self.layout.addLayout(motor_layout)

        motors = [
            ("Stepper Motor Steps", -45, 45, "(steps)"),
            ("Servo Motor Position", 0, 180, "(angle)"), # deg or rad?
            ("DC Motor Position", -45, 45, "(angle)"), # deg or rad?
            ("DC Motor Speed", 0, 150, "(rpm)")
        ]
   
        for row, (motor_type, min_val, max_val, unit) in enumerate(motors):
            label = QLabel(f"{motor_type}:")
            slider = QSlider(Qt.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval((max_val - min_val) // 10)
            
            value_label = QLabel(f"{0} {unit}")
            value_label.setMinimumWidth(80)
            
            motor_layout.addWidget(label, row, 0)
            motor_layout.addWidget(slider, row, 1)
            motor_layout.addWidget(value_label, row, 2)

            slider.valueChanged.connect(lambda value, label=value_label, unit=unit: 
                                        label.setText(f"{value} {unit}"))

            attr_name = f"{motor_type.lower().replace(' ', '_')}_slider"
            setattr(self, attr_name, slider)

        self.stepper_motor_steps_slider.valueChanged.connect(self.control_stepper)
        self.servo_motor_position_slider.valueChanged.connect(self.control_servo)
        self.dc_motor_position_slider.valueChanged.connect(self.control_dc_position)
        self.dc_motor_speed_slider.valueChanged.connect(self.control_dc_speed)

        # Add override button
        self.override_button = QPushButton("Override OFF")
        self.override_button.setCheckable(True)
        self.override_button.setStyleSheet("QPushButton:checked { background-color: green; color: white; }")
        self.override_button.toggled.connect(self.toggle_override)
        motor_layout.addWidget(self.override_button, len(motors), 0, 1, 3)

        motor_layout.setColumnStretch(1, 1)
        self.setMinimumSize(450, 200)

    def toggle_override(self, state):
        self.override_button.setText("Override ON" if state else "Override OFF")
        command = f"OVERRIDE:{'ON' if state else 'OFF'}\n"
        if self.serial_port.is_open:
            self.serial_port.write(command.encode())
            print(f"Sent command: {command}")
            time.sleep(0.1)
            response = self.serial_port.readline().decode().strip()
            print(f"Received response: {response}")
        else:
            print(f"Serial port is not open. Cannot send command: {command}")

    def setup_IR_sensor_display(self):
        self.ir_sensor_label = QLabel("IR Sensor Data: N/A")
        self.layout.addWidget(self.ir_sensor_label)

    def setup_ultrasonic_sensor_display(self):
        self.ultrasonic_sensor_label = QLabel("Ultrasonic Sensor Data: N/A")
        self.layout.addWidget(self.ultrasonic_sensor_label)

    def setup_potentiometer_sensor_display(self):
        self.potentiometer_sensor_label = QLabel("Potentiometer Sensor Data: N/A")
        self.layout.addWidget(self.potentiometer_sensor_label)

    def setup_slot_sensor_display(self):
        self.slot_sensor_label = QLabel("Slot Sensor Data: N/A")
        self.layout.addWidget(self.slot_sensor_label)

    def control_stepper(self, value):
        command = f"STEPPER:{value}\n" # Send command to Arduino
        if self.serial_port.is_open:
            if self.override_button.isChecked():
                self.serial_port.write(command.encode())
                # print(f"Sent command: {command}")
        else:
            print(f"Serial port is not open. Cannot send command: {command}")


    def control_servo(self, value):
        command = f"SERVO:{value}\n"
        self.serial_port.write(command.encode())
        if self.serial_port.is_open:
            if self.override_button.isChecked():
                self.serial_port.write(command.encode())
                # print(f"Sent command: {command}")
        else:
            print(f"Serial port is not open. Cannot send command: {command}")

    def control_dc_position(self, value):
        command = f"DC_POS:{value}\n"
        self.serial_port.write(command.encode())
        if self.serial_port.is_open:
            if self.override_button.isChecked():
                self.serial_port.write(command.encode())
                # print(f"Sent command: {command}")
        else:
            print(f"Serial port is not open. Cannot send command: {command}")

    def control_dc_speed(self, value):
        command = f"DC_SPEED:{value}\n"
        self.serial_port.write(command.encode())
        if self.serial_port.is_open:
            if self.override_button.isChecked():
                self.serial_port.write(command.encode())
                # print(f"Sent command: {command}")
        else:
            print(f"Serial port is not open. Cannot send command: {command}")

    def update_sensor_data(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode().strip()
            if data.startswith("SENSOR_DATA:"):
                print(f"Received sensor data: {data}")
                sensor_values = data.split(":")[1].split(",")
                print(f"Sensor values: {sensor_values}")
                if len(sensor_values) == 4: 
                    self.potentiometer_sensor_label.setText(f"Potentiometer Sensor Data: {sensor_values[0]}")
                    self.ultrasonic_sensor_label.setText(f"Ultrasonic Sensor Data: {sensor_values[1]} cm")
                    self.ir_sensor_label.setText(f"IR Sensor Data: {sensor_values[2]} cm")
                    self.slot_sensor_label.setText(f"Slot Sensor Data: {'Blocked' if sensor_values[3] == '1' else 'Clear'}")
            else:
                print(f"Received data: {data}")

def wait_for_ping(port='', baud_rate=9600):
    ser = serial.Serial(port, baud_rate, timeout=1)
    time.sleep(2)  # wait for port to open
    print("Waiting for PING from Arduino...")
    
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode().strip()
            print(f"Received message: {message}")
            
            if message == "PING":
                response = "PONG\n"
                ser.write(response.encode())
                print(f"Sent response: {response.strip()}")
                
            # # Wait for confirmation from Arduino
            # confirmation = ser.readline().decode().strip()
            if message == "GUI Connection Established!!":
                print("Connection established with Arduino")
                return ser
        
        time.sleep(0.1)
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ArduinoGUI()

    # show the GUI
    gui.show()
    sys.exit(app.exec_())