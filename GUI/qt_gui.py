import sys
import serial
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QSlider, QLabel, QGridLayout
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt

class ArduinoGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sensors and Motor Control Lab GUI - Team C")
        self.setGeometry(100, 100, 500, 500)

        try:
            self.serial_port = serial.Serial('COM3', 9600, timeout=1) # Open serial port
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
        self.timer.start(1000)  # Update sensor data every 1 second

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
        #self.serial_port.write(command.encode())

    def setup_IR_sensor_display(self):
        self.sensor_label = QLabel("IR_Sensor Data: N/A")
        self.layout.addWidget(self.sensor_label)
    
    def setup_ultrasonic_sensor_display(self):
        self.sensor_label = QLabel("Ultrasonic Sensor Data: N/A")
        self.layout.addWidget(self.sensor_label)
    
    def setup_potentiometer_sensor_display(self):
        self.sensor_label = QLabel("Potentiometer Sensor Data: N/A")
        self.layout.addWidget(self.sensor_label)
    
    def setup_slot_sensor_display(self):
        self.sensor_label = QLabel("Slot Sensor Data: N/A")
        self.layout.addWidget(self.sensor_label)

    def control_stepper(self, value):
        command = f"STEPPER:{value}\n" # Send command to Arduino
        # self.serial_port.write(command.encode())

    def control_servo(self, value):
        command = f"SERVO:{value}\n"
        # self.serial_port.write(command.encode())

    def control_dc_position(self, value):
        command = f"DC_POS:{value}\n"
        # self.serial_port.write(command.encode())

    def control_dc_speed(self, value):
        command = f"DC_SPEED:{value}\n"
        # self.serial_port.write(command.encode())

    def update_sensor_data(self):
        # self.serial_port.write(b"SENSOR\n")
        # data = self.serial_port.readline().decode().strip()
        # self.sensor_label.setText(f"Sensor Data: {data}")
        pass
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = ArduinoGUI()
    gui.show()
    sys.exit(app.exec_())