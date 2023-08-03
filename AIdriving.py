# Import Modules
import inputs
import socket
import subprocess
import time
from threading import Timer, Thread
import json
#import GUI

# Import Raspberry Pi Packages
import board
import busio

# Steering Linear Actuator Limit Switch Pins
left_limit = 23
right_limit = 24

# Import ODrive Packages
from odrive.enums import *
from odrive.utils import *

# Steering Linear Actuator Driver Pins
pwm = 19
steering_direction = 26

# Steering Potentiometer Address & Config
import RPi.GPIO as gpio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


steering_direction = 26
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
potentiometer = AnalogIn(ads, ADS.P1)

steering_tolerance = 100
digital_steer = 0

steering_tolerance = 100
digital_steer = 0

#Take potentiometer value from Raspbhome/Brushlerry Pi VIA direct ethernet connection UDP
#filename = '/home/brushlessdc/Desktop/TESTING/framezz.csv'
serverAddress = ('192.168.1.25',5000) #Jetson address
clientAddress = ('192.168.1.14', 5000) #Raspberry address
bufferSize = 1024
UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  
UDPClient.bind(clientAddress)


# Initialise Raspberry Pi GPIO Pins
def initialise():
    gpio.setmode(gpio.BCM)
    gpio.setup(steering_direction, gpio.OUT)
    gpio.output(steering_direction, False)
    gpio.setup(pwm, gpio.OUT)
    gpio.output(pwm, False)
    
    gpio.setup(left_limit, gpio.IN)
    gpio.setup(right_limit, gpio.IN)


# <<<Steering>>> #
def set_motor_power(direction):
    # Steer Right
    if direction == -1:
        gpio.output(steering_direction, True)
        gpio.output(pwm, True)
        return
    # Steer Left
    elif direction == 1:
        gpio.output(steering_direction, False)
        gpio.output(pwm, True)
        return
    # Stop
    else:
        gpio.output(steering_direction, False)
        gpio.output(pwm, False)
        return


def read_steering_encoder():
    steering_encoder_val = potentiometer.value
    
    return steering_encoder_val


def steer_to_angle(target_angle, encoder):
    current_angle = read_steering_encoder()
    
    if abs(target_angle - current_angle) < steering_tolerance:
        return
    elif target_angle > current_angle:
        while not gpio.input(left_limit) and not abs(target_angle - current_angle) < steering_tolerance:
            set_motor_power(-1)
            current_angle = read_steering_encoder()
            print(f'Current_angle: {current_angle}')
        set_motor_power(0)
    elif target_angle < current_angle:
        while not gpio.input(right_limit) and not abs(target_angle - current_angle) < steering_tolerance:
            set_motor_power(1)
            current_angle = read_steering_encoder()
            print(f'Current_angle: {current_angle}')
        set_motor_power(0)
    
def steering_cal():
    global left_steering_range
    global left_center_angle 
    global left_max_mean_left_encoder, right_max_mean_left_encoder
    global left_max_mean_right_encoder, right_max_mean_right_encoder
    
    left_max_mean_left_encoder = 0
    right_max_mean_left_encoder = 0
    
    left_max_mean_right_encoder = 0
    right_max_mean_right_encoder = 0
    
    print('Steering Calibration Initiated')
    
    # Steer to Max Left & Calibrate Both Potentiometer
    while not gpio.input(left_limit):
        set_motor_power(-1)
        print('Left Limit: ', gpio.input(left_limit))
        time.sleep(0.01)
    print('Left Limit Hit')
    set_motor_power(0)
    
    for _ in range(10):
        reading = read_steering_encoder()
        left_max_mean_left_encoder += reading
    left_max_mean_left_encoder /= 10
    
    # Steer to Max Right & Calibrate Both Potentiometer
    while not gpio.input(right_limit):
        set_motor_power(1)
        print('Right Limit: ', gpio.input(right_limit))
        time.sleep(0.01)
    print('Right Limit Hit')
    set_motor_power(0)
    
    for _ in range(10):
        reading = read_steering_encoder()
        right_max_mean_left_encoder += reading
    right_max_mean_left_encoder /= 10
    
    # Mono-Encoder Steering
    left_steering_range = abs(left_max_mean_left_encoder - right_max_mean_left_encoder)
    left_center_angle = (left_max_mean_left_encoder - right_max_mean_left_encoder) / 2 + right_max_mean_left_encoder
    
    steer_to_angle(left_center_angle, 1)
    
    print('Left Encoder')
    print(f'Left Max Mean Value: {round(left_max_mean_left_encoder, 2)}')
    print(f'Right Max Mean Value {round(right_max_mean_left_encoder, 2)}')
    print(f'Steering Range: {round(left_steering_range, 2)}')
    print(f'Centre Estimate: {round(left_center_angle, 2)}')
    
    set_motor_power(0)
    print('Steering Calibration Complete')     


def steering_to_center():
    global digital_steer
    digital_steer = 0

    steer_to_angle(left_center_angle, 1)
    #steer2(left_center_angle)


def steering(state):
    global digital_steer
    
    if state == -1 and digital_steer > -5:
        digital_steer -= 1
    if state == 1 and digital_steer < 5:
        digital_steer += 1
    
    steer_to_angle(left_steering_range / 10 * digital_steer + (left_center_angle), 1)


# ODrive Axis States List in Strings 
axis_states = [
    'Undefined', 
    'Idle', 
    'Startup Sequence', 
    'Motor Calibration', 
    'Encoder Index', 
    '', 
    'Encoder Index Search', 
    'Encoder Offset Calibration', 
    'Closed Loop Control', 
    'Lockin Spin',
    'Encoder Dir Find', 
    'Homing', 
    'Encoder Hall Polarity Calibration', 
    'Encoder Hall Phase Calibration'
]


def send_dic():

    global odrv, server, terminate
    
    while not terminate:
        try:
            odrive_dic = {
                'battery_voltage': odrv.vbus_voltage,
                'control_mode': odrv.axis1.controller.config.control_mode,
                'm0_state': get_state_result(odrv.axis0.current_state),
                'm1_state': get_state_result(odrv.axis1.current_state),
                'm0_velocity': odrv.axis0.encoder.vel_estimate,
                'm1_velocity': odrv.axis1.encoder.vel_estimate,
                'm0_current': odrv.axis0.motor.current_control.Iq_measured,
                'm1_current': odrv.axis1.motor.current_control.Iq_measured,
                'm0_calibration': get_error_boolean(odrv.axis0.motor.is_calibrated),
                'm1_calibration': get_error_boolean(odrv.axis1.motor.is_calibrated),
                'watchdog_timer': get_error_boolean(odrv.axis1.config.enable_watchdog),
                'a1_system_error': get_error_boolean(odrv.error),
                'a1_motor_error': get_error_boolean(odrv.axis1.motor.error),
                'a1_controller_error': get_error_boolean(odrv.axis1.controller.error),
                'a1_encoder_error': get_error_boolean(odrv.axis1.encoder.error)
            }
        except:
            print('ERROR: ODrive Dictionary')
            odrv = odrive.find_any()
            print(f'ODrive Retrieve: {odrv}')


def get_error_boolean(data):
    if data == 0 or 'False':
        result = 'No Error'
    elif data == 1 or 'True':
        result = 'Error'
    else:
        result = 'Unknown'
    return result


def get_state_result(data):
    result = axis_states[data]
    return result
    

# <<<ODRIVE>>> #
def power():
    global odrv, cam
    
    odrv = odrive.find_any()
    print(f'ODrive Retrieve: {odrv}')
    odrv.clear_errors()
    dump_errors(odrv)

    
def Motor_Calibraton():
    global odrv
    
    try:
        odrv.clear_errors()
        odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print('Fully calibrated')
        time.sleep(20)
        odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        odrv.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        print('Encoder completed')
        time.sleep(10)
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print('In closed loop')
        time.sleep(10)
        odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        odrv.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        print('In velocity control')
        dump_errors(odrv)
    except:
        print('ERROR: Calibration')
        
        odrv = odrive.find_any()
        print(f'ODrive Retrieve: {odrv}')


def move_motors(speed):
    odrv.axis0.controller.input_vel = -(speed)
    odrv.axis1.controller.input_vel = (speed)


def controller_command_handling():
    global digital_steer, terminate_socket
    
    print("in controller handling")
    while True:
        
        events = inputs.get_gamepad()
        for event in events:
            # print(event.ev_type, event.code, event.state)
            '''
            # Check if Controller Bind Exist
            if event.code or event.code+'_BUTTON' in keybindCommandDict:
                keybindCommandDict[event.code](event.state)             # Highlight Controller Buttons in GUI
                
            if event.code+'_BUTTON' in keybindCommandDict:
                keybindCommandDict[event.code+'_BUTTON'](event.state)   # Highlight Buttons in GUI
            '''
            # BUTTONS OVERVIEW #
            # CONTROL LOOP              >>> BTN_START       >>> Start
            # DISCONNECT                >>> BTN_BACK        >>> Back
            # MOTOR FORWARD/BACKWARD    >>> ABS_HAYOY       >>> △ & ▽
            # STEER LEFT/RIGHT          >>> ABS_HAY0X       >>> ◁ & ▷
            # STEER CALIBRATION         >>> BTN_NORTH       >>> Y
            # STEER TO CENTER           >>> BTN_SOUTH       >>> A
            # MOTOR STOP                >>> BTN_EAST        >>> B
            # MOTOR CALIBRATION         >>> BTN_WEST        >>> X
            # START TRAINING            >>> BTN_TR          >>> RB
            # STOP TRAINING             >>> BTN_TL          >>> LB
            
            # KNOWN ISSUES #
            # Raspberry Pi OS   - Inputs X and Y swapped, Back as BTN_SELECT
            # Windows OS        - Start as BTN_SELECT, Back as BTN_START
            
            # Start Button
            if event.code == 'BTN_START' and event.state == 1:
                #toggle_state = shared_variables.controlCloseloop            # Store Local Boolean
                #shared_variables.controlCloseloop = not toggle_state        # Write New Boolean
                #keybindCommandDict[event.code+'_TOGGLE'](not toggle_state)  # Call Function in GUI
                print("start")
            
            # Back Button
            elif event.code == 'BTN_SELECT' and event.state == 1:
                #systemShutdown(GUI.root)
                
                print("stop program button")
                return
            
            # Action Buttons (X)(Y)(A)(B)
            # Steering Calibration Button
            elif event.code == 'BTN_NORTH' and event.state == 1: 
                steering_cal()
            # Steer to Center Button
            elif event.code == 'BTN_SOUTH' and event.state == 1: 
                steering_to_center()
            # Motor Stop Button    
            elif event.code == 'BTN_EAST' and event.state == 1: 
                odrv.axis0.requested_state = AXIS_STATE_IDLE
                odrv.axis1.requested_state = AXIS_STATE_IDLE
                print("motor stop button")
            # Motor Calibration Button    
            elif event.code == 'BTN_WEST' and event.state == 1:
                print("Motor Calibration Initiated")
                Motor_Calibraton()

            # Trigger Buttons
            elif event.code == 'BTN_TR' and event.state == 1:
                print("start AI button")
                start_thread("Connect Jetson", conn_jetson)
                start_thread("Start Self Driving", self_drive)

            elif event.code == 'BTN_TL' and event.state == 1:
                conn_jetson.close()
                stop_flag.set()
                print("stop training button")


            # D-Pad Buttons
            # Patching Buttons with 3 States
            elif 'ABS_HAT0Y' in event.code:
                # Drive Forward
                if event.state == 1:
                    move_motors(2)
                # Drive Backwards
                elif event.state == -1:
                    move_motors(-2)
                # No Input
                else:                  
                    move_motors(0)

            # Patching Buttons with 3 States
            elif 'ABS_HAT0X' in event.code:
                # Steering Left
                if event.state == -1 and gpio.input(right_limit) != 1:   
                    set_motor_power(1)
                # Steering Right
                elif event.state == 1 and gpio.input(left_limit) != 1: 
                    set_motor_power(-1)
                # No Input
                else:
                    set_motor_power(0)


# <<<AI>>>
def conn_jetson():
    subprocess.run([ "ssh", "brushlessdc@192.168.1.24", "python3 /home/brushlessdc/Desktop/TESTING/modelANDtransfer.py"])
    time.sleep(5)
    subprocess.run([ "ssh", "brushlessdc@192.168.1.24", "exit"])
    print("ssh connection success")

def Self_Driving():
	move_motors(1)
	while True:
		# Define the file address for the current frame
		data,address= UDPClient.recvfrom(bufferSize)
		bytes2unpack1 = data[:8]
		print('twast')
		P_value= struct.unpack('i',bytes2unpack1)
		P_value = P_value[0]
		print(type(P_value))
		print(P_value)
		print(f"Data from Server, P = {P_value}")
		time.sleep(1) 
		
		if potentiometer - P_value > 50 or potentiometer - P_value <50 :
			if P_value > right_max and P_value < left_max:
				if P_value > center :
					while potentiometer < P_value and gpio.input(left_limit) != 1:
						set_motor_power(1)
						print("set_motor_power(1)")
				else:
					while potentiometer > P_value and gpio.input(right_limit) != 1:
						set_motor_power(-1)
						print("set_motor_power(-1)")
			
				set_motor_power(0)
				print("stop motor")


# <<<MULTITHREADING>>>
# Start Function as Background Thread
def start_thread(name, target):
    newthread = threading.Thread(name = name, target = target)
    newthread.start()


def main():
	initialise()
	set_motor_power(0)    
	power()
	start_thread("Controller Command Handling", controller_command_handling)
    



def exit():
    
    set_motor_power(0)
    gpio.cleanup()
    time.sleep(1)
    
    print('Raspberry Pi Ended') 


if __name__ == '__main__':
    main()
    exit()
