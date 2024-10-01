from simple_pid import PID
from picamera2 import Picamera2
from flask import Flask, Response, request
from gpiozero import Robot, Motor, DigitalInputDevice
import io
import time
import threading


app = Flask(__name__)


class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self.encoder = DigitalInputDevice(pin)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        self._value += 1
        
    @property
    def value(self):
        return self._value
        
def handle_mode0():
    """
    for self-driving mode
    """
    global use_pid, left_speed, right_speed
    flag_new_pid_cycle = True
    while True:
        ### if not using pid, just move the wheels as commanded
        if not use_pid:
            pibot.value = (left_speed, right_speed)          
            # print('Value', left_encoder.value, right_encoder.value)
        ### with pid, left wheel is set as reference, and right wheel will try to match the encoder counter of left wheel
        ### pid only runs when robot moves forward or backward. Turning does not use pid
        else:
            if (motion == 'stop') or (motion == 'turning'):
                pibot.value = (left_speed, right_speed) 
                left_encoder.reset()
                right_encoder.reset()
                flag_new_pid_cycle = True          
            else:
                left_speed, right_speed = abs(left_speed), abs(right_speed)
                if flag_new_pid_cycle:
                    pid_right = PID(kp_lin, ki_lin, kd_lin, setpoint=left_encoder.value, output_limits=(0,1), starting_output=right_speed)
                    flag_new_pid_cycle = False
                pid_right.setpoint = left_encoder.value
                right_speed = pid_right(right_encoder.value)
                if motion == 'forward': pibot.value = (left_speed, right_speed)
                else: pibot.value = (-left_speed, -right_speed)
                print('Value', left_encoder.value, right_encoder.value)
                # print('Value', left_encoder.value, right_encoder.value)
                # print('Speed', left_speed, right_speed)
        time.sleep(0.005)
        if drive_mode == 1:
            break


def handle_mode1():
    """
    for waypoint navigation
    """
    global motion_queue, kp_lin, ki_lin, kd_lin, kp_turn, ki_turn, kd_turn, turn_tolerance, linear_tolerance
    while True:
        # print("motion", motion)
        try:
            motion_elem = motion_queue.pop(0)
            if len(motion_elem) == 2:
                motion, dt = motion_elem
            elif len(motion_elem) == 3:
                motion, left_disp, right_disp = motion_elem
            left_encoder.reset()
            right_encoder.reset()
        except:
            motion = "stop"
        finally:
            if motion == "forward":
                pid_left = PID(kp_lin, ki_lin, kd_lin, setpoint=right_encoder.value, output_limits=(0.35,0.65), starting_output=linear_speed)
                # pid_right =  PID(kp_lin, ki_lin, kd_lin, setpoint=right_disp, output_limits=(0.48,0.75), starting_output=linear_speed)
                while (left_encoder.value < abs(left_disp) - linear_tolerance) and (right_encoder.value < abs(right_disp) - linear_tolerance):
                    pid_left.setpoint = right_encoder.value
                    # pid_right.setpoint = left_encoder.value
                    print(f"Setpoint: {left_encoder.value}, {right_encoder.value}")
                    # right_speed = pid_right(right_encoder.value)
                    left_speed = pid_left(left_encoder.value)
                    print(f"Speed: {left_speed}, {linear_speed}")
                    pibot.value = (left_speed, linear_speed)
                pibot.value = (0, 0)
            elif motion == "backward":
                pid_left = PID(kp_lin, ki_lin, kd_lin, setpoint=left_disp, output_limits=(0.48,0.52), starting_output=linear_speed)
                pid_right =  PID(kp_lin, ki_lin, kd_lin, setpoint=right_disp, output_limits=(0.48,0.52), starting_output=linear_speed)
                while (left_encoder.value < abs(left_disp) - linear_tolerance) and (right_encoder.value < abs(right_disp) - linear_tolerance):
                    pid_left.setpoint = max(left_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    pid_right.setpoint = max(right_encoder.value, (right_encoder.value+left_encoder.value)/2)
                    print(f"Setpoint: {pid_left.setpoint}, {pid_right.setpoint}")
                    right_speed = pid_right(right_encoder.value)
                    left_speed = pid_left(left_encoder.value)
                    print(f"Speed: {left_speed}, {right_speed}")
                    pibot.value = (-left_speed, -right_speed)
                pibot.value = (0, 0)
            elif motion == "turn left":
                set_point = (left_encoder.value + right_encoder.value) / 2
                pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                start_time = time.time()
                while (time.time() - start_time) < dt:
                    left_speed = pid_left(left_encoder.value)
                    right_speed = pid_right(right_encoder.value)
                    pibot.value = (-left_speed, right_speed)
                pibot.value = (0, 0)
            elif motion == "turn right":
                set_point = (left_encoder.value + right_encoder.value) / 2
                pid_left = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                pid_right = PID(kp_turn, ki_turn, kd_turn, setpoint=set_point, output_limits=(0.65,0.85), starting_output=turn_speed)
                start_time = time.time()
                while (time.time() - start_time) < dt:
                    left_speed = pid_left(left_encoder.value)
                    right_speed = pid_right(right_encoder.value)
                    pibot.value = (left_speed, -right_speed)
                pibot.value = (0, 0)
            if motion != "stop":
                print('Value', left_encoder.value, right_encoder.value)
        time.sleep(0.05)
        if drive_mode == 0:
            break

# main function to control the robot wheels
def move_robot():
    # print("mode", drive_mode)
    if drive_mode == 0:
        handle_mode0()
    else:
        # print("motion", motion)
        handle_mode1()
    
@app.route('/linearpid')
def set_linearpid():
    global kp_lin, ki_lin, kd_lin
    kp_lin, ki_lin, kd_lin = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Linear PID to ", kp_lin, ki_lin, kd_lin)
    return "Setting Linear PID"

    
@app.route('/turnpid')
def set_turnpid():
    global kp_turn, ki_turn, kd_turn
    kp_turn, ki_turn, kd_turn = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
    print("Setting Turn PID to ", kp_turn, ki_turn, kd_turn)
    return "Setting Turn PID"

@app.route('/lineartolerance')
def set_lineartolerance():
    global linear_tolerance
    linear_tolerance = int(request.args.get('tolerance'))
    print("Setting Linar tolerance to ", linear_tolerance)
    return "Setting Linear tolerance"
    
# Receive a request to capture and send a snapshot of the picamera
@app.route('/image')
def capture_image():
    stream = io.BytesIO()
    picam2.capture_file(stream, format='jpeg')
    stream.seek(0)
    return Response(stream, mimetype='image/jpeg')
    

 # Receive command to move the pibot
@app.route('/move')
def move():
    global left_speed, right_speed, motion
    left_speed, right_speed = float(request.args.get('left_speed')), float(request.args.get('right_speed'))
    if (left_speed == 0 and right_speed == 0):
        motion = 'stop'
    elif (left_speed != right_speed ):
        motion = 'turning'
    elif (left_speed > 0 and right_speed > 0):
        motion = 'forward'
    elif (left_speed < 0 and right_speed < 0):
        motion = 'backward'
    return motion

@app.route('/angle')
def set_angle():
    global motion, motion_queue
    dt, motion = float(request.args.get('dt')), request.args.get('motion')
    motion_queue.append((motion, dt))
    print("The motion now is", motion)
    return motion

@app.route('/disp')
def set_disp():
    global left_disp, right_disp, motion, motion_queue
    left_disp, right_disp = int(request.args.get('left_disp')), int(request.args.get('right_disp'))
    if (left_disp == 0 and right_disp == 0):
        motion = 'stop'
    elif (left_disp > 0 and right_disp > 0):
        motion = 'forward'
    elif (left_disp < 0 and right_disp < 0):
        motion = 'backward'
    if motion != 'stop':
        motion_queue.append((motion, left_disp, right_disp))
    print("The motion now is", motion)
    return motion

@app.route('/mode')
def set_mode():
    global drive_mode 
    drive_mode = int(request.args.get('mode'))
    print(drive_mode)
    return str(drive_mode)
    

# Constants
in1 = 17 # may have to change this
in2 = 27 # may have to change this
ena = 18
in3 = 23 # may have to change this
in4 = 24 # may have to change this
enb = 25
enc_a = 26
enc_b = 16

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0

kp_turn= 0.1
ki_turn = 0.005
kd_turn = 0.001

kp_lin = 2
ki_lin = 0.05
kd_lin = 0.01
left_speed = 0
right_speed = 0
linear_speed = 0.5
turn_speed = 0.75
turn_tolerance = 3
linear_tolerance = 6
motion = ''
drive_mode = 1
motion_queue = []

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# Initialize flask
def run_flask():
    app.run(host='0.0.0.0', port=5000)
    
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        if drive_mode == 0:
            handle_mode0()
        else:
            handle_mode1()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")