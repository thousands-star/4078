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
    
# Gradual acceleration and deceleration
def gradual_speed_change(current_speed, target_speed, step=0.01):
    if abs(current_speed - target_speed) <= 0.01:  # Tolerance for closeness
        return target_speed
    elif current_speed < target_speed:
        return min(current_speed + step, target_speed)
    else:
        return max(current_speed - step, target_speed)

# main function to control the robot wheels
# With Simple displacement-controlled function installed.
def move_robot():
    global use_pid, left_speed, right_speed, motion
    global command_queue, left_disp, right_disp, auto_motion
    flag_new_pid_cycle = True

    while True:
        # Autonomous Driving (driving_mode == 1)
        if driving_mode == 1:
            if command_queue:
                # Fetch the first command from the queue
                auto_motion, left_disp, right_disp = command_queue.pop(0)

                # Execute the command based on auto_motion
                if auto_motion == 'forward':
                    left_encoder.reset()
                    right_encoder.reset()
                    target_speed = 0.6
                    current_left_speed = 0
                    current_right_speed = 0
                    while abs((left_disp + right_disp) / 2 - (left_encoder.value + right_encoder.value) / 2) > 3:
                        current_left_speed = gradual_speed_change(current_left_speed, target_speed)
                        current_right_speed = gradual_speed_change(current_right_speed, target_speed)
                        pibot.value = (current_left_speed, current_right_speed)
                        
                        if (left_encoder.value + right_encoder.value) > (right_disp + left_disp + 3):
                            break
                    pibot.value = (0, 0)
                    print(auto_motion, "Value", left_encoder.value, right_encoder.value)

                elif auto_motion == 'backward':
                    left_encoder.reset()
                    right_encoder.reset()
                    current_left_speed = 0
                    current_right_speed = 0
                    target_speed = -0.6
                    while abs((left_disp + right_disp) / 2 - (left_encoder.value + right_encoder.value) / 2) > 3:
                        current_left_speed = gradual_speed_change(current_left_speed, target_speed)
                        current_right_speed = gradual_speed_change(current_right_speed, target_speed)
                        pibot.value = (current_left_speed, current_right_speed)
                        
                        if (left_encoder.value + right_encoder.value) > (abs(right_disp + left_disp) + 3):
                            break
                    pibot.value = (0, 0)
                    print(auto_motion, "Value", left_encoder.value, right_encoder.value)


                elif auto_motion == 'turning':
                    left_encoder.reset()
                    right_encoder.reset()
                    ls = 0
                    rs = 0

                    while ((abs(left_disp) - left_encoder.value) + (abs(right_disp) - right_encoder.value)) / 2 > 0:
                        # Determine the direction for turning
                        # Because moving front is more powerful than moving backward
                        # we have to take a look that which wheel is moving backward, and add more speed to it.
                        if left_disp > 0:
                            ls = gradual_speed_change(ls, 0.75)
                        else:
                            ls = gradual_speed_change(ls, -0.7)
                        
                        if right_disp > 0:
                            rs = gradual_speed_change(rs, 0.75)
                        else:
                            rs = gradual_speed_change(rs, -0.7)

                        pibot.value = (ls, rs)
                        # Breaking logic if the encoder value goes beyond expected range
                        if left_encoder.value > abs(left_disp) or right_encoder.value > abs(right_disp):
                            break
                    pibot.value = (0, 0)
                    print(auto_motion, "Value", left_encoder.value, right_encoder.value)

                # Once the command is executed, mark auto_motion as 'stop'
                auto_motion = 'stop'
                # To ensure integrity of every command, directly execute next command would make the robot busy and which the effectiveness was affected.
                time.sleep(0.1)

            else:
                # If no commands are in the queue, pause the robot and wait
                pibot.value = (0, 0)

        # Teleoperating Mode (driving_mode == 0)
        elif driving_mode == 0:
            # Move the robot based on velocity commands
            if not use_pid:
                # Direct velocity control
                pibot.value = (left_speed, right_speed)
            else:
                # PID-based control for teleoperation
                if motion == 'stop' or motion == 'turning':
                    pibot.value = (left_speed, right_speed)
                    left_encoder.reset()
                    right_encoder.reset()
                    flag_new_pid_cycle = True          
                else:
                    left_speed, right_speed = abs(left_speed), abs(right_speed)
                    if flag_new_pid_cycle:
                        pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0, 1), starting_output=right_speed)
                        flag_new_pid_cycle = False
                    pid_right.setpoint = left_encoder.value
                    right_speed = pid_right(right_encoder.value)
                    if motion == 'forward': 
                        pibot.value = (left_speed, right_speed)
                    else: 
                        pibot.value = (-left_speed, -right_speed)

        # Small delay to avoid busy-waiting
        time.sleep(0.005)


# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        return "Using PID"
    else:
        return "Not using PID"
    
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
    
    # if 'time' in request.args:

# The main function to enable autonomous driving
@app.route('/disp')
def disp():
    global left_disp, right_disp, auto_motion, command_queue

    left_disp, right_disp = float(request.args.get('left_disp')), float(request.args.get('right_disp'))
    print("Value",left_disp,right_disp)
    if (left_disp == 0 and right_disp == 0):
        auto_motion = 'stop'
        command_queue = []
    elif (left_disp != right_disp ):
        auto_motion = 'turning'
    elif (left_disp > 0 and right_disp > 0):
        auto_motion = 'forward'
    elif (left_disp < 0 and right_disp < 0):
        auto_motion = 'backward'

    command_queue.append([auto_motion, left_disp, right_disp])

    return auto_motion

# The function to switch mode
@app.route('/mode')
def mode():
    global driving_mode
    mode = float(request.args.get('driving_mode'))
    driving_mode = mode
    return mode


# Constants
# Pin Number
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
kp = 0
ki = 0
kd = 0
left_speed, right_speed = 0, 0
left_disp, right_disp = 0, 0
motion = ''
auto_motion = ''
driving_mode = 0
command_queue = []

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
        move_robot()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")