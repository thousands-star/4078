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

def move_robot():
    global use_pid, left_speed, right_speed, motion
    global command_queue, left_encoder, right_encoder
    flag_new_pid_cycle = True

    while True:
        # Autonomous Driving (driving_mode == 1)
        if driving_mode == 1:
            if command_queue:
                # Fetch the first command from the queue
                auto_motion, left_vel, right_vel, dt = command_queue.pop(0)
                
                # Reset encoders before starting a new command
                left_encoder.reset()
                right_encoder.reset()

                # Create PID controllers for left and right wheels
                pid_left = PID(kp, ki, kd, setpoint=left_vel, output_limits=(0, 1))
                pid_right = PID(kp, ki, kd, setpoint=right_vel, output_limits=(0, 1))

                # Record the start time
                start_time = time.time()

                # Continue motion until the specified duration (dt) has passed
                while (time.time() - start_time) < dt:
                    # Get the current encoder values (current velocities of the wheels)
                    current_left_vel = left_encoder.value
                    current_right_vel = right_encoder.value

                    # Use PID controllers to adjust the speed towards the target
                    adjusted_left_speed = pid_left(current_left_vel)
                    adjusted_right_speed = pid_right(current_right_vel)

                    # Set the robot's wheel velocities using the adjusted speeds
                    pibot.value = (adjusted_left_speed, adjusted_right_speed)

                    print(time.time() - start_time)
                    # Small delay to avoid busy-waiting
                    time.sleep(0.01)
                
                # After the time for this command has passed, stop the robot
                pibot.value = (0, 0)

                print(f"{auto_motion} completed: Left Velocity = {left_vel}, Right Velocity = {right_vel}, Duration = {dt} seconds")
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
    global left_speed, right_speed,  motion
    left_speed, right_speed, dt = float(request.args.get('left_speed')), float(request.args.get('right_speed')), float(request.args.get('dt'))
    if (left_speed == 0 and right_speed == 0):
        motion = 'stop'
    elif (left_speed != right_speed ):
        motion = 'turning'
    elif (left_speed > 0 and right_speed > 0):
        motion = 'forward'
    elif (left_speed < 0 and right_speed < 0):
        motion = 'backward'
    command_queue.append([motion, left_speed, right_speed, dt])
    return motion
    

# The function to switch mode
@app.route('/mode')
def mode():
    global driving_mode
    mode = float(request.args.get('driving_mode'))
    driving_mode = mode
    return str(mode)

@app.route('/calibrate')
def calibrate():
    global calibrate
    if(calibrate == False):
        calibrate = True
    return "Calibrating"


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
dt = 0

motion = ''
auto_motion = ''
driving_mode = 0
command_queue = []
calibrate = False

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