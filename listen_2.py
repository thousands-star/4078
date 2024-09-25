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
def move_robot_auto():
    global command_queue
    global radius
    global calibrate
    global kp,ki,kd
    tolerance = 3

    while True:
        if calibrate:
            disp_for_one_meter = round(1 / 0.00534)  # Distance equivalent for 1 meter

            botSpeed = [0.6]
            print("Forward calibration")
            for bot_speed in botSpeed:
                T = 0
                for i in range(10):
                    not_moving = False
                    left_encoder.reset()
                    right_encoder.reset()
                    startTime = time.time()

                    # Loop until either encoder reaches the distance for one meter
                    while left_encoder.value < disp_for_one_meter and right_encoder.value < disp_for_one_meter:
                        try:
                            pibot.value = (bot_speed, bot_speed)  # Set the speed for both wheels
                        except KeyboardInterrupt:
                            not_moving = True
                            break

                    pibot.value = (0, 0)  # Stop the bot
                    dt = time.time() - startTime  # Time taken to travel 1 meter
                    world_speed = 1 / dt  # World speed in meters per second
                    scale = world_speed / bot_speed  # Scale factor for the bot
                    T = T + dt
                    if not_moving:
                        print(f"For bot speed {bot_speed:.2f}, the robot is not moving")
                    else:
                        print(f"bot speed {bot_speed:.2f},  it took {dt:.2f} seconds for 1 meter, real speed {(1/dt):.2f} scale = {scale:.2f}")
                    print("waiting for 5 second for next round")
                    time.sleep(5)
                T = T / 10
                world_speed = 1 / T
                scale = world_speed / bot_speed
                print(f"bot speed {bot_speed:.2f},  average {T:.2f} seconds for 1 meter, real speed {(1/T):.2f} scale = {scale:.2f}")

            calibrate = False

        # Autonomous Driving (driving_mode == 1)
        if command_queue:
            # Fetch the first command from the queue
            auto_motion, desired_lin_disp, desired_ang_disp = command_queue.pop(0)
            lin_disp_error = desired_lin_disp
            ang_disp_error = desired_ang_disp
            tol = 0.001

            # Execute the command based on auto_motion
            if auto_motion in ['forward', 'backward']:
                left_encoder.reset()
                right_encoder.reset()

                # Set the target speeds based on direction (positive for forward, negative for backward)
                target_sign = 1 if auto_motion == 'forward' else -1

                left_disp, right_disp = lin2lr(lin_disp_error)

                # Set PID controllers for both wheels
                pid_left = PID(1.5, 0.25, 0.1, setpoint=left_disp, output_limits=(-0.6, 0.6))
                pid_right = PID(1.5, 0.25, 0.1, setpoint=right_disp, output_limits=(-0.6, 0.6))
                
                tolerance = 3  # Tolerance for displacement
                current_left_speed = 0
                current_right_speed = 0

                # Main control loop for forward/backward motion with PID
                while abs((left_disp + right_disp) / 2 - (left_encoder.value + right_encoder.value) / 2) > tolerance:
                    # Get the current encoder values
                    left_value = left_encoder.value
                    right_value = right_encoder.value

                    # Calculate PID output for both wheels
                    left_speed = pid_left(left_value) * target_sign
                    right_speed = pid_right(right_value) * target_sign

                    # Gradually adjust speeds using PID
                    current_left_speed = gradual_speed_change(current_left_speed, left_speed)
                    current_right_speed = gradual_speed_change(current_right_speed, right_speed)

                    # Set motor speeds
                    pibot.value = (current_left_speed, current_right_speed)

                    # If both wheels have exceeded the target displacement, stop the robot
                    if (left_encoder.value + right_encoder.value) > (right_disp + left_disp + tolerance):
                        break

                ld, _ = lr2linang(target_sign*left_encoder.value,target_sign*right_encoder.value)
                lin_disp_error = lin_disp_error - ld

                pibot.value = (0, 0)  # Stop the robot
                print(auto_motion, "Value", left_encoder.value, right_encoder.value)

            elif auto_motion == 'turning':
                left_encoder.reset()
                right_encoder.reset()
                ls, rs = 0, 0  # Initialize speeds
                
                left_disp, right_disp = ang2lr(desired_ang_disp)
                print("Left_disp:" + str(left_disp) + "Right_disp" + str(right_disp))
                # Set initial PID controllers for both wheels
                pid_left = PID(kp, ki, kd, setpoint=abs(left_disp), output_limits=(-0.95, 0.95))
                pid_right = PID(kp, ki, kd, setpoint=abs(right_disp), output_limits=(-0.95, 0.95))
                
                tolerance = 1  # Tolerance in encoder counts for stopping
                slow_down_distance = 0  # Distance (in encoder counts) to start slowing down
                
                # Main loop to handle turning with PID control
                while True:
                    # Get the current encoder values
                    left_value = left_encoder.value
                    right_value = right_encoder.value

                    # Calculate how far each wheel is from the target
                    left_error = abs(left_disp) - left_value
                    right_error = abs(right_disp) - right_value

                    # Start slowing down when close to the target
                    if left_error < slow_down_distance or right_error < slow_down_distance:
                        pid_left.output_limits = (-0.4, 0.4)  # Reduce speed limits when close to target
                        pid_right.output_limits = (-0.4, 0.4)

                    # Compute the PID output (speed adjustment) for each wheel
                    ls = pid_left(left_value)
                    rs = pid_right(right_value)

                    # Adjust direction for turning
                    if left_disp < 0:
                        ls = -abs(ls)  # Reverse for left wheel if turning left
                    if right_disp < 0:
                        rs = -abs(rs)  # Reverse for right wheel if turning right

                    # Set the robot motor speeds
                    pibot.value = (ls, rs)

                    # Stopping condition: Check if both wheels have reached their target displacement
                    if (left_error + right_error) / 2 < tolerance:
                        break

                if left_disp > 0:
                    sign = 1
                else:
                    sign = -1


                _, ad = lr2linang(sign*left_value,-1*sign*right_value)
                ang_disp_error = ang_disp_error - ad

                print("Angle error:" + str(ang_disp_error))
                pibot.value = (0, 0)
                    # Add a small delay to avoid busy-waiting
                time.sleep(0.01)

                print(f"Turn completed: Left encoder = {left_value}, Right encoder = {right_value}")

            # Once the command is executed, mark auto_motion as 'stop'
            auto_motion = 'stop'
            # To ensure integrity of every command, directly execute next command would make the robot busy and which the effectiveness was affected.
            time.sleep(0.1)

        else:
            # If no commands are in the queue, pause the robot and wait
            pibot.value = (0, 0)

        if driving_mode == 0:
            break

        # Small delay to avoid busy-waiting
        time.sleep(0.005)

def move_robot_manual():
    global use_pid, left_speed, right_speed, motion
    global kp, ki, kd
    flag_new_pid_cycle = True

    while True:
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
        if driving_mode == 1:
            break
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
    global lin_disp, ang_disp, auto_motion, command_queue

    lin_disp, ang_disp = float(request.args.get('lin_disp')), float(request.args.get('ang_disp'))
    print("Linear Disp:" + str(lin_disp) + ", Angular disp" + str(right_disp))
    if (lin_disp == 0 and ang_disp == 0):
        auto_motion = 'stop'
        command_queue = []
    elif (ang_disp != 0):
        auto_motion = 'turning'
    elif (lin_disp > 0):
        auto_motion = 'forward'
    elif (lin_disp < 0):
        auto_motion = 'backward'

    command_queue.append([auto_motion, lin_disp, ang_disp])

    return auto_motion

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

@app.route('/radius')
def set_radius():
    global radius
    radius = float(request.args.get('radius'))
    return str(radius) 

def lin2lr(lin_disp):
    unit_disp = abs(round(lin_disp / 0.00534))
    return unit_disp, unit_disp

def ang2lr(ang_disp):
    global radius
    unit_disp = (ang_disp / 180 * 3.1415926 * radius) / 0.00534
    unit_disp = round(abs(unit_disp))
    if(ang_disp > 0):
        target_sign = 1
    else:
        target_sign = -1
    return target_sign * unit_disp * -1, target_sign * unit_disp 

def lr2linang(l_disp, r_disp):
    global radius
    # Calculate linear displacement as the average of left and right displacements
    linear_disp = (l_disp + r_disp) / 2
    linear_disp = linear_disp * 0.00534
    
    # Calculate angular displacement as the difference divided by the distance between wheels
    angular_disp = (r_disp - l_disp) * 0.00534 / (2 * radius)
    angular_disp = angular_disp * 180 / 3.1415926
    
    return linear_disp, angular_disp


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
lin_disp, ang_disp = 0, 0
motion = ''
auto_motion = ''
driving_mode = 0
radius = 0.055
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
        if driving_mode == 1:
            move_robot_auto()
        if driving_mode == 0:
            move_robot_manual()
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")