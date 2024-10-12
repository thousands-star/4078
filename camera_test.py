from picamera2 import Picamera2
import time

# Initialize the Picamera2 object
picam2 = Picamera2()

# Configure the camera settings (optional)
camera_config = picam2.create_still_configuration()
picam2.configure(camera_config)

# Start the camera
picam2.start()
print("Camera started, adjusting settings...")

# Allow some time for camera settings to adjust
time.sleep(2)

# Capture an image and save it
picam2.capture_file("/home/pi/Desktop/image.jpg")
print("Image captured!")

# Stop the camera
picam2.stop()
