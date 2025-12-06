import serial
import time

# Set up the serial communication
arduino = serial.Serial('/dev/ttyACM0', 9600)  # Change '/dev/ttyACM0' to your port
time.sleep(2)  # Wait for the Arduino to initialize

def make_small_circle():
    try:
        while True:
            # Send values to make a small circle (Right motor slower than Left motor)
            # You can adjust the values here to make the circle larger or smaller
            arduino.write(b"120,-100\n")  # Right motor is slower (turning left)
            time.sleep(1)

            arduino.write(b"120,100\n")   # Right motor is faster (turning right)
            time.sleep(1)

    except KeyboardInterrupt:
        # Stop the motors when the program is interrupted
        arduino.write(b"0,0\n")
        print("Stopped the motors.")
        arduino.close()

# Start making the small circle
make_small_circle()
