import RPi.GPIO as GPIO
import time
import sys

# Define GPIO pins for flow sensors
FLOW_SENSOR_1 = 5
FLOW_SENSOR_2 = 6
FLOW_SENSOR_3 = 16
FLOW_SENSOR_4 = 17
FLOW_SENSOR_5 = 22
FLOW_SENSOR_6 = 23
FLOW_SENSOR_7 = 24
FLOW_SENSOR_8 = 25
FLOW_SENSOR_9 = 26
FLOW_SENSOR_10 = 27
# FLOW_SENSOR_11 = 23
# FLOW_SENSOR_12 = 24

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(FLOW_SENSOR_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_6, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_7, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_8, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_9, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FLOW_SENSOR_10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Initialize global counters
count_1 = 0
count_2 = 0
count_3 = 0
count_4 = 0
count_5 = 0
count_6 = 0
count_7 = 0
count_8 = 0
count_9 = 0
count_10 = 0


# Define callback function for flow sensor 1
def count_pulse_1(channel):
    global count_1
    count_1 += 1


# Define callback function for flow sensor 2
def count_pulse_2(channel):
    global count_2
    count_2 += 1


# Define callback function for flow sensor 1
def count_pulse_3(channel):
    global count_3
    count_3 += 1


# Define callback function for flow sensor 2
def count_pulse_4(channel):
    global count_4
    count_4 += 1


# Define callback function for flow sensor 1
def count_pulse_5(channel):
    global count_5
    count_5 += 1


# Define callback function for flow sensor 2
def count_pulse_6(channel):
    global count_6
    count_6 += 1


# Define callback function for flow sensor 1
def count_pulse_7(channel):
    global count_7
    count_7 += 1


# Define callback function for flow sensor 2
def count_pulse_8(channel):
    global count_8
    count_8 += 1


# Define callback function for flow sensor 1
def count_pulse_9(channel):
    global count_9
    count_9 += 1


# Define callback function for flow sensor 2
def count_pulse_10(channel):
    global count_10
    count_10 += 1


# Add event detection for flow sensors
GPIO.add_event_detect(FLOW_SENSOR_1, GPIO.BOTH, callback=count_pulse_1)
GPIO.add_event_detect(FLOW_SENSOR_2, GPIO.BOTH, callback=count_pulse_2)
GPIO.add_event_detect(FLOW_SENSOR_3, GPIO.BOTH, callback=count_pulse_3)
GPIO.add_event_detect(FLOW_SENSOR_4, GPIO.BOTH, callback=count_pulse_4)
GPIO.add_event_detect(FLOW_SENSOR_5, GPIO.BOTH, callback=count_pulse_5)
GPIO.add_event_detect(FLOW_SENSOR_6, GPIO.BOTH, callback=count_pulse_6)
GPIO.add_event_detect(FLOW_SENSOR_7, GPIO.BOTH, callback=count_pulse_7)
GPIO.add_event_detect(FLOW_SENSOR_8, GPIO.BOTH, callback=count_pulse_8)
GPIO.add_event_detect(FLOW_SENSOR_9, GPIO.BOTH, callback=count_pulse_9)
GPIO.add_event_detect(FLOW_SENSOR_10, GPIO.BOTH, callback=count_pulse_10)

try:
    while True:
        # Read and print the current state of flow sensors
        print(f"Waterflow1 : {GPIO.input(FLOW_SENSOR_1)}")
        print(f"Waterflow2 : {GPIO.input(FLOW_SENSOR_2)}")
        print(f"Waterflow3 : {GPIO.input(FLOW_SENSOR_3)}")
        print(f"Waterflow4 : {GPIO.input(FLOW_SENSOR_4)}")
        print(f"Waterflow5 : {GPIO.input(FLOW_SENSOR_5)}")
        print(f"Waterflow6 : {GPIO.input(FLOW_SENSOR_6)}")
        print(f"Waterflow7 : {GPIO.input(FLOW_SENSOR_7)}")
        print(f"Waterflow8 : {GPIO.input(FLOW_SENSOR_8)}")
        print(f"Waterflow9 : {GPIO.input(FLOW_SENSOR_9)}")
        print(f"Waterflow10 : {GPIO.input(FLOW_SENSOR_10)}")
        time.sleep(1)

except KeyboardInterrupt:
    print("\nCaught keyboard interrupt! Exiting...")
    GPIO.cleanup()
    sys.exit()
