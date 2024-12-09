# Import libraries
import RPi.GPIO as GPIO
import time
import pygame
import subprocess
from trill_lib import TrillLib

import serial

# initalise serial communication
ser = serial.Serial('/dev/ttyUSB0', 115200)

'''
Setup for the Ears
'''
# Set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)
# Set pins 11 & 12 as outputs and right/left ears
GPIO.setup(12,GPIO.OUT)
right_ear = GPIO.PWM(12,50)
GPIO.setup(11,GPIO.OUT)
left_ear = GPIO.PWM(11,50)
# Start PWM on servos, setting them to closed position
right_ear.start(6)
left_ear.start(6)


'''
Setup for the image display
'''
# Initialise PyGame
pygame.init()
# Set up the LCD display
lcd_size = (800, 480)
# screen = pygame.display.set_mode(lcd_size, pygame.FULLSCREEN)
screen = pygame.display.set_mode(lcd_size)


'''
Initialize Pygame and the joystick
'''
pygame.joystick.init()

DEADZONE = 0.1

def apply_deadzone(value, deadzone):
    return 0.0 if abs(value) < deadzone else value

def scale_input(value, in_min=-1.0, in_max=1.0, out_min=-1.0, out_max=1.0):
    return((value - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to controller: {joystick.get_name()}")
else:
    print("No controller connected.")
    pygame.quit()
    exit()


'''
Functions to enable video playback and displaying of images
'''
def play_video(file_name):
    # Use subprocess.Popen to play the video without allowing the next command to happen before the end of the video
    subprocess.Popen(f"cvlc --fullscreen --no-osd -I dummy /home/harry/Animation/{file_name}", shell=True)

def display_image(image_path):
    # Display an image
    image = pygame.image.load(f"/home/harry/Animation/{image_path}")
    image = pygame.transform.scale(image, lcd_size)
    screen.blit(image, (0, 0))
    pygame.display.update()


'''
Setup for the trill
'''
touchSensor = TrillLib(1, "craft", 0x30)
touchSensor.printDetails()
print()
touchSensor.setMode("DIFF")


'''
Setup for ultrasonic sensor
'''
TRIG_PIN = 31
ECHO_PIN = 29
THRESHOLD_DISTANCE = 30  # 30cm away
mei_angry = False
anger_allowed = True

# GPIO setup
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)


'''
Setup for the microphone
'''
GPIO.setup(37, GPIO.IN)




# Initiate: Ears flap and eyes open to show that Mei has woken up
def initiate():
    display_image("closed.jpg")
    time.sleep(2)

    right_ear.ChangeDutyCycle(2.2)
    left_ear.ChangeDutyCycle(11)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)

    time.sleep(0.5)

    right_ear.ChangeDutyCycle(2.8)
    left_ear.ChangeDutyCycle(10)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)

    play_video("open_eyes.mp4")
    time.sleep(0.5)
    display_image("neutral.jpg")



# Happy: Ears flap up and eyes become happy
def happy():
    right_ear.ChangeDutyCycle(1.8)
    left_ear.ChangeDutyCycle(12)
    time.sleep(0.3)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("happy.mp4")
    time.sleep(0.5)
    display_image("happy.jpg")

# Happy -> neutral
def happy_neutral():
    right_ear.ChangeDutyCycle(2.8)
    left_ear.ChangeDutyCycle(10)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("happy_neutral.mp4")
    time.sleep(0.5)
    display_image("neutral.jpg")


# Sad: Close ears and eyes become sad
def sad():
    right_ear.ChangeDutyCycle(4)
    left_ear.ChangeDutyCycle(8.5)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("sad.mp4")
    time.sleep(0.5)
    display_image("sad.jpg")

# Sad -> neutral
def sad_neutral():
    right_ear.ChangeDutyCycle(2.8)
    left_ear.ChangeDutyCycle(10)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("sad_neutral.mp4")
    time.sleep(0.5)
    display_image("neutral.jpg")


# Angry: Ears half close and eyes become angry
def angry():
    right_ear.ChangeDutyCycle(3.4)
    left_ear.ChangeDutyCycle(9.3)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("angry.mp4")
    time.sleep(0.5)
    display_image("angry.jpg")

# Angry -> neutral
def angry_neutral():
    right_ear.ChangeDutyCycle(2.8)
    left_ear.ChangeDutyCycle(10)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("angry_neutral.mp4")
    time.sleep(0.5)
    display_image("neutral.jpg")


# just play blink animation
def blink():
    play_video("blink.mp4")


# Finish: close eyes and ears and clean things up at the end
def sleep():
    right_ear.ChangeDutyCycle(6)
    left_ear.ChangeDutyCycle(6)
    time.sleep(0.2)
    right_ear.ChangeDutyCycle(0)
    left_ear.ChangeDutyCycle(0)
    play_video("close_eyes.mp4")
    time.sleep(0.5)
    display_image("closed.jpg")
    # cleanup
    left_ear.stop()
    right_ear.stop()
    GPIO.cleanup()
    
    # wait 2 secs
    time.sleep(2)

    # Quit window
    print("Exiting Program")
    pygame.display.quit()
    pygame.quit()
    ser.close()


# Code ran when touch detected on top of head. As person strokes further back, Mei's ears prick up further
def stroke():
    # start timing to know when to end stroke
    last_stroked_time = time.time()
    # make happy
    play_video("happy.mp4")
    time.sleep(0.5)
    display_image("happy.jpg")
    time.sleep(0.5)
    # while loop to constantly update ear position according to thy word
    while(1):
        touchSensor.readTrill()
        # Assuming rawData is a list and has enough elements
        try:
            # Convert the rawData items to floats
            first_touch = float(touchSensor.rawData[21])
            second_touch = float(touchSensor.rawData[23])
            third_touch = float(touchSensor.rawData[19])
            forth_touch = float(touchSensor.rawData[17])
            # Check each of the 4 touch sensors in reverse order
            if forth_touch > 0.1:
                right_ear.ChangeDutyCycle(1.8)
                left_ear.ChangeDutyCycle(12)
                time.sleep(0.1)
                right_ear.ChangeDutyCycle(0)
                left_ear.ChangeDutyCycle(0)
                last_stroked_time = time.time()
            elif third_touch > 0.1:
                right_ear.ChangeDutyCycle(2.5)
                left_ear.ChangeDutyCycle(11)
                time.sleep(0.1)
                right_ear.ChangeDutyCycle(0)
                left_ear.ChangeDutyCycle(0)
                last_stroked_time = time.time()
            elif second_touch > 0.1:
                right_ear.ChangeDutyCycle(3)
                left_ear.ChangeDutyCycle(10)
                time.sleep(0.1)
                right_ear.ChangeDutyCycle(0)
                left_ear.ChangeDutyCycle(0)
                last_stroked_time = time.time()
            elif first_touch > 0.1:
                right_ear.ChangeDutyCycle(3.4)
                left_ear.ChangeDutyCycle(9.3)
                time.sleep(0.1)
                right_ear.ChangeDutyCycle(0)
                left_ear.ChangeDutyCycle(0)
                last_stroked_time = time.time()
            else:
                # close ears
                right_ear.ChangeDutyCycle(6)
                left_ear.ChangeDutyCycle(6)
                time.sleep(0.1)
                right_ear.ChangeDutyCycle(0)
                left_ear.ChangeDutyCycle(0)
            # print the touch values for maintenance
            print(f"Items 23, 21, 19 and 17 as floats: {round(first_touch, 2)}, {round(second_touch, 2)}, {round(third_touch, 2)}, {round(forth_touch, 2)}")
            # once it has been 5 seconds since being stroked
            if time.time() - last_stroked_time > 5:
                # animation to go back to neutral
                right_ear.ChangeDutyCycle(2.8)
                left_ear.ChangeDutyCycle(10)
                time.sleep(0.2)
                right_ear.ChangeDutyCycle(0)
                left_ear.ChangeDutyCycle(0)
                play_video("happy_neutral.mp4")
                time.sleep(0.5)
                display_image("neutral.jpg")
                # end while loop
                break
        except IndexError:
            print("Not enough data in rawData")
        except ValueError:
            print("Error: Could not convert data to floats")
        time.sleep(0.1)
    
    print("Finished")


def measure_distance():
    """
    Measures the distance using the ultrasonic sensor.
    Returns the distance in cm.
    """
    # Send a 10us pulse to the TRIG pin
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_PIN, False)

    # Measure the time for the echo
    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    # Calculate the distance
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound: 34300 cm/s

    return distance





def brain():
    global mei_angry
    while(1):
        
        '''
        Do stuff, start, initiate, run try for buttons
        '''
        print("waiting...")
        time.sleep(2)
        print("initiating")
        initiate()
        print("initiated")
        
        try:
            while True:
                pygame.event.pump()  # Process pygame events

                # Read and apply scale and deadzone for movment joysticks

                js1 = scale_input(apply_deadzone(joystick.get_axis(0), DEADZONE))
                js2 = scale_input(apply_deadzone(joystick.get_axis(1), DEADZONE))


                # js3 = scale_input(apply_deadzone(joystick.get_axis(2), DEADZONE))
                # js4 = scale_input(apply_deadzone(joystick.get_axis(3), DEADZONE))

                # Process data to be sent

                data_to_send = f"J{js1:.2f},{js2:.2f}\n"

                # Send that data through the serial

                ser.write(data_to_send.encode())
                print("Sent:", data_to_send.strip())

                
                
                # Triangle and cross button control happiness
                triangle_button = joystick.get_button(0)
                if triangle_button:
                    print("triange pressed. Making happy!")
                    happy()
                cross_button = joystick.get_button(2)
                if cross_button:
                    print("Cross pressed. Making neutral, from happy")
                    happy_neutral()

                # Right trigger and button control anger
                right_trigger = joystick.get_button(5)
                if right_trigger:
                    print("RT pressed. Making Angry")
                    angry()
                right_button = joystick.get_button(7)
                if right_button:
                    print("RB pressed. Making neutral from angry")
                    angry_neutral()

                # Left trigger and button control sadness
                left_trigger = joystick.get_button(4)
                if left_trigger:
                    print("LT pressed. Making sad")
                    sad()
                left_button = joystick.get_button(6)
                if left_button:
                    print("LB pressed. Making neutral from sad")
                    sad_neutral()

                # Circle button blinks
                circle_button = joystick.get_button(1)
                if circle_button:
                    print("Circle pressed. Blinking.")
                    blink()

                # Square button sends to sleep
                square_button = joystick.get_button(3)
                if square_button:
                    print("Square pressed. Sleeping")
                    sleep()


                # Debugging:

                # dpad_x = joystick.get_axis(0)
                # dpad_y = joystick.get_axis(1)
                
               
                # if dpad_x < -0.5:
                #     print("Rotates left: -0.1")
                #     ser.write(b'q')
                #     print("Sent:", data_to_send.strip())

                # elif dpad_x > 0.5:
                #     print("Rotates right: +0.1")
                #     ser.write(b'e')
                #     print("Sent:", data_to_send.strip())

                # if dpad_y < -0.5:
                #     print("Toggles motors on or off")
                #     ser.write(b'M')
                #     print("Sent:", data_to_send.strip())

                # elif dpad_y > 0.5:
                #     print("Recalibrates gyro to 0,0")
                #     ser.write(b'c')
                #     print("Sent:", data_to_send.strip())
                
                
                # Read trill and convert data to 4 floats
                touchSensor.readTrill()
                first_touch = float(touchSensor.rawData[21])
                second_touch = float(touchSensor.rawData[23])
                third_touch = float(touchSensor.rawData[19])
                forth_touch = float(touchSensor.rawData[17])
                # If any of these readings are above a threshold, run stroke function
                if first_touch > 0.2 or second_touch > 0.2 or third_touch > 0.2 or forth_touch > 0.2:
                    stroke()

                # Measure distance and get angry if anything is too close
                distance = measure_distance()
                if distance < THRESHOLD_DISTANCE and mei_angry == False and anger_allowed == True:
                    angry()
                    mei_angry = True
                elif distance > THRESHOLD_DISTANCE and mei_angry == True:
                    angry_neutral()
                    mei_angry = False

                # If the microphone detects a sound too loud, make sad for 5 seconds
                if GPIO.input(37) == GPIO.HIGH:
                    global anger_allowed = False
                    sad()
                    time.sleep(5)
                    sad_neutral()
                    time.sleep(0.5)
                    global anger_allowed = True



                # Delay to avoid rapid toggling
                pygame.time.delay(100)
                
        except IndexError:
            print("Not enough data in rawData")
        except ValueError:
            print("Error: Could not convert data to floats")


brain()
