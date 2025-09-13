import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"  # Disable pygame welcome message
import pygame
import sys
import time
# This code pipes std outputs to the jetson via ssh
# Make sure to pipe the outputs of this script into a named pipe:
# python3 windowsJoystick.py | ssh muddsub@192.168.1.2 "cat > ~/joystick_pipe"

# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init() # TO DO: Get rid of initial print message that pygame prints out

# Convert joystick axis reading (-1-1) to a byte value (0-254) (256 would overflow and this way 0 is an even 127)
# This transformation is linear
def convert(x):
    x = min(max(x, -1), 1)  # Clamp x to be between -1 and 1
    return int(127 * (x + 1))

last_bytes = None
last_time = None

# Check for joysticks
if pygame.joystick.get_count() > 0:
    # Use the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # print(f"Initialized Joystick : {joystick.get_name()}")
    msg = [0, 0, 0, 0, 0, 0, 0, 0]
    
    last_time = time.time() # records last time a message was sent

    # Main loop
    running = True
    while running:
        if time.time() - last_time > 0.1: # if 0.1 seconds have passed since last message, resend message
            
            if pygame.joystick.get_init() == False: # if joystick is disconnected send 0s
                msg = [0, 0, 0, 0, 0, 3, 0, 0]
                
            sys.stdout.buffer.write(bytes(msg)) 
            sys.stdout.flush()
            last_time = time.time() # update last time message was sent
            
        for event in pygame.event.get():
            msg = [0] * 8
            if event.type == pygame.QUIT:
                running = False

            # Check for joystick movements
            if event.type == pygame.JOYAXISMOTION:
                msg[6] = 2
                msg[5] = convert(event.value)
                msg[7] = event.axis



                # print(f"Joystick Axis Moved: {event.axis} Value: {event.value}")

            if event.type == pygame.JOYBUTTONDOWN:
                msg[6] = 1
                msg[4] = 1
                msg[7] = event.button

                # print(f"Joystick Button Down: {event.button}")
            if event.type == pygame.JOYBUTTONUP:
                msg[6] = 1
                msg[4] = 0
                msg[7] = event.button

                # print(f"Joystick Button Up: {event.button}")

                # byte 6 is if 1 is a button event, 2 axis event
                # byte 4 and 5, value of message
                # byte 7 identifies the button or axis number
            sys.stdout.buffer.write(bytes(msg))
            sys.stdout.flush()

            # my_list = [bytes(item) for item in msg]
            # print(f'{msg} -> {bytes(msg)}')

pygame.quit()
