import pygame
import sys

# Make sure to pipe the outputs of this script into a named pipe:
# python3 windowsJoystick.py | ssh muddsub@192.168.1.2 "cat > ~/joystick_pipe"

# Initialize Pygame
pygame.init() 

# Initialize the joystick module
pygame.joystick.init() # TO DO: Get rid of initial print message that pygame prints out

def convert(x):
    if x < 0:
        return int(x * 128 + 256)
    else:
        return int(x * 127)

    

# Check for joysticks
if pygame.joystick.get_count() > 0:
    # Use the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Initialized Joystick : {joystick.get_name()}")
    msg = [0, 0, 0, 0, 0, 0, 0, 0]

    # Main loop
    running = True
    while running:
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
            # my_list = [bytes(item) for item in msg]  
            sys.stdout.buffer.write(bytes(msg))
            sys.stdout.flush()
            # print(my_list)
            
pygame.quit()
