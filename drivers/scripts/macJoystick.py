import pyglet
import sys
import time
# This code pipes std outputs to the jetson via ssh
# Make sure to pipe the outputs of this script into a named pipe:
# python3 macJoystick.py | ssh muddsub@192.168.1.2 "cat > ~/joystick_pipe"

# It also relies on the pyglet library, so make sure that is installed.
# For me pyglet only worked inside a virtual enviroment, so try that if
# this program is giving you an error that pyglet does not exist

# Convert joystick axis reading (-1-1) to a byte value (0-254) (256 would overflow and this way 0 is an even 127)
# This transformation is linear
def convert(x):
    x = min(max(x, -1), 1)  # Clamp x to be between -1 and 1
    return int(127 * (x + 1))

last_time = None

# Check for joysticks
if len(pyglet.input.get_joysticks()) > 0:
    # Use the first joystick
    joystick = pyglet.input.get_joysticks()[0]
    joystick.open()

    # print(f"Initialized Joystick : {joystick.get_name()}")

    # msg encoding:
    # byte 4: 1 for button pressed (otherwise 0)
    # byte 5: The value of joystick (ranges from 0-254)
    #       - For horizontal axies ("x"/0, "z"/2) 0 is far left
    #       - For vertical axies ("y"/1, "rz"/3) 0 is top
    # byte 6: 1 for a button event, 2 for axis event
    # byte 7: identifies the button or axis number
    msg = [0, 0, 0, 0, 0, 0, 0, 0]

    def send_msg(msg):
        sys.stdout.buffer.write(bytes(msg))
        sys.stdout.flush()
        global last_time
        last_time = time.time()

    # Event handlers for each type of joystick event
    def on_joybutton_press(joystick, button):
        global msg
        msg = [0] * 8
        msg[6] = 1
        msg[4] = 1
        msg[7] = button
        send_msg(msg)
    joystick.on_joybutton_press = on_joybutton_press

    def on_joybutton_release(joystick, button):
        global msg
        msg = [0] * 8
        msg[6] = 1
        msg[4] = 0
        msg[7] = button
        send_msg(msg)
    joystick.on_joybutton_release = on_joybutton_release

    def on_joyaxis_motion(joystick, axis, value):
        global msg
        msg = [0] * 8
        msg[6] = 2
        msg[5] = convert(value)
        # Left horizontal: "x"->0, Left Vertical: "y"->1,
        # Right Horizontal: "z"->2, Right Vertical: "rz"->3
        mapping = {"x": 0, "y": 1, "z": 2, "rz": 3}
        msg[7] = mapping[axis]
        send_msg(msg)
    joystick.on_joyaxis_motion = on_joyaxis_motion

    # If no messages have been sent in more the 0.1 seconds, resend the last one
    def resend_on_wait(dt):
        global msg
        if time.time() - last_time > 0.1:
            if len(pyglet.input.get_joysticks()) == 0: # if joystick is disconnected send 0s
                msg = [0, 0, 0, 0, 0, 0, 0, 0]
                
            send_msg(msg)
    pyglet.clock.schedule_interval(resend_on_wait, 0.1)

    last_time = time.time() # records last time a message was sent

    # Run everything, pass None means no windows created
    pyglet.app.run(None)
    
            
else:
    raise Exception("Controller not detected")

pyglet.app.exit()
