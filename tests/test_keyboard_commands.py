# Import all the basic drone control functions
from pymavlink import mavutil
from commands import *
import time
import sys
import keyboard
from drone import Drone

#Connect to the drone
#Default parameter: "udp:127.0.0.1:14551"
#Returns object that will be used to refer the drone for other methods
#The python script is connecting to the MAVLink stream forwarded by mission plnanner. It's not connecting to the drone directly
drone1 = Drone("udpin:localhost:14551")
SPEED_MPS = 0.4

# print("Heartbeat from system (System %u component %u)" % (drone1.target_system, drone1.target_component))

drone1.connect()

arm_response = drone1.arm()

if arm_response[0] != "success":
    print(arm_response)
    sys.exit()

take_off_response = drone1.takeoff(0.5)


if (take_off_response[0] == "command denied"):
    print(take_off_response)
    drone1.disarm()
    sys.exit()

time.sleep(5)

print("Keyboard control started")
print("Up arrow    = forward")
print("Down arrow  = backward")
print("Left arrow  = left")
print("Right arrow = right")
print("Space       = stop")
print("L           = land")
print("Q           = quit and land")

try:
    while True:
        if keyboard.is_pressed("up"):
            drone1.move_forward(SPEED_MPS)

        elif keyboard.is_pressed("down"):
            drone1.move_backward(SPEED_MPS)

        elif keyboard.is_pressed("left"):
            drone1.move_left(SPEED_MPS)

        elif keyboard.is_pressed("right"):
            drone1.move_right(SPEED_MPS)

        else:
            drone1.stop_guided_motion()

        if keyboard.is_pressed("space"):
            drone1.stop_guided_motion()

        if keyboard.is_pressed("l") or keyboard.is_pressed("q"):
            drone1.stop_guided_motion()
            drone1.land()
            time.sleep(8)
            drone1.disarm()
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    drone1.stop_guided_motion()
    drone1.land()
    time.sleep(8)
    drone1.disarm()







