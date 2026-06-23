from pymavlink import mavutil
import time
from commands import Commands

ARM_DISARM_COMMAND = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
TAKEOFF_COMMAND = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
LAND_COMMAND = mavutil.mavlink.MAV_CMD_NAV_LAND
RTL_COMMAND = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH

"""
SAFETY CHECKS AND MODE SETTERS

<#> Safety Guard Functions
check_battery()

<#> Mode Functions
set_mode(mode)
ensure_mode(mode) - checks if the drone is in the specified mode and if not sets the mode to the specified mode
get_mode() - returns the current mode of the drone


"""

"""
ALL AVAILABLE VEHICLE COMMANDS


<#> Basic Vehicle State Functions
get_flight_status() will return a dictionary of the following functions
get_mode()
get_armed_state()
get_battery_status()
get_gps_status()
get_global_position()
get_local_position()
get_attitude()
get_heading()
get_altitude()
get_rc_channels()
get_flight_status()

<#> Mode Functions
set_mode(mode_name)
enter_guided()
enter_loiter()
enter_alt_hold()
enter_stabilize()
enter_rtl()
enter_land()
ensure_mode(mode_name)

<#> Arm/Disarm Functions
arm() - return if successful or the reason if unccessful (String)
disarm()
safe_disarm() - Checks if landed and safe to disarm. Returns nothing?

<#> Take Off/Land
*MAKE SURE THE DRONE IS IN GUIDED MODE FOR THESE FUNCTION 
*OTHERWISE THE DRONE WILL BEHAVE UNEXPECTEDLY
*EX - DRONE TOOK OFF WITH FULL THROTTLE WHEN USING THE TAKE OFF FUNCTION
takeoff(target_altitude_m)
land()
rtl()
rtl_no_land()
check_rtl()??
guided_takeoff(target_altitude_m, tolerance=0.5, timeout=20) - Will wait till the drone reaches the target altitude

<#> Guided Movement Functions
*These functions will only work in GUIDED mode
send_velocity_body(forward, right, down)
stop_guided_motion()
send_velocity_ned(vx, vy, vz)
send_position_ned(north, east, down)
send_global_position(lat, lon, alt, max_distance=5km)
hold_position()
set_yaw(yaw_deg)
condition_yaw(yaw_deg, relative=False)

<#> Movement Convinience Functions
*Built using the Guided Movement Functions
*Only works in GUIDED mode
move_up(speed_ms=0.3)
move_down(speed_ms=0.3)
move_forward(speed_ms=0.5)
move_backward(speed_ms=0.5)
move_right(speed_ms=0.5)
move_left(speed_mps=0.5)
yaw_left(angle, rate=200 degrees/s)
yaw_right(angle, rate=200 degrees/s)
move_forward_for(duration_s, speed_ms=0.5) - Code will be stuck at this function till the drone is done moving?
move_backward_for(duration_s, speed_ms=0.5) - Code will be stuck at this function till the drone is done moving?
move_ned_for(vx, vy, vz, duration_s) - Code will be stuck at this function till the drone is done moving?
move_forward_distance(distance_m, speed_mps=0.5) - Code will be stuck at this function till the drone is done moving?
move_ned_distance(north_m, east_m, speed_mps=0.5) - Code will be stuck at this function till the drone is done moving?


"""

class Drone:
    def __init__(self, connection_string="udpin:localhost:14551"):
        """
        Runs automatically when you create a Drone object.

        Example:
            drone1 = Drone()
            drone2 = Drone("COM7")
        """

        self.connection_string = connection_string
        self.master = None

    def ack_was_accepted(self, ack):
        """
        Checks if the COMMAND_ACK says the command was accepted.
        """

        return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    def wait_for_command_ack(self, command_id, timeout=5):
        """
        Waits for a COMMAND_ACK for a specific command.

        Returns:
            ("success", msg)
                if the correct ACK was found and accepted

            ("command denied", msg)
                if the correct ACK was found but not accepted

            ("ack not found", None)
                if no matching ACK was found before timeout
        """

        start_time = time.time()

        while time.time() - start_time < timeout:
            remaining_time = timeout - (time.time() - start_time)

            msg = self.master.recv_match(
                type="COMMAND_ACK",
                blocking=True,
                timeout=remaining_time
            )

            if msg is None:
                break

            if msg.get_type() == "BAD_DATA":
                continue

            if msg.command != command_id:
                print(f"Ignoring ACK for different command: {msg.command}")
                continue

            # At this point, we found the ACK for the command we care about.
            if self.ack_was_accepted(msg):
                return "success", msg
            else:
                return "command denied", msg

        return "ack not found", None

    def connect(self, connection_string=None):
        """
        Connects to the drone.

        If a connection_string is passed in here, it overrides the one
        stored when the Drone object was created.
        """

        #If the user passes in a connection string when calling the function then that will be used. Otherwise the one that was passed when creating the drone object will be used.
        #If the user doesn't pass in a connection string when creating the drone object when calling the connect function, the default one will be used
        if connection_string is not None:
            self.connection_string = connection_string

        print(f"Connecting to drone using {self.connection_string}...")

        self.master = mavutil.mavlink_connection(self.connection_string)

        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()

        print("Connected!")

        return self.master
    
    def has_connection_object(self):
        return self.master is not None


#<#> Mode Functions

    def set_mode(self, mode_name):
        """
        Sends a mode change command.

        Example:
            drone.set_mode("GUIDED")
            drone.set_mode("LOITER")

        Returns:
            ("success", None)
            ("mode not available", None)
            ("Please connect to drone first", None)
        """

        if not self.has_connection_object():
            return "Please connect to drone first", None

        mode_name = mode_name.upper()

        mode_mapping = self.master.mode_mapping()

        if mode_name not in mode_mapping:
            print(f"Mode {mode_name} is not available.")
            print(f"Available modes: {list(mode_mapping.keys())}")
            return "mode not available", None

        mode_id = mode_mapping[mode_name]

        print(f"Setting mode to {mode_name}...")

        self.master.set_mode(mode_id)

        return "success", None

    def ensure_mode(self, mode_name, timeout=5):
        """
        Makes sure the drone is actually in the requested mode.

        If the drone is already in the requested mode, it returns success.
        If not, it sends a mode change command and waits until the drone's
        heartbeat confirms the new mode.

        Returns:
            ("success", None)
            ("mode not available", None)
            ("mode change failed", None)
            ("Please connect to drone first", None)
        """

        if not self.has_connection_object():
            return "Please connect to drone first", None

        mode_name = mode_name.upper()

        print(f"Checking if drone is in {mode_name} mode...")

        start_time = time.time()

        while time.time() - start_time < timeout:
            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            current_mode = mavutil.mode_string_v10(msg)

            if current_mode == mode_name:
                print(f"Drone is already in {mode_name} mode.")
                return "success", None

            break

        mode_status, mode_ack = self.set_mode(mode_name)

        if mode_status != "success":
            return mode_status, mode_ack

        print(f"Waiting for drone to enter {mode_name} mode...")

        start_time = time.time()

        while time.time() - start_time < timeout:
            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            current_mode = mavutil.mode_string_v10(msg)

            print(f"Current mode: {current_mode}")

            if current_mode == mode_name:
                print(f"Drone entered {mode_name} mode.")
                return "success", None

        print(f"Drone did not enter {mode_name} mode.")
        return "mode change failed", None

#<#> Arm/Disarm Functions

    def arm(self):
        """
        Sends an arm command.

        Returns:
            ("success", msg)
            ("command denied", msg)
            ("ack not found", None)
        """
        if self.has_connection_object():

            print("Sending arm command...")

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                ARM_DISARM_COMMAND,
                0,
                1,      # 1 means arm
                0,      # normal arm, do not force
                0, 0, 0, 0, 0
            )

            status, ack = self.wait_for_command_ack(ARM_DISARM_COMMAND, timeout=5)

            return status, ack
        else:
            return "Please connect to drone first", None

    def disarm(self):
        """
        Sends a disarm command.

        Returns:
            ("success", msg)
            ("command denied", msg)
            ("ack not found", None)
        """

        print("Sending disarm command...")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            ARM_DISARM_COMMAND,
            0,
            0,      # 0 means disarm
            0,      # normal disarm
            0, 0, 0, 0, 0
        )

        status, ack = self.wait_for_command_ack(ARM_DISARM_COMMAND)

        return status, ack


#<#> Take Off/Land

    def takeoff(self, target_altitude_m):
        """
        Sends a takeoff command.

        Assumes:
            - Drone is connected
            - Drone is armed
            - Drone is in GUIDED mode
            - *These are checked before sending the takeoff command.
            - The function will not attempt to fix any of these issues (Except for setting guided mode). It will just return an error message.*

        Returns:
            ("success", msg)
            ("command denied", msg)
            ("ack not found", None)
            ("Please connect to drone first", None)
        """

        if not self.has_connection_object():
            return "Please connect to drone first", None
        
        ensure_mode_reponse = self.ensure_mode("GUIDED")

        if ensure_mode_reponse[0] != "success":
            return ensure_mode_reponse

        print(f"Sending takeoff command to {target_altitude_m} meters...")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            TAKEOFF_COMMAND,
            0,
            0, 0, 0, 0,      # params 1-4 unused for Copter
            0,               # latitude, ignored
            0,               # longitude, ignored
            target_altitude_m # altitude
        )

        status, ack = self.wait_for_command_ack(TAKEOFF_COMMAND, timeout=5)

        return status, ack



        def move_forward(self, distance_m, speed_m_s=2):
            """
            Moves the drone forward by a certain distance at a controlled speed.

            Returns:
                ("success", ack)
                ("command denied", ack)
                ("ack not found", None)
            """

            speed_status, speed_ack = self.set_guided_speed(self.master, speed_m_s)

            if speed_status != "success":
                return speed_status, speed_ack

            print(f"Moving forward {distance_m} meters at about {speed_m_s} m/s...")

            self.master.mav.set_position_target_local_ned_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                3576,
                distance_m,
                0,
                0,
                0, 0, 0,
                0, 0, 0,
                0,
                0
            )

            return "success", None

    def land(self):
        """
        Sends a land command.

        Returns:
            ("success", msg)
            ("command denied", msg)
            ("ack not found", None)
            ("Please connect to drone first", None)
        """

        if not self.has_connection_object():
            return "Please connect to drone first", None

        print("Sending land command...")

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            LAND_COMMAND,
            0,
            0, 0, 0, 0,  # params 1-4 unused here
            0,           # latitude, 0 means current location / ignored
            0,           # longitude, 0 means current location / ignored
            0            # altitude
        )

        status, ack = self.wait_for_command_ack(LAND_COMMAND, timeout=5)

        return status, ack


#<#> Guided Movement Functions

    def send_velocity_body(self, forward_mps, right_mps, down_mps):
        """
        Sends a velocity command in the drone's body frame.

        forward_mps:
            Positive = move forward
            Negative = move backward

        right_mps:
            Positive = move right
            Negative = move left

        down_mps:
            Positive = move down
            Negative = move up

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        if not self.has_connection_object():
            return "Please connect to drone first", None

        # Use velocity only.
        # Ignore position, acceleration, yaw, and yaw rate.
        velocity_only_type_mask = 0b110111000111

        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            velocity_only_type_mask,

            0, 0, 0,  # position x, y, z ignored

            forward_mps,
            right_mps,
            down_mps,

            0, 0, 0,  # acceleration ignored
            0,        # yaw ignored
            0         # yaw rate ignored
        )

        return "success", None

    def stop_guided_motion(self):
        """
        Stops guided velocity movement.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(0, 0, 0)


#<#> Movement Convinience Functions

    def move_up(self, speed_mps=0.3):
        """
        Moves the drone upward at the given speed.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(0, 0, -speed_mps)

    def move_down(self, speed_mps=0.3):
        """
        Moves the drone downward at the given speed.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(0, 0, speed_mps)

    def move_forward(self, speed_mps=0.5):
        """
        Moves the drone forward at the given speed.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(speed_mps, 0, 0)

    def move_backward(self, speed_mps=0.5):
        """
        Moves the drone backward at the given speed.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(-speed_mps, 0, 0)

    def move_right(self, speed_mps=0.5):
        """
        Moves the drone right at the given speed.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(0, speed_mps, 0)

    def move_left(self, speed_mps=0.5):
        """
        Moves the drone left at the given speed.

        Returns:
            ("success", None)
            ("guided mode required", None)
            ("Please connect to drone first", None)
        """

        return self.send_velocity_body(0, -speed_mps, 0)


# Height adjust global (Go to the altitude that was passed in)

# Height adjust local (Adjust the altitude by the value passed in)
#  - Two values are passed in one is the value to adjust the altitude the second is to tell it whether to increase or decrease the altitude
#  - If the adjusted altitude is above the set max or below 1m above the ground then do not execute and return the relavent issue

# Take off
