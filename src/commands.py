import time
from pymavlink import mavutil


class Commands:

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
