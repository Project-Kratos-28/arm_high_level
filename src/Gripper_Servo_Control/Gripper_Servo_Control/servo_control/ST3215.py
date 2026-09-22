from Gripper_Servo_Control.scservo_sdk import *


class ST3215:
    def __init__(self, device_name, baudrate=1000000, speed=2400, acceleration=50):
        self.portHandler = PortHandler(device_name)
        self.packetHandler = sms_sts(self.portHandler)

        self.SCS_MOVING_SPEED = speed
        self.SCS_MOVING_ACC = acceleration

        # Open serial port
        if not self.portHandler.openPort():
            raise Exception(f"Failed to open port: {device_name}")

        # Set baudrate
        if not self.portHandler.setBaudRate(baudrate):
            raise Exception(f"Failed to set baudrate: {baudrate}")

        print(f"Port {device_name} opened successfully at {baudrate} bps.")

    # ---------------------------------------------------------
    # SERVO CHECK
    # ---------------------------------------------------------

    def check_servo(self, servo_id):
        """
        Ping a single servo.
        Returns True if the servo responds correctly.
        """

        print(f"Checking Servo ID {servo_id}...")

        model, result, error = self.packetHandler.ping(servo_id)

        if result != COMM_SUCCESS:
            print(
                f"Not working, [ID:{servo_id:03d}] "
                f"Comm Error: {self.packetHandler.getTxRxResult(result)}"
            )
            return False

        if error != 0:
            print(
                f"Not working, [ID:{servo_id:03d}] "
                f"Hardware Error: {self.packetHandler.getRxPacketError(error)}"
            )
            return False

        print(
            f"[ID:{servo_id:03d}] Succeeded! "
            f"SC Servo model number: {model}"
        )

        return True

    # ---------------------------------------------------------
    # SCAN
    # ---------------------------------------------------------

    def scan(self, start=0, end=20):
        """Scan a range of servo IDs."""

        found = []

        print(f"Scanning IDs from {start} to {end}...")

        for i in range(start, end + 1):
            model, result, error = self.packetHandler.ping(i)

            if result == COMM_SUCCESS and error == 0:
                print(
                    f"Found servo at ID: {i:03d} | Model: {model}"
                )
                found.append(i)

        print(f"Scan complete. Found {len(found)} servos.")

        return found

    # ---------------------------------------------------------
    # CHANGE ID
    # ---------------------------------------------------------

    def change_id(self, current_id, new_id):
        """
        Change servo ID.

        Only have ONE servo connected while doing this.
        """

        if not (0 <= new_id <= 253):
            print("Error: New ID must be between 0 and 253.")
            return False

        print(
            f"Attempting to change servo ID "
            f"from {current_id} to {new_id}..."
        )

        # Unlock EPROM
        result, error = self.packetHandler.unLockEprom(current_id)

        if result != COMM_SUCCESS or error != 0:
            print("Failed to unlock EPROM.")
            return False

        # Change ID
        result, error = self.packetHandler.write1ByteTxRx(
            current_id,
            scs_id,
            new_id
        )

        if result != COMM_SUCCESS:
            print(
                f"Write Comm Error: "
                f"{self.packetHandler.getTxRxResult(result)}"
            )
            return False

        if error != 0:
            print(
                f"Write Hardware Error: "
                f"{self.packetHandler.getRxPacketError(error)}"
            )
            return False

        # Lock EPROM
        self.packetHandler.LockEprom(current_id)

        print(
            f"Successfully changed Servo ID "
            f"from {current_id} to {new_id}!"
        )

        return True

    # ---------------------------------------------------------
    # WRITE POSITION
    # ---------------------------------------------------------

    def write_angle(self, servo_id, angle):
        """
        Command servo to a position.

        angle here is the raw servo position value,
        NOT degrees.
        """

        scs_comm_result, scs_error = self.packetHandler.WritePosEx(
            servo_id,
            angle,
            self.SCS_MOVING_SPEED,
            self.SCS_MOVING_ACC
        )

        if scs_comm_result != COMM_SUCCESS:
            print(
                self.packetHandler.getTxRxResult(scs_comm_result)
            )

        elif scs_error != 0:
            print(
                self.packetHandler.getRxPacketError(scs_error)
            )

    # ---------------------------------------------------------
    # READ POSITION
    # ---------------------------------------------------------

    def read_position(self, servo_id):
        """
        Read the current raw servo position.

        Returns:
            int: Raw servo position
        """

        position = self.packetHandler.ReadPos(servo_id)

        if position == -1:
            print(f"Failed to read position from Servo ID {servo_id}")

        return position

    # ---------------------------------------------------------
    # READ POSITION + SPEED
    # ---------------------------------------------------------

    def read_angle_speed(self, servo_id):
        """
        Read current position and speed.

        Returns:
            position, speed
        """

        (
            scs_present_position,
            scs_present_speed,
            scs_comm_result,
            scs_error
        ) = self.packetHandler.ReadPosSpeed(servo_id)

        if scs_comm_result != COMM_SUCCESS:
            print(
                self.packetHandler.getTxRxResult(scs_comm_result)
            )

        if scs_error != 0:
            print(
                self.packetHandler.getRxPacketError(scs_error)
            )

        return scs_present_position, scs_present_speed

    # ---------------------------------------------------------
    # READ LOAD
    # ---------------------------------------------------------

    def read_load(self, servo_id):
        """
        Read the current servo load.

        Returns:
            int: Servo load
        """

        load = self.packetHandler.ReadLoad(servo_id)

        if load == -1:
            print(f"Failed to read load from Servo ID {servo_id}")

        return load

    # ---------------------------------------------------------
    # WHEEL MODE
    # ---------------------------------------------------------

    def wheel(self, servo_id, rot_speed):
        """
        Put servo into wheel/continuous rotation mode.
        """

        scs_comm_result, scs_error = self.packetHandler.WheelMode(
            servo_id
        )

        if scs_comm_result != COMM_SUCCESS:
            print(
                self.packetHandler.getTxRxResult(scs_comm_result)
            )

        elif scs_error != 0:
            print(
                self.packetHandler.getRxPacketError(scs_error)
            )

        scs_comm_result, scs_error = self.packetHandler.WriteSpec(
            servo_id,
            rot_speed,
            self.SCS_MOVING_ACC
        )

        if scs_comm_result != COMM_SUCCESS:
            print(
                self.packetHandler.getTxRxResult(scs_comm_result)
            )

        if scs_error != 0:
            print(
                self.packetHandler.getRxPacketError(scs_error)
            )

    # ---------------------------------------------------------
    # READ CURRENT
    # ---------------------------------------------------------

    def ReadCurrent(self, servo_id):
        """
        Read servo current.
        """

        (
            scs_current_current,
            scs_comm_result,
            scs_error
        ) = self.packetHandler.ReadCurrent(servo_id)

        if scs_comm_result != COMM_SUCCESS:
            print(
                self.packetHandler.getTxRxResult(scs_comm_result)
            )

        if scs_error != 0:
            print(
                self.packetHandler.getRxPacketError(scs_error)
            )

        return scs_current_current

    # ---------------------------------------------------------
    # READ TEMPERATURE
    # ---------------------------------------------------------

    def ReadTemp(self, servo_id):
        """
        Read servo temperature.

        Converts the raw temperature value to °C
        according to the ST3215 scaling.
        """

        (
            scs_current_temperature,
            scs_comm_result,
            scs_error
        ) = self.packetHandler.ReadTemp(servo_id)

        if scs_comm_result != COMM_SUCCESS:
            print(
                self.packetHandler.getTxRxResult(scs_comm_result)
            )

        if scs_error != 0:
            print(
                self.packetHandler.getRxPacketError(scs_error)
            )

        scs_current_temperature *= 6.5

        return scs_current_temperature

    # ---------------------------------------------------------
    # CLOSE PORT
    # ---------------------------------------------------------

    def close(self):
        self.portHandler.closePort()
        print("Port closed.")