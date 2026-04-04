import serial
import time

class BusServo:
    def __init__(self, port='COM3', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port} at {baudrate} baud.")
        except Exception as e:
            print(f"Could not open serial port: {e}")

    def _calculate_checksum(self, id, length, cmd, params):
        # Checksum = ~ (ID + Length + Command + Data) & 0xFF
        sum_data = id + length + cmd + sum(params)
        return (~sum_data) & 0xFF

    def move(self, servo_id, position, duration):
        """
        Moves the servo to a position (0-1000) over a set time (ms).
        :param position: 0 to 1000 (roughly 0 to 240 degrees)
        :param duration: Time in milliseconds (higher = slower/smoother)
        """
        # Constrain inputs
        position = max(0, min(1000, position))
        duration = max(0, min(30000, duration))

        # Split 16-bit values into two 8-bit bytes (Little Endian)
        pos_l = position & 0xFF
        pos_h = (position >> 8) & 0xFF
        time_l = duration & 0xFF
        time_h = (duration >> 8) & 0xFF

        params = [pos_l, pos_h, time_l, time_h]
        cmd = 1 # SERVO_MOVE_TIME_WRITE
        length = 7 # ID + Length + Command + 4 params
        
        checksum = self._calculate_checksum(servo_id, length, cmd, params)
        
        # Build packet: Header(2) + ID + Length + Cmd + Params + Checksum
        packet = bytearray([0x55, 0x55, servo_id, length, cmd] + params + [checksum])
        
        self.ser.write(packet)

    def set_id(self, current_id, new_id):
        """
        Change servo ID.
        IMPORTANT: Only one servo should be connected when running this!
        """

        if not (0 <= new_id <= 253):
            print("Invalid ID. Must be 0–253.")
            return

        cmd = 13  # SERVO_ID_WRITE
        params = [new_id]
        length = 4  # ID + Length + Cmd + 1 param

        checksum = self._calculate_checksum(current_id, length, cmd, params)

        packet = bytearray([0x55, 0x55, current_id, length, cmd] + params + [checksum])

        self.ser.write(packet)
        time.sleep(0.1)

        print(f"Servo ID changed from {current_id} → {new_id}")
        
    def read_id(self, servo_id):
        """
        Reads the ID of a servo.
        Returns the ID if successful, None otherwise.
        """

        cmd = 14  # SERVO_ID_READ
        params = []
        length = 3  # ID + Length + Cmd

        checksum = self._calculate_checksum(servo_id, length, cmd, params)

        # Build request packet
        packet = bytearray([0x55, 0x55, servo_id, length, cmd, checksum])

        # Clear input buffer before sending
        self.ser.reset_input_buffer()

        # Send request
        self.ser.write(packet)

        # Wait briefly for response
        time.sleep(0.1)

        # Read response
        response = self.ser.read(7)  # Expected 7 bytes

        if len(response) != 7:
            print("No valid response received.")
            return None

        if response[0] != 0x55 or response[1] != 0x55:
            print("Invalid response header.")
            return None

        returned_id = response[5]

        print(f"Servo reported ID: {returned_id}")
        return returned_id

    def close(self):
        self.ser.close()

    def torque_off(self, servo_id):
            """
            Turns off the motor torque. You can move the servo by hand.
            Command: 20 (SERVO_LOAD_OR_UNLOAD_WRITE), Parameter: 0 (Unload)
            """
            params = [0] # 0 = Unload
            cmd = 20
            length = 4
            checksum = self._calculate_checksum(servo_id, length, cmd, params)
            
            packet = bytearray([0x55, 0x55, servo_id, length, cmd] + params + [checksum])
            print(packet)
            self.ser.write(packet)
            print(f"Servo {servo_id} torque DISABLED.")

    def torque_on(self, servo_id):
        """
        Locks the motor torque. The servo will hold its current position.
        Command: 20, Parameter: 1 (Load)
        """
        params = [1] # 1 = Load
        cmd = 20
        length = 4
        checksum = self._calculate_checksum(servo_id, length, cmd, params)
        
        packet = bytearray([0x55, 0x55, servo_id, length, cmd] + params + [checksum])
        self.ser.write(packet)
        print(f"Servo {servo_id} torque ENABLED.")

    def emergency_shutdown(self, servo_id):
        print("EMERGENCY STOP ACTIVATED")
        try:
            self.torque_off(servo_id)
        except:
            pass

    def is_alive(self, servo_id):
        try:
            pos = self.read_position(servo_id)
            return pos is not None
        except:
            return False

    def monitorDisconnection(self, servo_id, timeout=0.5):
        while True:
            alive = self.is_alive(servo_id)
            if not alive:
                print("Servo disconnected")
                self.emergency_shutdown(self, servo_id)
                break
            time.sleep(timeout)

# --- Example Usage ---
if __name__ == "__main__":
    my_robot = BusServo(port='COM9') # Replace with your COM port

    my_robot.read_id(1)
    my_robot.set_id(1, 4)
    my_robot.read_id(4)
    
    # Smooth move: ID 1 to position 500 (middle) over 1 second
    # my_robot.move(1, 500, 1000)
    # time.sleep(1.5)
    
    # Fast move: ID 1 to position 200 over 0.2 seconds
    # my_robot.move(1, 200, 200)
    
    my_robot.close()