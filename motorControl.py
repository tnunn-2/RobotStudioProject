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
        """Careful: Use only when one servo is connected!"""
        packet = bytearray([0x55, 0x55, current_id, 0x04, 13, new_id])
        checksum = self._calculate_checksum(current_id, 4, 13, [new_id])
        packet.append(checksum)
        self.ser.write(packet)
        print(f"Servo {current_id} changed to ID {new_id}")

    def stop(self, servo_id):
        """Unloads the servo (turns off motor torque)"""
        packet = bytearray([0x55, 0x55, servo_id, 0x03, 20, 0, 232]) # Checksum pre-calc'd for ID 1
        self.ser.write(packet)

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

# --- Example Usage ---
if __name__ == "__main__":
    my_robot = BusServo(port='COM9') # Replace with your COM port
    
    # Smooth move: ID 1 to position 500 (middle) over 1 second
    my_robot.move(1, 500, 1000)
    time.sleep(1.5)
    
    # Fast move: ID 1 to position 200 over 0.2 seconds
    # my_robot.move(1, 200, 200)
    
    my_robot.close()