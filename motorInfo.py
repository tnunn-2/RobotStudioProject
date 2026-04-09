import serial
import time

PORT = '/dev/ttyUSB0'
# PORT = 'COM9'

BAUDRATE = 115200

class ServoMonitor:
    def __init__(self, ser=None, port=PORT, baudrate=BAUDRATE):
        if ser is not None:
            self.ser = ser
            self.owns_serial = False
        else:
            self.ser = serial.Serial(port, baudrate, timeout=0.5)
            self.owns_serial = True

    def _send_request(self, servo_id, cmd, length=3, params=None):
        if params is None:
            params = []
        packet = [0x55, 0x55, servo_id, length, cmd] + params
        checksum = (~(servo_id + length + cmd + sum(params))) & 0xFF
        packet.append(checksum)

        self.ser.reset_input_buffer()
        self.ser.write(bytearray(packet))

    def _read_response(self, expected_cmd, expected_len=None):
        data = self.ser.read(10)
        if len(data) < 6:
            return None

        for i in range(len(data) - 1):
            if data[i] == 0x55 and data[i + 1] == 0x55:
                res_len = data[i + 3]
                res_cmd = data[i + 4]

                if res_cmd == expected_cmd:
                    params = data[i + 5 : i + 5 + (res_len - 3)]
                    return list(params)
        return None

    def _set_torque(self, servo_id, enable):
        """
        LX-16A style:
        cmd 31, param 0 = torque off / unload
        cmd 31, param 1 = torque on  / load
        """
        value = 1 if enable else 0
        self._send_request(servo_id, 31, length=4, params=[value])
        time.sleep(0.03)

    def torque_off(self, servo_id):
        self._set_torque(servo_id, False)

    def torque_on(self, servo_id):
        self._set_torque(servo_id, True)

    def _read_torque_state(self, servo_id):
        """
        Attempts to read current load/unload state.
        Returns:
            1 -> torque on
            0 -> torque off
            None -> unknown / no response
        """
        self._send_request(servo_id, 32)
        data = self._read_response(32)
        if data and len(data) >= 1:
            return data[0]
        return None

    def _get_single_servo_stats(self, servo_id):
        stats = {"ID": servo_id}

        self._send_request(servo_id, 28)
        pos_data = self._read_response(28)
        if pos_data and len(pos_data) >= 2:
            stats["Position"] = pos_data[0] + (pos_data[1] << 8)

        self._send_request(servo_id, 26)
        temp_data = self._read_response(26)
        if temp_data and len(temp_data) >= 1:
            stats["Temperature"] = f"{temp_data[0]}°C"

        self._send_request(servo_id, 27)
        volt_data = self._read_response(27)
        if volt_data and len(volt_data) >= 2:
            mv = volt_data[0] + (volt_data[1] << 8)
            stats["Voltage"] = f"{mv/1000:.2f}V"

        torque_state = self._read_torque_state(servo_id)
        if torque_state is not None:
            stats["Torque"] = "ON" if torque_state == 1 else "OFF"

        return stats

    def get_stats(self, servo_ids=None, print_stats=True, torque_off_while_reading=True, restore_torque=True):
        """
        For each connected servo:
        - optionally turn torque off
        - read and print stats
        - optionally restore prior torque state

        This makes it easier to physically move the legs and observe position values.
        """
        if servo_ids is None:
            servo_ids = range(1, 5)

        if isinstance(servo_ids, int):
            servo_ids = [servo_ids]

        all_stats = []

        for servo_id in servo_ids:
            original_torque = None

            # Check whether servo responds at all
            original_torque = self._read_torque_state(servo_id)
            if original_torque is None:
                time.sleep(0.05)
                continue

            try:
                if torque_off_while_reading:
                    self.torque_off(servo_id)
                    time.sleep(0.05)

                stats = self._get_single_servo_stats(servo_id)

                if "Position" in stats:
                    all_stats.append(stats)

                    if print_stats:
                        print(
                            f"Servo {stats['ID']} | "
                            f"Position: {stats.get('Position', 'Unknown')} | "
                            f"Temp: {stats.get('Temperature', 'Unknown')} | "
                            f"Voltage: {stats.get('Voltage', 'Unknown')} | "
                            f"Torque: {stats.get('Torque', 'Unknown')}"
                        )

            finally:
                if restore_torque and original_torque is not None:
                    if original_torque == 1:
                        self.torque_on(servo_id)
                    else:
                        self.torque_off(servo_id)

            time.sleep(0.05)

        if print_stats and not all_stats:
            print("No connected servos responded.")

        return all_stats

    def close(self):
        if getattr(self, "owns_serial", False):
            self.ser.close()


if __name__ == "__main__":
    monitor = ServoMonitor()
    print("--- Starting Servo Health Monitor ---")
    print("Torque will be turned OFF while each servo is being read so you can move it by hand.\n")

    try:
        while True:
            monitor.get_stats(
                servo_ids=range(1, 5),
                print_stats=True,
                torque_off_while_reading=True,
                restore_torque=False
            )
            print("-" * 70)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
        monitor.close()