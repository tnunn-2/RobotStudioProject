import serial
import time

class ServoMonitor:
    def __init__(self, ser=None, port='COM3', baudrate=115200):
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

    def _read_response(self, expected_cmd, expected_len):
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

    def _get_single_servo_stats(self, servo_id):
        stats = {"ID": servo_id}

        self._send_request(servo_id, 28)
        pos_data = self._read_response(28, 5)
        if pos_data and len(pos_data) >= 2:
            stats["Position"] = pos_data[0] + (pos_data[1] << 8)

        self._send_request(servo_id, 26)
        temp_data = self._read_response(26, 4)
        if temp_data and len(temp_data) >= 1:
            stats["Temperature"] = f"{temp_data[0]}°C"

        self._send_request(servo_id, 27)
        volt_data = self._read_response(27, 5)
        if volt_data and len(volt_data) >= 2:
            mv = volt_data[0] + (volt_data[1] << 8)
            stats["Voltage"] = f"{mv/1000:.2f}V"

        return stats

    def get_stats(self, servo_ids=None, print_stats=True):
        if servo_ids is None:
            servo_ids = range(1, 5)

        if isinstance(servo_ids, int):
            servo_ids = [servo_ids]

        all_stats = []

        for servo_id in servo_ids:
            stats = self._get_single_servo_stats(servo_id)

            if "Position" in stats:
                all_stats.append(stats)

                if print_stats:
                    print(
                        f"Servo {stats['ID']} | "
                        f"Position: {stats.get('Position', 'Unknown')} | "
                        f"Temp: {stats.get('Temperature', 'Unknown')} | "
                        f"Voltage: {stats.get('Voltage', 'Unknown')}"
                    )

            time.sleep(0.05)

        if print_stats and not all_stats:
            print("No connected servos responded.")

        return all_stats

    def close(self):
        if getattr(self, "owns_serial", False):
            self.ser.close()


if __name__ == "__main__":
    monitor = ServoMonitor(port='COM9')
    print("--- Starting Servo Health Monitor ---")

    try:
        while True:
            monitor.get_stats()
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
        monitor.close()