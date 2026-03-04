import serial
import time

class ServoMonitor:
    def __init__(self, port='COM3', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.5)

    def _send_request(self, servo_id, cmd, length=3, params=None):
        if params is None: params = []
        packet = [0x55, 0x55, servo_id, length, cmd] + params
        checksum = (~(servo_id + length + cmd + sum(params))) & 0xFF
        packet.append(checksum)
        self.ser.write(bytearray(packet))

    def _read_response(self, expected_cmd, expected_len):
        """Parses the incoming serial buffer for the servo's answer"""
        data = self.ser.read(10) 
        if len(data) < 6: return None
        
        for i in range(len(data) - 1):
            if data[i] == 0x55 and data[i+1] == 0x55:
                res_id = data[i+2]
                res_len = data[i+3]
                res_cmd = data[i+4]
                
                if res_cmd == expected_cmd:
                    params = data[i+5 : i+5 + (res_len - 3)]
                    return list(params)
        return None

    def get_stats(self, servo_id):
        stats = {"ID": servo_id}
        
        self._send_request(servo_id, 28)
        pos_data = self._read_response(28, 5)
        if pos_data:
            stats["Position"] = pos_data[0] + (pos_data[1] << 8)
        
        self._send_request(servo_id, 26)
        temp_data = self._read_response(26, 4)
        if temp_data:
            stats["Temperature"] = f"{temp_data[0]}°C"
        self._send_request(servo_id, 27)
        volt_data = self._read_response(27, 5)
        if volt_data:
            mv = volt_data[0] + (volt_data[1] << 8)
            stats["Voltage"] = f"{mv/1000:.2f}V"

        return stats

if __name__ == "__main__":
    monitor = ServoMonitor('COM9')
    print("--- Starting Servo Health Monitor ---")
    
    try:
        while True:
            data = monitor.get_stats(1)
            if "Position" in data:
                print(f"ID: {data['ID']} | Pos: {data['Position']} | Temp: {data['Temperature']} | Volts: {data['Voltage']}")
            else:
                print("Servo not responding... check power/wiring.")
            
            time.sleep(0.5) 
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
        monitor.ser.close()