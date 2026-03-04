import serial

# Connect to the Debug Board
ser = serial.Serial('COM9', 115200)

# The "Move to Position 500" Hex Packet for Hiwonder/LewanSoul
# Frame Header (0x55 0x55), ID (0x01), Length (0x07), Command (0x01), 
# Position (0xF4 0x01 = 500), Time (0xF4 0x01 = 500ms), Checksum
packet = bytearray([0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0xF4, 0x01, 0x0B])

ser.write(packet)
ser.close()