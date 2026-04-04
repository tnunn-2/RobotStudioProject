import time
from motorControl import BusServo
from motorInfo import ServoMonitor

HOME_POSITIONS = {
    1: 982,
    2: 546,
    3: 739,
    4: 430
}

def homePosition(my_robot, connected_ids, duration=1000):
    print("Moving connected servos to home positions...")

    for servo_id in connected_ids:
        if servo_id in HOME_POSITIONS:
            my_robot.move(servo_id, HOME_POSITIONS[servo_id], duration)
            time.sleep(0.1)

    time.sleep(duration / 1000 + 0.5)
    print("Home position reached.")

def bootUp(my_robot):
    print("Booting up robot...")

    connected_ids = []
    monitor = ServoMonitor(ser=my_robot.ser)

    try:
        print("\nScanning for connected servos (IDs 1-4)...")
        for servo_id in range(1, 5):
            returned_id = my_robot.read_id(servo_id)
            if returned_id is not None:
                connected_ids.append(servo_id)

        if not connected_ids:
            print("No servos detected.")
            return []

        print(f"\nConnected servos: {connected_ids}")

        print("\nReading servo info...")
        monitor.get_stats(connected_ids)

        print("\nEnabling torque on connected servos...")
        for servo_id in connected_ids:
            my_robot.torque_on(servo_id)
            time.sleep(0.05)

        print("\nMoving to home position...")
        homePosition(my_robot, connected_ids)

        print("\nBoot-up complete.")
        return connected_ids

    except Exception as e:
        print(f"Boot-up error: {e}")
        return []

def shutdown(my_robot):
    print("Shutting down robot...")

    connected_ids = []
    monitor = ServoMonitor(ser=my_robot.ser)

    try:
        print("\nScanning for connected servos (IDs 1-4)...")
        for servo_id in range(1, 5):
            returned_id = my_robot.read_id(servo_id)
            if returned_id is not None:
                connected_ids.append(servo_id)

        if not connected_ids:
            print("No servos detected. Nothing to shut down.")
            return []

        print(f"\nConnected servos: {connected_ids}")

        print("\nReading servo info before shutdown...")
        monitor.get_stats(connected_ids)

        print("\nDisabling torque on connected servos...")
        for servo_id in connected_ids:
            my_robot.torque_off(servo_id)
            time.sleep(0.05)

        print("\nShutdown complete.")
        return connected_ids

    except Exception as e:
        print(f"Shutdown error: {e}")
        return []

if __name__ == "__main__":
    PORT = 'COM9'

    my_robot = BusServo(port=PORT)

    connected = bootUp(my_robot)

    # Example actions here
    # stepRight(my_robot)
    # stepHip(my_robot)

    shutdown(my_robot)
    my_robot.close()