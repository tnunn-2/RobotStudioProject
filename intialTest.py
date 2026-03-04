import serial
import time
from motorControl import BusServo
from motorInfo import ServoMonitor
import threading

def stepRight(my_robot):
    my_robot.move(2, 300, 1000)
    time.sleep(1.5)
    my_robot.move(2, 500, 1000)
    time.sleep(1.5)
    my_robot.move(2, 700, 1000)
    time.sleep(1.5)
    my_robot.move(2, 500, 1000)
    time.sleep(1.5)
    my_robot.move(2, 300, 1000)
    time.sleep(1.5)
    my_robot.move(2, 500, 1000)
    time.sleep(1.5)


def stepHip(my_robot):
    my_robot.move(1, 300, 1000)
    time.sleep(1.5)
    my_robot.move(1, 500, 1000)
    time.sleep(1.5)
    my_robot.move(1, 700, 1000)
    time.sleep(1.5)
    my_robot.move(1, 500, 1000)
    time.sleep(1.5)
    my_robot.move(1, 300, 1000)
    time.sleep(1.5)
    my_robot.move(1, 500, 1000)
    time.sleep(1.5)

def homePosition(my_robot):
    my_robot.move(1, 500, 1000)
    time.sleep(1.5)
    my_robot.move(2, 500, 1000)
    time.sleep(1.5)

if __name__ == "__main__":
    my_robot = BusServo(port='COM9') # Replace with your COM port

    monitor_thread = threading.Thread(
        target=my_robot.monitorDisconnection,
        args=(my_robot, 1),
        daemon=True
    )
    monitor_thread.start()

    # my_robot.read_id(1)
    # my_robot.torque_off(1)
    # Smooth move: ID 1 to position 500 (middle) over 1 second
    # my_robot.move(2, 500, 1000)
    # time.sleep(1.5)
    stepRight(my_robot)
    stepHip(my_robot)
    homePosition(my_robot)



    # Fast move: ID 1 to position 200 over 0.2 seconds
    # my_robot.move(1, 200, 200)
    
    my_robot.close()