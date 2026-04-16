import sys
import time
import math
import numpy as np
import matplotlib.pyplot as plt

from motorControl import BusServo
from motorInfo import ServoMonitor

PORT = '/dev/ttyUSB0'
# PORT = 'COM9'

HOME_POSITIONS = {
    1: 996,  # left hip
    2: 571,  # left knee
    3: 765,  # right hip
    4: 434   # right knee
}

# LX-16A style assumption: ~240 deg over 1000 position units
DEG_PER_UNIT = 240.0 / 1000.0

# Optional geometry for future cm/sec conversion
THIGH_LENGTH_CM = 2
SHIN_LENGTH_CM = 16

# -------------------------
# Gait tuning constants
# -------------------------
# You may need to flip signs depending on your robot's actual mounting direction.
LEFT_HIP_FORWARD_DELTA = 120
RIGHT_HIP_FORWARD_DELTA = 120

LEFT_HIP_BACK_DELTA = -60
RIGHT_HIP_BACK_DELTA = -60

LEFT_KNEE_LIFT_DELTA = 110
RIGHT_KNEE_LIFT_DELTA = 110

STEP_DURATION_MS = 120
SAMPLE_DT = 0.01
COMMAND_STAGGER = 0.005

def clamp_position(pos):
    return max(0, min(1000, int(pos)))

def servo_units_to_deg(pos_units, servo_id=None):
    return pos_units * DEG_PER_UNIT

def relative_deg_from_home(pos_units, servo_id):
    return (pos_units - HOME_POSITIONS[servo_id]) * DEG_PER_UNIT

def homePosition(my_robot, connected_ids, duration=1000, tolerance=15):
    print("Moving connected servos to home positions...")

    monitor = ServoMonitor(ser=my_robot.ser)

    # Command all connected servos to home
    for servo_id in connected_ids:
        if servo_id in HOME_POSITIONS:
            my_robot.move(servo_id, HOME_POSITIONS[servo_id], duration)
            time.sleep(500)

    time.sleep(duration / 1000 + 0.3)

    # Read back actual positions
    positions = read_servo_positions(monitor, connected_ids)

    all_reached = True
    for servo_id in connected_ids:
        if servo_id not in HOME_POSITIONS:
            continue

        actual = positions.get(servo_id)
        target = HOME_POSITIONS[servo_id]

        if actual is None:
            print(f"Servo {servo_id}: could not read position.")
            all_reached = False
        else:
            error = actual - target
            print(f"Servo {servo_id}: target={target}, actual={actual}, error={error}")

            if abs(error) > tolerance:
                print(f"Servo {servo_id} did NOT reach home within tolerance ±{tolerance}.")
                all_reached = False

    if all_reached:
        print("Home position reached.")
    else:
        print("Warning: one or more servos did not fully reach home.")

    return all_reached

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

        home_ok = homePosition(my_robot, connected_ids)

        if not home_ok:
            print("\nBoot-up warning: not all connected servos reached home.")
        else:
            print("\nAll connected servos confirmed at home.")

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

def get_connected_ids(my_robot):
    connected_ids = []
    for servo_id in range(1, 5):
        returned_id = my_robot.read_id(servo_id)
        if returned_id is not None:
            connected_ids.append(servo_id)
    return connected_ids

def read_servo_positions(monitor, servo_ids):
    positions = {}
    for servo_id in servo_ids:
        stats = monitor._get_single_servo_stats(servo_id)
        if "Position" in stats:
            positions[servo_id] = stats["Position"]
        else:
            positions[servo_id] = None
    return positions

def initialize_log(servo_ids):
    log = {}
    for sid in servo_ids:
        log[sid] = {
            "t": [],
            "pos_units": [],
            "angle_deg_abs": [],
            "angle_deg_rel_home": []
        }
    return log

def sample_log(monitor, servo_ids, log, t0):
    positions = read_servo_positions(monitor, servo_ids)
    t = time.time() - t0

    for sid in servo_ids:
        pos = positions.get(sid, None)
        if pos is not None:
            log[sid]["t"].append(t)
            log[sid]["pos_units"].append(pos)
            log[sid]["angle_deg_abs"].append(servo_units_to_deg(pos, sid))
            log[sid]["angle_deg_rel_home"].append(relative_deg_from_home(pos, sid))

def move_pose_and_log(my_robot, monitor, pose_dict, servo_ids, log, t0, duration_ms=220, sample_dt=0.02):
    for sid, target in pose_dict.items():
        if sid in servo_ids:
            my_robot.move(sid, clamp_position(target), duration_ms)

    end_time = time.time() + duration_ms / 1000.0 + 0.01
    while time.time() < end_time:
        sample_log(monitor, servo_ids, log, t0)
        time.sleep(sample_dt)

def build_pose(left_hip_delta=0, left_knee_delta=0, right_hip_delta=0, right_knee_delta=0):
    return {
        1: HOME_POSITIONS[1] + left_hip_delta,
        2: HOME_POSITIONS[2] + left_knee_delta,
        3: HOME_POSITIONS[3] + right_hip_delta,
        4: HOME_POSITIONS[4] + right_knee_delta
    }

def compute_speed(time_array, value_array):
    if len(time_array) < 2:
        return np.zeros_like(value_array)
    return np.gradient(value_array, time_array)

def estimate_leg_endpoint_speed_cm_s(log, hip_id, knee_id, thigh_cm, shin_cm):
    t = np.array(log[hip_id]["t"])
    hip_deg = np.array(log[hip_id]["angle_deg_rel_home"])
    knee_deg = np.array(log[knee_id]["angle_deg_rel_home"])

    n = min(len(t), len(hip_deg), len(knee_deg))
    t = t[:n]
    hip_deg = hip_deg[:n]
    knee_deg = knee_deg[:n]

    hip_rad = np.deg2rad(hip_deg)
    knee_rad = np.deg2rad(knee_deg)

    x = thigh_cm * np.sin(hip_rad) + shin_cm * np.sin(hip_rad + knee_rad)
    z = -thigh_cm * np.cos(hip_rad) - shin_cm * np.cos(hip_rad + knee_rad)

    if len(t) < 2:
        return t, np.zeros_like(t)

    dx = np.gradient(x, t)
    dz = np.gradient(z, t)
    speed = np.sqrt(dx**2 + dz**2)

    return t, speed

def plot_walking_results(log):
    plt.figure(figsize=(11, 6))
    for sid in sorted(log.keys()):
        t = np.array(log[sid]["t"])
        ang = np.array(log[sid]["angle_deg_rel_home"])
        if len(t) > 0:
            plt.plot(t, ang, label=f"Motor {sid}")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint angle relative to home (deg)")
    plt.title("Motor Angle vs Time During Walking")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("walking_angles.png", dpi=200)

    plt.figure(figsize=(11, 6))

    if THIGH_LENGTH_CM is not None and SHIN_LENGTH_CM is not None:
        tL, vL = estimate_leg_endpoint_speed_cm_s(log, 1, 2, THIGH_LENGTH_CM, SHIN_LENGTH_CM)
        tR, vR = estimate_leg_endpoint_speed_cm_s(log, 3, 4, THIGH_LENGTH_CM, SHIN_LENGTH_CM)

        if len(tL) > 0:
            plt.plot(tL, vL, label="Left leg foot speed")
        if len(tR) > 0:
            plt.plot(tR, vR, label="Right leg foot speed")

        plt.ylabel("Estimated foot speed (cm/s)")
        plt.title("Estimated Leg Endpoint Speed vs Time")
    else:
        for sid in sorted(log.keys()):
            t = np.array(log[sid]["t"])
            ang = np.array(log[sid]["angle_deg_rel_home"])
            if len(t) > 1:
                speed = compute_speed(t, ang)
                plt.plot(t, speed, label=f"Motor {sid}")

        plt.ylabel("Joint angular speed (deg/s)")
        plt.title("Motor Angular Speed vs Time")

        print("\nNote: second plot is angular speed (deg/s), not cm/s.")
        print("To get cm/s, set THIGH_LENGTH_CM and SHIN_LENGTH_CM near the top of the file.")

    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("walking_speeds.png", dpi=200)

    print("\nSaved plots:")
    print("  walking_angles.png")
    print("  walking_speeds.png")

    plt.show()

def walking(my_robot, num_steps=4, duration_ms=STEP_DURATION_MS, sample_dt=SAMPLE_DT):
    print("Starting walking routine...")

    connected_ids = get_connected_ids(my_robot)
    if not connected_ids:
        print("No servos detected.")
        return

    required_ids = [1, 2, 3, 4]
    missing = [sid for sid in required_ids if sid not in connected_ids]
    if missing:
        print(f"Walking requires servos 1-4. Missing: {missing}")
        return

    print(f"Required IDs: {required_ids}")
    print(f"Connected IDs: {connected_ids}")

    print(f"\nConnected servos: {connected_ids}")

    monitor = ServoMonitor(ser=my_robot.ser)

    print("\nReading servo info...")
    monitor.get_stats(connected_ids)

    print("Enabling torque...")
    for sid in required_ids:
        my_robot.torque_on(sid)
        time.sleep(0.01)

    print("Moving to home position before walking...")
    homePosition(my_robot, required_ids)

    log = initialize_log(required_ids)
    t0 = time.time()

    for _ in range(3):
        sample_log(monitor, required_ids, log, t0)
        time.sleep(sample_dt)

    # Neutral/load-transfer poses
    shift_left_pose = build_pose(
        left_hip_delta=-20,
        left_knee_delta=20,
        right_hip_delta=-20,
        right_knee_delta=0
    )

    shift_right_pose = build_pose(
        left_hip_delta=-20,
        left_knee_delta=0,
        right_hip_delta=-20,
        right_knee_delta=20
    )

    # Left leg swings forward, right supports
    left_swing_pose = build_pose(
        left_hip_delta=LEFT_HIP_FORWARD_DELTA,
        left_knee_delta=LEFT_KNEE_LIFT_DELTA,
        right_hip_delta=RIGHT_HIP_BACK_DELTA,
        right_knee_delta=0
    )

    # Right leg swings forward, left supports
    right_swing_pose = build_pose(
        left_hip_delta=LEFT_HIP_BACK_DELTA,
        left_knee_delta=0,
        right_hip_delta=RIGHT_HIP_FORWARD_DELTA,
        right_knee_delta=RIGHT_KNEE_LIFT_DELTA
    )

    settle_pose = build_pose(
        left_hip_delta=0,
        left_knee_delta=0,
        right_hip_delta=0,
        right_knee_delta=0
    )

    for step_idx in range(num_steps):
        print(f"Walking cycle {step_idx + 1}/{num_steps}")

        # shift/load right leg so left can swing
        move_pose_and_log(
            my_robot, monitor, shift_left_pose, required_ids, log, t0,
            duration_ms=90, sample_dt=sample_dt
        )

        # left swing
        move_pose_and_log(
            my_robot, monitor, left_swing_pose, required_ids, log, t0,
            duration_ms=duration_ms, sample_dt=sample_dt
        )

        # brief settle
        move_pose_and_log(
            my_robot, monitor, settle_pose, required_ids, log, t0,
            duration_ms=60, sample_dt=sample_dt
        )

        # shift/load left leg so right can swing
        move_pose_and_log(
            my_robot, monitor, shift_right_pose, required_ids, log, t0,
            duration_ms=90, sample_dt=sample_dt
        )

        # right swing
        move_pose_and_log(
            my_robot, monitor, right_swing_pose, required_ids, log, t0,
            duration_ms=duration_ms, sample_dt=sample_dt
        )

        # brief settle
        move_pose_and_log(
            my_robot, monitor, settle_pose, required_ids, log, t0,
            duration_ms=60, sample_dt=sample_dt
        )

    print("Walking routine complete. Returning to home position...")
    homePosition(my_robot, required_ids)

    for _ in range(3):
        sample_log(monitor, required_ids, log, t0)
        time.sleep(sample_dt)

    plot_walking_results(log)

if __name__ == "__main__":
    my_robot = BusServo(port=PORT)

    try:
        command = sys.argv[1].lower() if len(sys.argv) > 1 else "full"

        if command == "bootup":
            bootUp(my_robot)

        elif command == "shutdown":
            shutdown(my_robot)

        elif command == "home":
            connected = get_connected_ids(my_robot)
            if connected:
                homePosition(my_robot, connected)
            else:
                print("No servos detected.")

        elif command == "walking":
            num_steps = int(sys.argv[2]) if len(sys.argv) > 2 else 4
            walking(my_robot, num_steps=num_steps)

        elif command == "full":
            bootUp(my_robot)
            shutdown(my_robot)

        else:
            print("Unknown command.")
            print("Usage: python3 intialTest.py [bootup|shutdown|home|walking|full] [optional_step_count]")

    finally:
        my_robot.close()