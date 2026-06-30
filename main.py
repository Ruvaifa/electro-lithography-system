# main.py
# dispatcher script for electro-lithography system modes.

import serial.tools.list_ports
from hmccontroller import HmcControlCs, App
import time
import threading
from threading import Thread
from collections import defaultdict
import config

# Import mode modules
from modes import mode_manual
from modes import mode_home
from modes import mode_trajectory
from modes import mode_probe
from modes import mode_sync_litho
from modes import mode_step_test
from modes import mode_async_litho

def format_elapsed_time(seconds):
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours >= 1:
        return f"{int(hours)}h {int(minutes)}m {seconds:.2f}s"
    if minutes >= 1:
        return f"{int(minutes)}m {seconds:.2f}s"
    return f"{seconds:.2f}s"


def compute_synchronized_speeds(dx, dy, v, resolution=config.RESOLUTION_UM_PER_STEP, max_speed_steps=config.MAX_SPEED_STEPS):
    """Compute per-axis speeds so both X and Y finish each segment in exactly the same time."""
    steps_x = round(dx / resolution)
    steps_y = round(dy / resolution)
    abs_steps_x = abs(steps_x)
    abs_steps_y = abs(steps_y)

    if abs_steps_x == 0 and abs_steps_y == 0:
        return v, v

    if v <= 0:
        return 0.0, 0.0

    dominant_steps = max(abs_steps_x, abs_steps_y)
    segment_time = dominant_steps * resolution / v

    speed_x = abs_steps_x / segment_time if abs_steps_x > 0 else 0.0
    speed_y = abs_steps_y / segment_time if abs_steps_y > 0 else 0.0

    max_current_speed = max(speed_x, speed_y)
    if max_current_speed > max_speed_steps:
        scale = max_speed_steps / max_current_speed
        speed_x *= scale
        speed_y *= scale

    def is_bad_speed(spd):
        spd_int = round(spd)
        for low, high in config.BAD_SPEED_BANDS:
            if low <= spd_int <= high:
                return True
        return False

    if (speed_x > 0 and is_bad_speed(speed_x)) or (speed_y > 0 and is_bad_speed(speed_y)):
        k = 1.0
        found = False
        for delta in range(1, 100):
            for k_candidate in [1.0 + delta * 0.01, 1.0 - delta * 0.01]:
                if k_candidate <= 0.05:
                    continue
                sx = speed_x * k_candidate
                sy = speed_y * k_candidate
                if speed_x > 0 and is_bad_speed(sx):
                    continue
                if speed_y > 0 and is_bad_speed(sy):
                    continue
                k = k_candidate
                found = True
                break
            if found:
                break
        speed_x *= k
        speed_y *= k

    vx = speed_x * resolution
    vy = speed_y * resolution
    return vx, vy


def wait_for_motion(app):
    if app.hmcControl.run_thread:
        app.hmcControl.run_thread.join()


def print_timing_summary(timer_map, move_count, xy_move_count):
    print("[TIMER] Detailed timing summary:")
    for key in sorted(timer_map.keys()):
        print(f"[TIMER] {key}: {format_elapsed_time(timer_map[key])}")
    print(f"[TIMER] Moves executed: {move_count}")
    print(f"[TIMER] X/Y moves counted in movement timer: {xy_move_count}")


def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return []
    print("Available Serial Ports:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
    return ports


def step_size_listener(step_queue):
    while True:
        try:
            user_input = input("Enter new Z step size (µm): ").strip()
            if user_input:
                step = float(user_input)
                step_queue.put(step)
        except ValueError:
            print("[WARN] Invalid step size. Try again.")
        except (KeyboardInterrupt, EOFError):
            break


def main():
    ports = list_ports()
    if not ports:
        return

    x_index = int(input("Enter COM port number for X axis: "))
    y_index = int(input("Enter COM port number for Y axis: "))
    z_index = int(input("Enter COM port number for Z axis: "))

    x_hmc = HmcControlCs(axis='x')
    y_hmc = HmcControlCs(axis='y')
    z_hmc = HmcControlCs(axis='z')

    if not x_hmc.config_serial_port(ports[x_index].device):
        print("Failed to connect X axis.")
        return
    if not y_hmc.config_serial_port(ports[y_index].device):
        print("Failed to connect Y axis.")
        return
    if not z_hmc.config_serial_port(ports[z_index].device):
        print("Failed to connect Z axis.")
        return

    app = App()
    app.x_hmc = x_hmc
    app.y_hmc = y_hmc
    app.z_hmc = z_hmc
    app.hmcControl = z_hmc

    def set_all_speed(vx, vy, vz):
        x_hmc.set_speed(vx, 0, 0)
        y_hmc.set_speed(0, vy, 0)
        z_hmc.set_speed(0, 0, vz)

    def reset_all_serial():
        for h in [x_hmc, y_hmc, z_hmc]:
            h.ser.close()
        time.sleep(0.2)
        for h in [x_hmc, y_hmc, z_hmc]:
            h.ser.open()
            h.ser.reset_input_buffer()

    def startup_all():
        z_hmc.on_startup()
        x_hmc.on_startup()
        y_hmc.on_startup()

    startup_all()

    try:
        response = int(input("Enter 1 to move specific distance, 2 to send to home, 3 to import data from text file, 4 to send probe, 5 to pause/resume, 6 to run sync litho, 7 to run step test, 8 to run async litho: "))
    except ValueError:
        print("Invalid choice.")
        return

    if response == 1:
        mode_manual.run(app, x_hmc, y_hmc, z_hmc, reset_all_serial)
    elif response == 2:
        mode_home.run(startup_all)
    elif response == 3:
        mode_trajectory.run(app, x_hmc, y_hmc, z_hmc, reset_all_serial, set_all_speed)
    elif response == 4:
        mode_probe.run_probe(app, z_hmc)
    elif response == 5:
        mode_probe.run_pause_resume(app, z_hmc)
    elif response == 6:
        mode_sync_litho.run(app, x_hmc, y_hmc, z_hmc, reset_all_serial, set_all_speed, startup_all)
    elif response == 7:
        mode_step_test.run(app, x_hmc, y_hmc, z_hmc, set_all_speed)
    elif response == 8:
        mode_async_litho.run(app, x_hmc, y_hmc, z_hmc, reset_all_serial, set_all_speed, startup_all)


if __name__ == "__main__":
    main()
