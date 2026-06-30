# modes/mode_trajectory.py
# ponytail: path trajectory command from file (Mode 3).

import time
import main

def run(app, x_hmc, y_hmc, z_hmc, reset_all_serial_fn, set_all_speed_fn):
    try:
        v = int(input("Enter velocity microns/sec:"))
    except ValueError:
        print("Invalid velocity.")
        return
    reset_all_serial_fn()
    filename = input("Enter txt filename:")
    try:
        with open(f"{filename}.txt", "r") as file:
            i = 1
            for line in file:
                parts = line.strip().replace(',', ' ').split()
                if len(parts) == 3:
                    x, y, z = map(float, parts)
                    dx = x - x_hmc.current_x
                    dy = y - y_hmc.current_y
                    dz = z - z_hmc.current_z
                    
                    actual_dx = round(dx / 0.2) * 0.2
                    actual_dy = round(dy / 0.2) * 0.2
                    actual_dz = round(dz / 0.2) * 0.2
                    
                    if actual_dx == 0 and actual_dy == 0 and actual_dz == 0:
                        i += 1
                        continue

                    vx, vy = main.compute_synchronized_speeds(actual_dx, actual_dy, v)
                    set_all_speed_fn(vx, vy, v)
                    
                    try:
                        app.command = '1'
                        app.x_value = actual_dx
                        app.y_value = actual_dy
                        app.z_value = actual_dz

                        app.start_thread()

                        while app.hmcControl.run_thread.is_alive():
                            time.sleep(0.2)

                        if app.hmcControl.run_thread:
                            app.hmcControl.run_thread.join()

                        print("Final Position:")
                        print(f"X: {x_hmc.x_current_position} µm")
                        print(f"Y: {y_hmc.y_current_position} µm")
                        print(f"Z: {z_hmc.z_current_position} µm")
                        print(f"{i} MOVE COMPLETED")

                    except Exception as e:
                        print(f"[ERROR] Move {i} failed: {e}")
                        reset_all_serial_fn()
                        for h in [x_hmc, y_hmc, z_hmc]:
                            h.ser.reset_input_buffer()

                    i += 1
    except FileNotFoundError:
        print(f"Error: file {filename}.txt not found.")
