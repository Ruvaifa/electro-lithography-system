# modes/mode_manual.py
# ponytail: clean and minimal manual mode function.

import time

def run(app, x_hmc, y_hmc, z_hmc, reset_all_serial_fn):
    reset_all_serial_fn()
    while True:
        try:
            x = float(input("Enter X movement (microns): "))
            y = float(input("Enter Y movement (microns): "))
            z = float(input("Enter Z movement (microns): "))
        except (KeyboardInterrupt, EOFError):
            print("\nExiting manual move mode.")
            break
        except ValueError:
            print("Invalid input. Please enter numeric values.")
            continue

        app.command = '1'
        app.x_value = x
        app.y_value = y
        app.z_value = z
        app.start_thread()
        while app.hmcControl.run_thread.is_alive():
            print("Waiting for move to complete...")
            time.sleep(0.2)
        if app.hmcControl.run_thread:
            app.hmcControl.run_thread.join()
        print("Final Position:")
        print(f"X: {x_hmc.x_current_position} µm")
        print(f"Y: {y_hmc.y_current_position} µm")
        print(f"Z: {z_hmc.current_z} µm")
