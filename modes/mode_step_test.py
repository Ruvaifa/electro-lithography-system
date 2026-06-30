# modes/mode_step_test.py
# ponytail: step accuracy diagnostic test (Mode 7).

import time

def run(app, x_hmc, y_hmc, z_hmc, set_all_speed_fn):
    set_all_speed_fn(5000, 5000, 5000)  

    step_size = 0.2
    steps = 1000
    total_distance = step_size * steps

    print("[INFO] Moving X axis...")
    for i in range(steps):
        app.command = '1'
        app.x_value = step_size
        app.y_value = 0
        app.z_value = 0
        app.start_thread()
        while app.hmcControl.run_thread.is_alive():
            time.sleep(0.1)
        app.hmcControl.run_thread.join()
        print(f"{i} X movements completed")

    print("[INFO] Moving Y axis...")
    for i in range(steps):
        app.command = '1'
        app.x_value = 0
        app.y_value = step_size
        app.z_value = 0
        app.start_thread()
        while app.hmcControl.run_thread.is_alive():
            time.sleep(0.1)
        app.hmcControl.run_thread.join()
        print(f"{i} Y movements completed")

    print("[INFO] Moving Z axis...")
    for i in range(steps):
        app.command = '1'
        app.x_value = 0
        app.y_value = 0
        app.z_value = step_size
        app.start_thread()
        while app.hmcControl.run_thread.is_alive():
            time.sleep(0.1)
        app.hmcControl.run_thread.join()
        print(f"{i} Z movements completed")

    print("[INFO] Moving all X, Y, Z axes together...")
    for i in range(steps):
        app.command = '1'
        app.x_value = step_size
        app.y_value = step_size
        app.z_value = step_size
        app.start_thread()
        while app.hmcControl.run_thread.is_alive():
            time.sleep(0.1)
        app.hmcControl.run_thread.join()

    print(" XYZ step movements complete.")
