import serial.tools.list_ports
from hmccontroller import HmcControlCs, App
import time

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return []
    print("Available COM Ports:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
    return ports

def main():
    ports = list_ports()
    if not ports:
        return

    port_index = int(input("Select COM port number for the axis to calibrate: "))
    axis_name = input("Enter axis name (x/y/z): ").strip().lower()

    hmc = HmcControlCs(axis=axis_name)
    if not hmc.config_serial_port(ports[port_index].device):
        print("Failed to connect to the axis.")
        return

    # Disable ramps if requested to have precise step timing
    disable_ramps = input("Disable acceleration/deceleration ramps for calibration? (y/n): ").strip().lower() == 'y'
    if disable_ramps:
        try:
            hmc.acceleration_and_deceleration(False, False)
            print("Acceleration/deceleration ramps disabled.")
        except Exception as e:
            print(f"Could not disable ramps: {e}")

    app = App()
    if axis_name == 'x':
        app.x_hmc = hmc
    elif axis_name == 'y':
        app.y_hmc = hmc
    else:
        app.z_hmc = hmc
    app.hmcControl = hmc

    # We will command a 20 µm move (100 steps) by default
    move_distance_um = float(input("Enter move distance in microns (default 20.0): ") or 20.0)
    resolution = 0.2
    steps = int(move_distance_um / resolution)

    # Fine sweep speeds to test (in steps/sec)
    speeds_to_test = list(range(10, 251))
    
    do_offset = input("Move axis by +10,000 µm offset? (y/n) [default: y]: ").strip().lower() != 'n'
    if do_offset:
        print("Moving axis to offset of 10,000 µm (safe starting position)...")
        # Move forward to 10,000 microns
        if axis_name == 'x':
            hmc.set_speed(1000, 0, 0)
            app.x_value = 10000.0
            app.y_value = 0
            app.z_value = 0
        elif axis_name == 'y':
            hmc.set_speed(0, 1000, 0)
            app.x_value = 0
            app.y_value = 10000.0
            app.z_value = 0
        else:
            hmc.set_speed(0, 0, 1000)
            app.x_value = 0
            app.y_value = 0
            app.z_value = 10000.0

        app.command = '1'
        app.start_thread()
        if app.hmcControl.run_thread:
            app.hmcControl.run_thread.join()
        print("Offset move completed.")
    else:
        print("Skipping offset move. Assuming axis is already positioned at 10,000 µm.")

    # We first estimate the serial communication overhead by running at a known safe speed
    # Speed of 50 steps/s is known to be safe.
    print("Estimating serial communication overhead...")
    safe_speed = 50.0
    safe_speed_um_s = safe_speed * resolution
    if axis_name == 'x':
        hmc.set_speed(safe_speed_um_s, 0, 0)
    elif axis_name == 'y':
        hmc.set_speed(0, safe_speed_um_s, 0)
    else:
        hmc.set_speed(0, 0, safe_speed_um_s)
        
    app.command = '1'
    if axis_name == 'x':
        app.x_value = move_distance_um
        app.y_value = 0
        app.z_value = 0
    elif axis_name == 'y':
        app.x_value = 0
        app.y_value = move_distance_um
        app.z_value = 0
    else:
        app.x_value = 0
        app.y_value = 0
        app.z_value = move_distance_um

    start_time = time.perf_counter()
    app.start_thread()
    if app.hmcControl.run_thread:
        app.hmcControl.run_thread.join()
    actual_safe_duration = time.perf_counter() - start_time
    
    # Return to offset
    if axis_name == 'x':
        hmc.set_speed(1000, 0, 0)
        app.x_value = -move_distance_um
        app.y_value = 0
        app.z_value = 0
    elif axis_name == 'y':
        hmc.set_speed(0, 1000, 0)
        app.x_value = 0
        app.y_value = -move_distance_um
        app.z_value = 0
    else:
        hmc.set_speed(0, 0, 1000)
        app.x_value = 0
        app.y_value = 0
        app.z_value = -move_distance_um
    app.command = '1'
    app.start_thread()
    if app.hmcControl.run_thread:
        app.hmcControl.run_thread.join()
        
    expected_safe_motion = steps / safe_speed
    measured_overhead = max(0.0, actual_safe_duration - expected_safe_motion)
    print(f"Measured serial overhead: {measured_overhead:.3f} s")

    log_filename = "le.txt"
    print(f"Results will be written to {log_filename}")
    
    with open(log_filename, "w", encoding="utf-8") as log_file:
        header = f"Commanded Speed (steps/s) | Expected Duration (s) | Actual Duration (s) | Effective Speed (steps/s) | Overridden?\n"
        header += "-" * 110 + "\n"
        log_file.write(header)
        print(header, end="")

        for step_speed in speeds_to_test:
            speed_um_s = step_speed * resolution
            expected_motion_duration = steps / step_speed
            expected_total_duration = expected_motion_duration + measured_overhead
            
            # Set speed
            if axis_name == 'x':
                hmc.set_speed(speed_um_s, 0, 0)
            elif axis_name == 'y':
                hmc.set_speed(0, speed_um_s, 0)
            else:
                hmc.set_speed(0, 0, speed_um_s)
            
            # Command the forward move
            app.command = '1'
            if axis_name == 'x':
                app.x_value = move_distance_um
                app.y_value = 0
                app.z_value = 0
            elif axis_name == 'y':
                app.x_value = 0
                app.y_value = move_distance_um
                app.z_value = 0
            else:
                app.x_value = 0
                app.y_value = 0
                app.z_value = move_distance_um

            start_time = time.perf_counter()
            app.start_thread()
            if app.hmcControl.run_thread:
                app.hmcControl.run_thread.join()
            actual_duration = time.perf_counter() - start_time

            effective_speed_steps = steps / (actual_duration - measured_overhead) if actual_duration > measured_overhead else 9999.9
            
            # If the actual duration is significantly faster than expected (by more than 0.08s), classify as YES
            time_diff = expected_total_duration - actual_duration
            overridden = "YES" if time_diff > 0.08 else "NO"

            log_line = f"{step_speed:25d} | {expected_total_duration:21.3f} | {actual_duration:19.3f} | {effective_speed_steps:25.2f} | {overridden}\n"
            log_line_file = f"{step_speed} steps/s ({speed_um_s:.1f} µm/s) | Expected: {expected_total_duration:.3f} s | Actual: {actual_duration:.3f} s | Effective: {effective_speed_steps:.2f} steps/s | Overridden: {overridden}\n"
            log_file.write(log_line_file)
            log_file.flush()
            print(log_line, end="")

            # Return to offset position at standard speed (1000 um/s = 5000 steps/s)
            if axis_name == 'x':
                hmc.set_speed(1000, 0, 0)
                app.x_value = -move_distance_um
                app.y_value = 0
                app.z_value = 0
            elif axis_name == 'y':
                hmc.set_speed(0, 1000, 0)
                app.x_value = 0
                app.y_value = -move_distance_um
                app.z_value = 0
            else:
                hmc.set_speed(0, 0, 1000)
                app.x_value = 0
                app.y_value = 0
                app.z_value = -move_distance_um

            app.command = '1'
            app.start_thread()
            if app.hmcControl.run_thread:
                app.hmcControl.run_thread.join()
            
            time.sleep(0.1) # short delay between runs

    if disable_ramps:
        try:
            hmc.acceleration_and_deceleration(True, True)
            print("\nAcceleration/deceleration ramps restored.")
        except Exception:
            pass

    print("\nKeeping axis at 10,000 µm offset position to prevent home limit switch hits.")
    print("Fine-grained sweep finished.")

if __name__ == '__main__':
    main()
