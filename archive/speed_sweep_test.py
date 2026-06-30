import serial.tools.list_ports
from hmccontroller import HmcControlCs, App
import time
from threading import Thread

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

    # We will command a 100 µm move (500 steps) to keep individual moves short
    move_distance_um = 100.0
    resolution = 0.2
    steps = int(move_distance_um / resolution) # 500 steps

    # Coarse sweep speeds to test (in steps/sec)
    speeds_to_test = [
        10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 
        110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 
        210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 
        320, 340, 360, 380, 400, 450, 500
    ]
    
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

    print("\nStarting Coarse Speed Sweep Test...")
    print(f"Move size: {move_distance_um} µm ({steps} steps)")
    print("-" * 100)

    for step_speed in speeds_to_test:
        speed_um_s = step_speed * resolution
        expected_duration = steps / step_speed
        
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

        # Print testing progress immediately
        print(f"Testing {step_speed:4d} steps/s ({speed_um_s:5.1f} µm/s) - Expected: {expected_duration:6.2f} s... ", end='', flush=True)

        start_time = time.perf_counter()
        app.start_thread()
        if app.hmcControl.run_thread:
            app.hmcControl.run_thread.join()
        actual_duration = time.perf_counter() - start_time

        effective_speed_steps = steps / actual_duration
        
        # Determine if it was overridden (if actual duration is significantly faster than expected, e.g. > 15% faster)
        overridden = "YES" if (expected_duration - actual_duration) / expected_duration > 0.15 else "NO"

        print(f"Actual: {actual_duration:6.2f} s | Effective: {effective_speed_steps:7.2f} steps/s | Overridden: {overridden}")

        # Return to origin at a fast, standard speed (1000 um/s = 5000 steps/s)
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
        
        time.sleep(0.3) # delay between runs

    if disable_ramps:
        try:
            hmc.acceleration_and_deceleration(True, True)
            print("\nAcceleration/deceleration ramps restored.")
        except Exception:
            pass

    print("Returning axis to home position (0 µm)...")
    if axis_name == 'x':
        hmc.set_speed(1000, 0, 0)
        app.x_value = -10000.0
        app.y_value = 0
        app.z_value = 0
    elif axis_name == 'y':
        hmc.set_speed(0, 1000, 0)
        app.x_value = 0
        app.y_value = -10000.0
        app.z_value = 0
    else:
        hmc.set_speed(0, 0, 1000)
        app.x_value = 0
        app.y_value = 0
        app.z_value = -10000.0

    app.command = '1'
    app.start_thread()
    if app.hmcControl.run_thread:
        app.hmcControl.run_thread.join()
    print("Returned to home.")

    print("\nSweep finished.")

if __name__ == '__main__':
    main()
