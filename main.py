import serial.tools.list_ports
from hmccontroller import HmcControlCs, App
import time
import threading
from threading import Thread
import math
import queue
from dataclasses import dataclass, field
from typing import Optional
import smumark1
import smumark2
from collections import defaultdict

# hmc = HmcControlCs()
# app = App()
# app.hmcControl=hmc

def format_elapsed_time(seconds):
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours >= 1:
        return f"{int(hours)}h {int(minutes)}m {seconds:.2f}s"
    if minutes >= 1:
        return f"{int(minutes)}m {seconds:.2f}s"
    return f"{seconds:.2f}s"

def wait_for_motion(app):
    if app.hmcControl.run_thread:
        app.hmcControl.run_thread.join()

def time_block(timer_map, key):
    class _Timer:
        def __enter__(self_inner):
            self_inner.start = time.perf_counter()
            return self_inner

        def __exit__(self_inner, exc_type, exc, tb):
            timer_map[key] += time.perf_counter() - self_inner.start

    return _Timer()

def print_timing_summary(timer_map, move_count, xy_move_count):
    print("[TIMER] Detailed timing summary:")
    for key in sorted(timer_map.keys()):
        print(f"[TIMER] {key}: {format_elapsed_time(timer_map[key])}")
    print(f"[TIMER] Moves executed: {move_count}")
    print(f"[TIMER] X/Y moves counted in movement timer: {xy_move_count}")

@dataclass
class VoltageFeedbackState:
    latest_voltage: Optional[float] = None
    latest_current: Optional[float] = None
    latest_direction: int = 0
    update_id: int = 0
    sample_time: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def update(self, voltage, current, direction):
        with self.lock:
            self.latest_voltage = voltage
            self.latest_current = current
            self.latest_direction = direction
            self.sample_time = time.perf_counter()
            self.update_id += 1

    def snapshot(self):
        with self.lock:
            return {
                "voltage": self.latest_voltage,
                "current": self.latest_current,
                "direction": self.latest_direction,
                "update_id": self.update_id,
                "sample_time": self.sample_time,
            }

def classify_voltage(voltage, threshold_voltage_1, threshold_voltage_2):
    if threshold_voltage_1 < voltage < threshold_voltage_2:
        return 1
    if voltage < threshold_voltage_1:
        return 2
    if voltage > threshold_voltage_2:
        return 3
    return 0

def feedback_direction_label(direction):
    if direction == 1:
        return "in_range"
    if direction == 2:
        return "low_lift_up"
    if direction == 3:
        return "high_move_down"
    return "unknown"

def read_voltage_sample(smu):
    response = smu.query('READ?').strip()
    if not response:
        smu.write("OUTP ON")
        time.sleep(0.02)
        response = smu.query('READ?').strip()
    voltage, current = map(float, response.split(',')[:2])
    return voltage, current

def read_first_last_point(filename):
    first_point = None
    last_point = None
    with open(f"{filename}.txt", "r") as file:
        for line in file:
            parts = line.strip().replace(',', ' ').split()
            if len(parts) >= 3:
                point = (float(parts[0]), float(parts[1]))
                if first_point is None:
                    first_point = point
                last_point = point
    if first_point is None or last_point is None:
        raise ValueError(f"No valid XY points found in {filename}.txt")
    return first_point[0], first_point[1], last_point[0], last_point[1]

class SmuVoltageSampler(Thread):
    def __init__(self, smu, threshold_voltage_1, threshold_voltage_2, state, stop_event, sample_interval=0.05, log_interval=1.0):
        super().__init__(daemon=True)
        self.smu = smu
        self.threshold_voltage_1 = threshold_voltage_1
        self.threshold_voltage_2 = threshold_voltage_2
        self.state = state
        self.stop_event = stop_event
        self.sample_interval = sample_interval
        self.log_interval = log_interval

    def run(self):
        last_log_time = 0.0
        last_direction = None
        print(f"[SMU SAMPLE] Sampler started, interval={self.sample_interval * 1000:.1f} ms")
        while not self.stop_event.is_set():
            try:
                voltage, current = read_voltage_sample(self.smu)
                direction = classify_voltage(voltage, self.threshold_voltage_1, self.threshold_voltage_2)
                self.state.update(voltage, current, direction)
                now = time.perf_counter()
                if direction != last_direction or now - last_log_time >= self.log_interval:
                    print(
                        f"[SMU SAMPLE] V={voltage:.4f} V, I={current:.4e} A, "
                        f"direction={feedback_direction_label(direction)}"
                    )
                    last_log_time = now
                    last_direction = direction
            except Exception as e:
                print(f"[WARN] SMU sampler error: {e}")
                self.state.update(None, None, 0)
            if self.stop_event.wait(self.sample_interval):
                break

class ZFeedbackWorker(Thread):
    def __init__(self, z_hmc, state, stop_event, step_um, max_safe_z, feedback_speed=1000):
        super().__init__(daemon=True)
        self.z_hmc = z_hmc
        self.state = state
        self.stop_event = stop_event
        self.step_um = step_um
        self.max_safe_z = max_safe_z
        self.feedback_speed = feedback_speed

    def run(self):
        print(f"[Z FEEDBACK] Worker started, step={self.step_um} um, speed={self.feedback_speed} um/s")
        self.z_hmc.set_speed(0, 0, self.feedback_speed)
        move_finish_time = time.perf_counter()
        while not self.stop_event.is_set():
            snapshot = self.state.snapshot()
            if snapshot["sample_time"] <= move_finish_time:
                self.stop_event.wait(0.001)
                continue

            direction = snapshot["direction"]
            if direction not in (2, 3):
                move_finish_time = time.perf_counter()
                continue

            delta_z = -self.step_um if direction == 2 else self.step_um
            predicted_z = self.z_hmc.z_current_position + delta_z
            print(
                f"[Z FEEDBACK] Applying {delta_z:+.3f} um from "
                f"{self.z_hmc.z_current_position:.3f} to {predicted_z:.3f} "
                f"for {feedback_direction_label(direction)}"
            )
            if predicted_z > self.max_safe_z:
                print(f"[ABORT] Z feedback would exceed safe limit: {predicted_z} um > {self.max_safe_z} um")
                self.stop_event.set()
                break

            try:
                previous_timeout = self.z_hmc.motion_timeout_seconds
                self.z_hmc.motion_timeout_seconds = max(5.0, abs(delta_z) / max(self.feedback_speed, 1) + 2.0)
                self.z_hmc.move(0, 0, delta_z)
                print(f"[Z FEEDBACK] Move complete, Z={self.z_hmc.z_current_position:.3f} um")
            except Exception as e:
                print(f"[WARN] Z feedback move failed: {e}")
                self.stop_event.set()
                break
            finally:
                self.z_hmc.motion_timeout_seconds = previous_timeout
                move_finish_time = time.perf_counter()

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return []
    
    print("Available COM Ports:")
    for i, port in enumerate(ports):
        print(f"[{i}] {port.device} - {port.description}")
    return ports

# Listener thread to read user input for speed updates
def user_input_listener(speed_queue):
    while True:
        try:
            user_input = input("Enter new Z speed in microns/sec: ")
            speed = float(user_input)
            speed_queue.put(speed)
        except ValueError:
            print("[WARN] Invalid input. Enter a number.")

def step_size_listener(step_queue):
    current_step = None
    while True:
        try:
            user_input = input("Enter new Z step size (µm): ").strip()
            if user_input:
                step = float(user_input)
                step_queue.put(step)
        except ValueError:
            print("[WARN] Invalid step size. Try again.")



def z_probe_move(hmc, app, total_distance, init_speed, speed_queue):
  
    app.hmcControl=hmc
    
    hmc.set_speed(1, 1, init_speed)
    remaining = total_distance

    # Start initial move
    app.command = '1'
    app.x_value = 0
    app.y_value = 0
    app.z_value = remaining
    app.start_thread()

    while True:
        # Break if motion is complete
        if not app.hmcControl.run_thread or not app.hmcControl.run_thread.is_alive():
            break

        if not speed_queue.empty():
            new_speed = speed_queue.get()
            print(f"[INFO] New speed input detected: {new_speed} µm/s")

            # Try stopping only if still moving
            if app.hmcControl.run_thread and app.hmcControl.run_thread.is_alive():
                try:
                    app.stop_process()
                    print("[INFO] Movement stopped")
                    time.sleep(0.2)
                except Exception as e:
                    print(f"[WARN] Stop failed or already complete: {e}")

            # Safely reset serial
            try:
                hmc.ser.close()
            except:
                pass

            time.sleep(0.2)

            try:
                hmc.ser.open()
                hmc.ser.reset_input_buffer()
            except Exception as e:
                print(f"[ERROR] Serial port reopen failed: {e}")
                return

            # Join stopped thread
            if app.hmcControl.run_thread:
                app.hmcControl.run_thread.join()

            # Compute remaining
            
            current = hmc.z_current_position
            remaining = total_distance - current
            print(f"[INFO] Resuming remaining distance: {remaining} µm")

            # Resume move with new speed
            hmc.set_speed(1, 1, new_speed)
            app.command = '1'
            app.x_value = 0
            app.y_value = 0
            app.z_value = remaining
            app.start_thread()

        time.sleep(0.1)

def z_movement(hmc,app,total_distance, initial_speed, speed_queue):


    hmc.set_speed(1, 1, initial_speed)
    remaining = total_distance
    
    # Start initial move
    app.command = '1'
    app.x_value = 0
    app.y_value = 0
    app.z_value = remaining
    app.start_thread()
   
    z_position_accumulated =0
    while True:
        # Break if motion is complete
        if not app.hmcControl.run_thread or not app.hmcControl.run_thread.is_alive():
            # moved_so_far += hmc.z_moving
            break
        if not speed_queue.empty():
            new_speed = speed_queue.get()
            print(f"[INFO] New speed input detected: {new_speed} µm/s")
            if app.hmcControl.run_thread and app.hmcControl.run_thread.is_alive():
                app.stop_process()
                print("Movement stopped")
                time.sleep(0.1)
            if app.hmcControl.run_thread:
                app.hmcControl.run_thread.join()

            # moved_so_far += hmc.z_moving
            moved_so_far = hmc.z_current_position
            #print(f"Z moving:{hmc.z_moving}")
            #current = hmc.z_current_position
            print(f"Z position so far: {moved_so_far}")
            #remaining = total_distance - current
            remaining = total_distance - moved_so_far
            if remaining <0:
                print("Movement completed;")
                hmc.ser.close()
                hmc.ser.open()
                break
            print(f"[INFO] Resuming remaining distance: {remaining} µm")
            #hmc.ser.close()
            #hmc.ser.open()
            hmc.set_speed(1, 1, new_speed)
            app.command = '1'
            app.x_value = 0
            app.y_value = 0
            #app.z_value = remaining
            # print(total_distance-(total_distance-hmc.z_moving))
            # print(f"total distance : {total_distance}")
            print(f"remaining: {remaining}")
            # print(f"total dist- remain = {total_distance- remaining}")
            app.z_value =remaining
            
            app.start_thread()
def main():
    move_thread = None
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
    response = int(input("Enter 1 to move specific distance, 2 to send to home, 3 to import data from text file, 4 to send probe:"))
    
    if response ==1:
        reset_all_serial()
        while True:
            x = float(input("Enter X movement (microns): "))
            y = float(input("Enter Y movement (microns): "))
            z = float(input("Enter Z movement (microns): "))

            app.command ='1'
            app.x_value =x
            app.y_value=y
            app.z_value=z
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

    elif response ==2:
        startup_all()
    elif response ==3:
        v = int(input("Enter velocity microns/sec:"))
        reset_all_serial()
        filename = input("Enter txt filename:")
        with open(f"{filename}.txt", "r") as file:
            i=1
            prev_x, prev_y, prev_z = 0, 0, 0

            for line in file:
                parts = line.strip().replace(',', ' ').split()
                if len(parts) == 3:
                    x, y, z = map(float, parts)

                    dx = x - prev_x
                    dy = y - prev_y
                    dz = z - prev_z
                    
                    theta = math.atan2(x- prev_x, y-prev_y)
                    sin_theta = math.sin(theta)
                    cos_theta = math.cos(theta)
                    vx = v*sin_theta
                    vy = v*cos_theta
                    set_all_speed(vx, vy, v)
                    prev_x, prev_y, prev_z = x, y, z
                    
                    try:
                        app.command = '1'
                        app.x_value = dx
                        app.y_value = dy
                        app.z_value = dz

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
                        reset_all_serial()
                        for h in [x_hmc, y_hmc, z_hmc]:
                            h.ser.reset_input_buffer()

                    i += 1
    elif response ==4:
        z_move = 30000
        init_speed = int(input("Enter initial speed of the probe:"))
     
        speed_queue = queue.Queue()

        Thread(target=user_input_listener, args=(speed_queue,), daemon=True).start()

        Thread(target=z_probe_move, args=(z_hmc, app, z_move, init_speed, speed_queue)).start()
    elif response ==5:
        from threading import Event

        z_move = 40000
        init_speed = 5000

        pause_event = Event()
        resume_event = Event()
        resume_event.set()

        z_hmc.set_speed(0, 0, init_speed)

        app.command = '1'
        app.x_value = 0
        app.y_value = 0
        app.z_value = z_move
        app.start_thread()

        def input_listener():
            while True:
                cmd = input("Enter 'p' to pause, 'r' to resume: ").lower()
                if cmd == 'p':
                    resume_event.clear()
                    try:
                        app.stop_process()
                        print("[INFO] Movement paused by user.")
                    except Exception as e:
                        print(f"[WARN] Could not pause: {e}")
                    pause_event.set()
                elif cmd == 'r' and pause_event.is_set():

                    current = z_hmc.z_current_position
                    remaining = z_move - current
                    print(f"[INFO] Resuming movement: remaining {remaining} µm")

                    z_hmc.set_speed(0, 0, init_speed)
                    app.command = '1'
                    app.x_value = 0
                    app.y_value = 0
                    app.z_value = remaining
                    app.start_thread()

                    resume_event.set()
                    pause_event.clear()

        Thread(target=input_listener, daemon=True).start()

        while True:
            if not app.hmcControl.run_thread or not app.hmcControl.run_thread.is_alive():
                if not pause_event.is_set():
                    break
            time.sleep(0.2)

        print("Final Z Position:", z_hmc.z_current_position)

    
    elif response == 6:
        init_speed = 1000
        timers = defaultdict(float)

        def set_fast_motion(xy_enabled, z_enabled=None):
            x_hmc.fast_mode = xy_enabled
            y_hmc.fast_mode = xy_enabled
            z_hmc.fast_mode = xy_enabled if z_enabled is None else z_enabled

        def sync_serial_once():
            for h in (x_hmc, y_hmc, z_hmc):
                h.ser.reset_input_buffer()

        class PatternAbort(Exception):
            pass

        def abort_patterning(smu, reason):
            print(f"[ABORT] {reason}")
            try:
                app.stop_process()
            except Exception as e:
                print(f"[WARN] Could not stop active movement cleanly: {e}")
            try:
                smumark2.out_off(smu)
            except Exception as e:
                print(f"[WARN] Could not turn SMU output off cleanly: {e}")
            raise PatternAbort(reason)

        def ensure_z_below_limit(target_z, max_safe_z, smu, context):
            if target_z > max_safe_z:
                abort_patterning(
                    smu,
                    f"{context}: target Z {target_z} um exceeds safe limit {max_safe_z} um"
                )

        def find_contact_point_custom(smu, contact_voltage, contact_compliance_current_ua, threshold_current_ua, liftoff_height, max_safe_z):
            smumark2.use_case_1(smu, voltage=contact_voltage, compliance_current_ua=contact_compliance_current_ua)
            smumark2.out_on(smu)
            step_size = liftoff_height -50
            total_distance = z_hmc.z_current_position
            ensure_z_below_limit(total_distance + step_size, max_safe_z, smu, "Re-probe coarse contact move")
            print(f"[Z PROBE] Starting with {step_size} µm step, then 1 µm steps.")
            app.command = '1'
            app.x_value = 0
            app.y_value = 0
            app.z_value = step_size
            with time_block(timers, "motion.start_thread"):
                app.start_thread()
            with time_block(timers, "motion.wait_for_completion"):
                app.hmcControl.run_thread.join()
            total_distance += step_size

            while True:
                ensure_z_below_limit(total_distance + 1, max_safe_z, smu, "Re-probe fine contact move")
                app.command = '1'
                app.x_value = 0
                app.y_value = 0
                app.z_value = 1
                with time_block(timers, "motion.start_thread"):
                    app.start_thread()
                with time_block(timers, "motion.wait_for_completion"):
                    app.hmcControl.run_thread.join()
                total_distance += 1

                with time_block(timers, "smu.check_current"):
                    status = smumark2.check_current(
                        smu,
                        threshold_current_1=threshold_current_ua * 1e-6,
                        ensure_output_on=False,
                        verbose=False,
                    )
                if status == 1:
                    print(f"[CONTACT] Contact at {total_distance} µm")
                    z_hmc.z_current_position = total_distance
                    return total_distance

        def plot_from_file(v, filename, z_contact_point, smu, delta_z, voltage_threshold_1, voltage_threshold_2, volt_source, curr_comp, contact_voltage, contact_compliance_current_ua, threshold_current_ua, liftoff_height, max_safe_z):
            with time_block(timers, "smu.use_case_2"):
                smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
            smumark2.out_on(smu)
            prev_flag = 0
            pattern_start_time = time.perf_counter()
            xy_movement_time = 0
            xy_move_count = 0
            xy_commanded_distance = 0.0
            move_count = 0
            print("[TIMER] Patterning timer started.")
            with open(f"{filename}.txt", "r") as file:
                i = 1
                prev_x, prev_y = 0, 0
                prev_z = z_contact_point
                liftoff = False
                for line in file:
                    parts = line.strip().replace(',', ' ').split()
                    if len(parts) >= 3:
                        x, y, flag = float(parts[0]), float(parts[1]), int(parts[2])
                        dx = x - prev_x
                        dy = y - prev_y
                        print(f"flag:{flag}")
                        if i == 1:
                            prev_x, prev_y = x, y
                            prev_z = z_contact_point
                            i += 1
                            z = z_contact_point
                            continue
                        elif flag == 1:
                            if prev_flag == 0:
                                print("[INFO] liftoff Initiated")
                                z = prev_z - liftoff_height
                                liftoff = True
                            else:
                                z = prev_z
                        elif flag == 0:
                            if liftoff:
                                print("[INFO] Finding new contact point...")
                                z = find_contact_point_custom(smu, contact_voltage, contact_compliance_current_ua, threshold_current_ua, liftoff_height, max_safe_z)
                                with time_block(timers, "smu.use_case_2"):
                                    smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
                                prev_z = z
                                liftoff = False
                            else:
                                with time_block(timers, "smu.check_voltage"):
                                    direction = smumark2.check_voltage(
                                        smu,
                                        threshold_voltage_1=voltage_threshold_1,
                                        threshold_voltage_2=voltage_threshold_2,
                                        ensure_output_on=False,
                                        verbose=False,
                                    )
                                if direction == 2:
                                    print("[FEEDBACK] Voltage too low — going down")
                                    z = prev_z - delta_z
                                    print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                                elif direction == 3:
                                    print("[FEEDBACK] Voltage too high — going up")
                                    z = prev_z + delta_z
                                    print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                                else:
                                    print("[FEEDBACK] Voltage in range — Z aligned")
                                    z = prev_z
                        else:
                            with time_block(timers, "smu.check_voltage"):
                                direction = smumark2.check_voltage(
                                    smu,
                                    threshold_voltage_1=voltage_threshold_1,
                                    threshold_voltage_2=voltage_threshold_2,
                                    ensure_output_on=False,
                                    verbose=False,
                                )
                            if direction == 2:
                                print("[FEEDBACK] Voltage too low — going down")
                                z = prev_z - delta_z
                                print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                            elif direction == 3:
                                print("[FEEDBACK] Voltage too high — going up")
                                z = prev_z + delta_z
                                print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                            else:
                                print("[FEEDBACK] Voltage in range — Z aligned")
                                z = prev_z

                        ensure_z_below_limit(z, max_safe_z, smu, f"Pattern move {i}")
                        dz = z - prev_z
                        segment_xy_distance = math.hypot(dx, dy)
                        theta = math.atan2(x - prev_x, y - prev_y)
                        sin_theta = math.sin(theta)
                        cos_theta = math.cos(theta)
                        vx = v * sin_theta
                        vy = v * cos_theta
                        with time_block(timers, "motion.set_speed"):
                            set_all_speed(vx, vy, 1000)
                        if i == 1:
                            with time_block(timers, "motion.set_speed"):
                                set_all_speed(5000, 5000, 5000)
                        prev_x, prev_y, prev_z = x, y, z

                        try:
                            app.command = '1'
                            app.x_value = dx
                            app.y_value = dy
                            app.z_value = dz
                            move_start_time = time.perf_counter()
                            with time_block(timers, "motion.start_thread"):
                                app.start_thread()
                            with time_block(timers, "motion.wait_for_completion"):
                                wait_for_motion(app)
                            move_elapsed = time.perf_counter() - move_start_time
                            if dx != 0 or dy != 0:
                                xy_movement_time += move_elapsed
                                xy_move_count += 1
                                xy_commanded_distance += segment_xy_distance
                            move_count += 1

                            print("Final Position:")
                            print(f"X: {x_hmc.current_x} µm")
                            print(f"Y: {y_hmc.current_y} µm")
                            print(f"Z: {z_hmc.current_z} µm")
                            print(f"{i} MOVE COMPLETED")

                        except Exception as e:
                            print(f"[ERROR] Move {i} failed: {e}")
                            reset_all_serial()
                            for h in [x_hmc, y_hmc, z_hmc]:
                                h.ser.reset_input_buffer()
                        prev_flag = flag
                        i += 1
                smumark2.out_off(smu)
            pattern_elapsed = time.perf_counter() - pattern_start_time
            print(f"[TIMER] Patterning completed in {format_elapsed_time(pattern_elapsed)}.")
            print(f"[TIMER] X/Y movement time: {format_elapsed_time(xy_movement_time)} across {xy_move_count} moves.")
            if xy_move_count:
                estimated_xy_travel = xy_commanded_distance / v if v else 0
                print(f"[TIMER] Commanded XY distance: {xy_commanded_distance:.2f} µm")
                print(f"[TIMER] Estimated pure XY travel at {v} µm/s: {format_elapsed_time(estimated_xy_travel)}")
                print(f"[TIMER] Average XY distance per move: {xy_commanded_distance / xy_move_count:.2f} µm")
            print_timing_summary(timers, move_count, xy_move_count)

        def find_contact_point(initial_step_size, smu, threshold_current_ua, step_queue):
            step_size = initial_step_size
            total_distance = 0
            i = 0

            print("[INFO] Starting Z probing. You can enter new step sizes any time.")

            while True:
                if not step_queue.empty():
                    step_size = step_queue.get()
                    print(f"[INFO] Updated step size: {step_size} µm")

                print(f"[STEP {i+1}] Moving Z down by {step_size} µm...")
                app.command = '1'
                app.x_value = 0
                app.y_value = 0
                app.z_value = step_size
                app.start_thread()

                if app.hmcControl.run_thread:
                    app.hmcControl.run_thread.join()

                total_distance += step_size
                i += 1
                with time_block(timers, "smu.check_current"):
                    status = smumark2.check_current(
                        smu,
                        threshold_current_1=threshold_current_ua * 1e-6,
                        ensure_output_on=False
                        ,verbose=False
                    )

                if status == 1:
                    print("[CONTACT] Probe contact confirmed by current.")
                    z_hmc.z_current_position = total_distance
                    return total_distance

        set_all_speed(1000, 1000, init_speed)

        contact_voltage = float(input("Enter source voltage in V for contact detection (use case 1): "))
        contact_compliance_current_ua = float(input("Enter compliance current in micro amps for contact detection (use case 1): "))
        threshold_current_ua = float(input("Enter threshold current in micro amps to detect contact: "))
        volt_source = float(input("Enter source voltage (use_case_2): "))
        curr_comp = float(input("Enter compliance current in micro amps (use_case_2): "))
        voltage_threshold_1 = float(input("Enter voltage lower threshold for feedback (V): "))
        voltage_threshold_2 = float(input("Enter voltage upper threshold for feedback (V): "))
        delta_z = float(input("Enter Z adjustment step for plotting (in µm): "))
        z_contact_step = float(input("Enter Z adjustment step for finding contact point:"))
        speed = float(input("Enter speed for xy axis: "))
        liftoff_height = float(input("Enter liftoff height in µm: "))
        max_safe_z_margin = float(input("Enter maximum allowed Z increase above first contact point (in um, e.g. 100): "))
        filename = input("Enter filename: ")

        with time_block(timers, "smu.init_smu"):
            smu = smumark2.init_smu()
        with time_block(timers, "smu.reset_smu"):
            smumark2.reset_smu(smu)
        with time_block(timers, "smu.use_case_1"):
            smumark2.use_case_1(smu, voltage=contact_voltage, compliance_current_ua=contact_compliance_current_ua) 
        smumark2.out_on(smu)

        sync_serial_once()
        set_fast_motion(True, False)

        with time_block(timers, "file.read_first_point"):
            with open(f"{filename}.txt", "r") as file:
                for line in file:
                    parts = line.strip().replace(',', ' ').split()
                    if len(parts) >= 3:
                        first_x, first_y, flag = float(parts[0]), float(parts[1]), int(parts[2])
                        break
                
        dx = first_x - x_hmc.current_x
        dy = first_y - y_hmc.current_y
        set_all_speed(5000, 5000, 5000)

        app.command = '1'
        app.x_value = dx
        app.y_value = dy
        app.z_value = 0
        with time_block(timers, "motion.start_thread"):
            app.start_thread()
        with time_block(timers, "motion.wait_for_completion"):
            wait_for_motion(app)
        
        set_all_speed(speed, speed, init_speed)

        step_queue = queue.Queue()
        Thread(target=step_size_listener, args=(step_queue,), daemon=True).start()

        z_contact_point = find_contact_point(initial_step_size=z_contact_step, smu=smu, threshold_current_ua=threshold_current_ua, step_queue=step_queue)

        z_hmc.current_z = z_contact_point
        z_contact_point = z_hmc.z_current_position
        max_safe_z = z_contact_point + max_safe_z_margin
        print(f"[SAFETY] Patterning will abort if Z exceeds {max_safe_z} um.")
        with time_block(timers, "smu.reset_smu"):
            smumark2.reset_smu(smu)

        try:
            plot_from_file(speed, filename, z_contact_point, smu, delta_z, voltage_threshold_1, voltage_threshold_2, volt_source, curr_comp, contact_voltage, contact_compliance_current_ua, threshold_current_ua, liftoff_height, max_safe_z)
        except PatternAbort:
            print("[SAFETY] Patterning stopped because the Z safety limit was reached.")

        set_fast_motion(False)
        print("Final Z Position:", z_hmc.z_current_position)
        startup_all()
    elif response == 7:
        set_all_speed(5000, 5000, 5000)  

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

    elif response == 8:
        init_speed = 1000
        timers = defaultdict(float)

        sample_interval_ms = float(input("Enter SMU sample interval in milliseconds (e.g. 50): "))

        class PatternAbort(Exception):
            pass

        def abort_patterning(smu, reason, stop_event=None):
            print(f"[ABORT] {reason}")
            if stop_event is not None:
                stop_event.set()
            try:
                app.stop_process()
            except Exception as e:
                print(f"[WARN] Could not stop active movement cleanly: {e}")
            try:
                smumark2.out_off(smu)
            except Exception as e:
                print(f"[WARN] Could not turn SMU output off cleanly: {e}")
            raise PatternAbort(reason)

        def ensure_z_below_limit(target_z, max_safe_z, smu, context, stop_event=None):
            if target_z > max_safe_z:
                abort_patterning(
                    smu,
                    f"{context}: target Z {target_z} um exceeds safe limit {max_safe_z} um",
                    stop_event=stop_event,
                )

        def find_contact_point(initial_step_size, smu, threshold_current_ua, step_queue):
            step_size = initial_step_size
            total_distance = 0
            i = 0

            print("[INFO] Starting Z probing. You can enter new step sizes any time.")

            while True:
                if not step_queue.empty():
                    step_size = step_queue.get()
                    print(f"[INFO] Updated step size: {step_size} µm")

                print(f"[STEP {i+1}] Moving Z down by {step_size} µm...")
                app.command = '1'
                app.x_value = 0
                app.y_value = 0
                app.z_value = step_size
                app.start_thread()

                if app.hmcControl.run_thread:
                    app.hmcControl.run_thread.join()

                total_distance += step_size
                i += 1
                with time_block(timers, "smu.check_current"):
                    status = smumark2.check_current(
                        smu,
                        threshold_current_1=threshold_current_ua * 1e-6,
                        ensure_output_on=True,
                        verbose=True,
                    )

                if status == 1:
                    print("[CONTACT] Probe contact confirmed by current.")
                    z_hmc.z_current_position = total_distance
                    return total_distance

        def set_fast_motion(xy_enabled, z_enabled=None):
            x_hmc.fast_mode = xy_enabled
            y_hmc.fast_mode = xy_enabled
            z_hmc.fast_mode = xy_enabled if z_enabled is None else z_enabled

        def sync_serial_once():
            for h in (x_hmc, y_hmc, z_hmc):
                h.ser.reset_input_buffer()

        def move_xy(dx, dy):
            app.hmcControl = x_hmc
            app.command = '1'
            app.x_value = dx
            app.y_value = dy
            app.z_value = 0
            with time_block(timers, "motion.start_thread"):
                app.start_thread()
            with time_block(timers, "motion.wait_for_completion"):
                wait_for_motion(app)

        set_all_speed(1000, 1000, init_speed)

        contact_voltage = float(input("Enter source voltage in V for contact detection (use case 1): "))
        contact_compliance_current_ua = float(input("Enter compliance current in micro amps for contact detection (use case 1): "))
        threshold_current_ua = float(input("Enter threshold current in micro amps to detect contact: "))
        volt_source = float(input("Enter source voltage (use_case_2): "))
        curr_comp = float(input("Enter compliance current in micro amps (use_case_2): "))
        voltage_threshold_1 = float(input("Enter voltage lower threshold for feedback (V): "))
        voltage_threshold_2 = float(input("Enter voltage upper threshold for feedback (V): "))
        delta_z = float(input("Enter Z adjustment step for plotting (in µm): "))
        z_contact_step = float(input("Enter Z adjustment step for finding contact point:"))
        speed = float(input("Enter speed for xy axis: "))
        z_feedback_speed = float(input("Enter speed for Z feedback in µm/s (e.g. 3000): "))
        liftoff_height = float(input("Enter liftoff height in µm: "))
        max_safe_z_margin = float(input("Enter maximum allowed Z increase above first contact point (in um, e.g. 100): "))
        filename = input("Enter filename: ")

        with time_block(timers, "smu.init_smu"):
            smu = smumark2.init_smu()
        with time_block(timers, "smu.reset_smu"):
            smumark2.reset_smu(smu)
        with time_block(timers, "smu.use_case_1"):
            smumark2.use_case_1(smu, voltage=contact_voltage, compliance_current_ua=contact_compliance_current_ua)
        smumark2.out_on(smu)

        sync_serial_once()
        set_fast_motion(True, False)

        first_x, first_y, last_x, last_y = read_first_last_point(filename)
        dx = first_x - x_hmc.current_x
        dy = first_y - y_hmc.current_y

        set_all_speed(5000, 5000, 5000)
        move_xy(dx, dy)

        set_all_speed(speed, speed, init_speed)

        step_queue = queue.Queue()
        Thread(target=step_size_listener, args=(step_queue,), daemon=True).start()

        z_contact_point = find_contact_point(initial_step_size=z_contact_step, smu=smu, threshold_current_ua=threshold_current_ua, step_queue=step_queue)
        z_hmc.current_z = z_contact_point
        z_contact_point = z_hmc.z_current_position
        max_safe_z = z_contact_point + max_safe_z_margin
        print(f"[SAFETY] Patterning will abort if Z exceeds {max_safe_z} um.")
        with time_block(timers, "smu.reset_smu"):
            smumark2.reset_smu(smu)
        with time_block(timers, "smu.use_case_2"):
            smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
        smumark2.out_on(smu)

        feedback_state = VoltageFeedbackState()
        feedback_stop = threading.Event()
        sampler = SmuVoltageSampler(
            smu,
            voltage_threshold_1,
            voltage_threshold_2,
            feedback_state,
            feedback_stop,
            sample_interval=sample_interval_ms / 1000.0,
        )
        z_worker = ZFeedbackWorker(
            z_hmc,
            feedback_state,
            feedback_stop,
            delta_z,
            max_safe_z,
            feedback_speed=z_feedback_speed,
        )

        # Decouple Z axis controller from App during X/Y movement to prevent serial port collisions
        app.z_hmc = None

        sampler.start()
        z_worker.start()

        try:
            line_dx = last_x - first_x
            line_dy = last_y - first_y
            theta = math.atan2(line_dx, line_dy)
            vx = speed * math.sin(theta)
            vy = speed * math.cos(theta)
            x_hmc.set_speed(vx, 0, 0)
            y_hmc.set_speed(0, vy, 0)

            print("[INFO] Starting threaded straight-line motion with background SMU feedback...")
            move_xy(line_dx, line_dy)
        finally:
            # Re-couple Z axis controller to App
            app.z_hmc = z_hmc
            app.hmcControl = z_hmc
            feedback_stop.set()
            sampler.join(timeout=2)
            z_worker.join(timeout=2)
            smumark2.out_off(smu)
            set_fast_motion(False, False)

        print("[INFO] Threaded straight-line feedback run complete.")
        print("Final Z Position:", z_hmc.z_current_position)
        startup_all()


if __name__ == "__main__":
    main()
