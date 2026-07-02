# lithography.py
# ponytail: shared helper functions and threads for patterning modes (Mode 6 and 8).

import time
import threading
import csv
import os
from threading import Thread
from dataclasses import dataclass, field
from typing import Optional
import config
import smumark2

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


class CSVLogger:
    def __init__(self, csv_path, app):
        self.csv_path = csv_path
        self.app = app
        self.data_point_counter = 0
        self.csv_file = None
        self.csv_writer = None
        if self.csv_path:
            try:
                os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
                self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(["data point", "x", "y", "voltage", "current", "move number", "flag"])
                self.csv_file.flush()
            except Exception as e:
                print(f"[ERROR] Failed to initialize CSV logger: {e}")

    def log(self, x, y, voltage, current, move_number, flag):
        if self.csv_writer:
            try:
                self.data_point_counter += 1
                self.csv_writer.writerow([self.data_point_counter, x, y, voltage, current, move_number, flag])
                self.csv_file.flush()
            except Exception as e:
                print(f"[ERROR] Failed to write CSV row: {e}")

    def close(self):
        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass


class PatternAbort(Exception):
    """Raised when patterning needs to be aborted due to safety or user request."""
    pass


def time_block(timer_map, key):
    class _Timer:
        def __enter__(self_inner):
            self_inner.start = time.perf_counter()
            return self_inner
        def __exit__(self_inner, exc_type, exc_val, exc_tb):
            timer_map[key] += time.perf_counter() - self_inner.start
    return _Timer()


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
        return "high_move_down"
    if direction == 3:
        return "low_lift_up"
    return "unknown"


def read_voltage_sample(smu):
    return smumark2.read_voltage_sample(smu, ensure_output_on=True)


def abort_patterning(smu, reason, stop_event=None):
    print(f"[ABORT] Aborting patterning: {reason}")
    if stop_event:
        stop_event.set()
    try:
        smumark2.out_off(smu)
    except Exception as e:
        print(f"[WARN] Failed to turn SMU output off during abort: {e}")
    raise PatternAbort(reason)


def ensure_z_below_limit(z_hmc, target_z, max_safe_z, smu, context, stop_event=None):
    if target_z > max_safe_z:
        abort_patterning(
            smu,
            f"{context}: target Z {target_z} um exceeds safe limit {max_safe_z} um",
            stop_event
        )


def find_contact_point(app, z_hmc, initial_step_size, smu, threshold_current_ua, step_queue, timers, ensure_output_on=False, verbose=False):
    step_size = initial_step_size
    total_distance = z_hmc.z_current_position
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
                ensure_output_on=ensure_output_on,
                verbose=verbose
            )
        app.smu_voltage = getattr(smu, "latest_voltage", 0.0)
        app.smu_current = getattr(smu, "latest_current", 0.0)
        if getattr(app, "csv_logger", None):
            app.csv_logger.log(
                getattr(getattr(app, "x_hmc", None), "current_x", 0.0),
                getattr(getattr(app, "y_hmc", None), "current_y", 0.0),
                app.smu_voltage,
                app.smu_current,
                getattr(app, "moves_done", 0),
                getattr(app, "current_flag", 0)
            )

        if status == 1:
            print("[CONTACT] Probe contact confirmed by current.")
            z_hmc.z_current_position = total_distance
            return total_distance


def find_contact_point_custom(app, z_hmc, smu, contact_voltage, contact_compliance_current_ua, threshold_current_ua, liftoff_height, max_safe_z, timers, stop_event=None):
    smumark2.use_case_1(smu, voltage=contact_voltage, compliance_current_ua=contact_compliance_current_ua)
    smumark2.out_on(smu)
    step_size = liftoff_height - 50
    total_distance = z_hmc.z_current_position
    ensure_z_below_limit(z_hmc, total_distance + step_size, max_safe_z, smu, "Re-probe coarse contact move", stop_event)
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
        ensure_z_below_limit(z_hmc, total_distance + 1, max_safe_z, smu, "Re-probe fine contact move", stop_event)
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
        app.smu_voltage = getattr(smu, "latest_voltage", 0.0)
        app.smu_current = getattr(smu, "latest_current", 0.0)
        if getattr(app, "csv_logger", None):
            app.csv_logger.log(
                getattr(getattr(app, "x_hmc", None), "current_x", 0.0),
                getattr(getattr(app, "y_hmc", None), "current_y", 0.0),
                app.smu_voltage,
                app.smu_current,
                getattr(app, "moves_done", 0),
                getattr(app, "current_flag", 0)
            )
        if status == 1:
            print(f"[CONTACT] Contact at {total_distance} µm")
            z_hmc.z_current_position = total_distance
            return total_distance


def read_pattern_file(filename):
    lines = []
    with open(f"{filename}.txt", "r") as file:
        for line in file:
            parts = line.strip().replace(',', ' ').split()
            if len(parts) >= 2:
                flag = int(float(parts[2])) if len(parts) >= 3 else 0
                lines.append((float(parts[0]), float(parts[1]), flag))
    return lines


class SmuVoltageSampler(Thread):
    def __init__(self, smu, threshold_voltage_1, threshold_voltage_2, state, stop_event, app=None, sample_interval=0.05, log_interval=1.0):
        super().__init__(daemon=True)
        self.smu = smu
        self.threshold_voltage_1 = threshold_voltage_1
        self.threshold_voltage_2 = threshold_voltage_2
        self.state = state
        self.stop_event = stop_event
        self.app = app
        self.sample_interval = sample_interval
        self.log_interval = log_interval
        self.enabled = threading.Event()
        self.enabled.set()

    def run(self):
        last_log_time = 0.0
        last_direction = None
        print(f"[SMU SAMPLE] Sampler started, interval={self.sample_interval * 1000:.1f} ms")
        while not self.stop_event.is_set():
            if not self.enabled.is_set():
                self.stop_event.wait(0.005)
                continue
            try:
                voltage, current = read_voltage_sample(self.smu)
                setattr(self.smu, "latest_voltage", voltage)
                setattr(self.smu, "latest_current", current)
                direction = classify_voltage(voltage, self.threshold_voltage_1, self.threshold_voltage_2)
                self.state.update(voltage, current, direction)

                if self.app:
                    self.app.smu_voltage = voltage
                    self.app.smu_current = current
                    
                    # Update Z feedback direction if not overridden by liftoff/finding contact state
                    current_dir = getattr(self.app, "z_feedback_direction", "inactive")
                    if current_dir not in ("liftoff", "finding_contact"):
                        if direction == 2:
                            self.app.z_feedback_direction = "voltage_low"
                        elif direction == 3:
                            self.app.z_feedback_direction = "voltage_high"
                        elif direction == 1:
                            self.app.z_feedback_direction = "aligned"
                        else:
                            self.app.z_feedback_direction = "aligned"
                
                # Log to app's CSV logger if available
                if self.app and getattr(self.app, "csv_logger", None):
                    self.app.csv_logger.log(
                        getattr(getattr(self.app, "x_hmc", None), "current_x", 0.0),
                        getattr(getattr(self.app, "y_hmc", None), "current_y", 0.0),
                        voltage,
                        current,
                        getattr(self.app, "moves_done", 0),
                        getattr(self.app, "current_flag", 0)
                    )
                
                now = time.perf_counter()
                if direction != last_direction or now - last_log_time >= self.log_interval:
                    print(
                        f"[SMU SAMPLE] V={voltage:.4f} V, I={current:.4e} A, "
                        f"direction={feedback_direction_label(direction)}"
                    )
                    last_log_time = now
                    last_direction = direction
            except Exception as e:
                if self.stop_event.is_set():
                    break
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
        self.enabled = threading.Event()
        self.enabled.set()
        self.is_moving = False

    def run(self):
        print(f"[Z FEEDBACK] Worker started, step={self.step_um} um, speed={self.feedback_speed} um/s")
        self.z_hmc.set_speed(0, 0, self.feedback_speed)
        move_finish_time = time.perf_counter()
        while not self.stop_event.is_set():
            if not self.enabled.is_set():
                self.stop_event.wait(0.005)
                continue
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
                self.is_moving = True
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
                self.is_moving = False
