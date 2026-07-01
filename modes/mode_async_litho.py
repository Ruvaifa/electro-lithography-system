# modes/mode_async_litho.py
# ponytail: asynchronous threaded lithography with continuous Z feedback (Mode 8).

import time
import math
import queue
import threading
from threading import Thread
from collections import defaultdict
import main
import smumark2
import lithography

def run(app, x_hmc, y_hmc, z_hmc, reset_all_serial_fn, set_all_speed_fn, startup_all_fn, params=None):
    init_speed = 1000
    timers = defaultdict(float)
    disable_ramps = False
    
    # Prompt or load variables
    if params is not None:
        disable_ramps = params.get('disable_ramps', True)
        contact_voltage = float(params.get('contact_voltage', 5.0))
        contact_compliance_current_ua = float(params.get('contact_compliance_current_ua', 0.3))
        threshold_current_ua = float(params.get('threshold_current_ua', 0.1))
        volt_source = float(params.get('volt_source', 20.0))
        curr_comp = float(params.get('curr_comp', 1.0))
        voltage_threshold_1 = float(params.get('voltage_threshold_1', 1.9))
        voltage_threshold_2 = float(params.get('voltage_threshold_2', 2.2))
        delta_z = float(params.get('delta_z', 0.1))
        z_contact_step = float(params.get('z_contact_step', 1.0))
        speed = float(params.get('speed', 5000))
        z_feedback_speed = float(params.get('z_feedback_speed', 3000))
        liftoff_height = float(params.get('liftoff_height', 1000))
        max_safe_z_margin = float(params.get('max_safe_z_margin', 100))
        sample_interval_ms = float(params.get('sample_interval_ms', 50))
        filename = params.get('filename', 'circles')
    else:
        disable_ramps = input("Disable motor acceleration/deceleration ramps? (y/n): ").strip().lower() == 'y'
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
        sample_interval_ms = float(input("Enter sampling interval in ms (e.g. 50): "))
        filename = input("Enter filename: ")

    # Initialize live telemetry attributes for frontend monitoring
    app.patterning_active = True
    app.patterning_mode = 8
    app.total_moves = 0
    app.moves_done = 0
    app.moves_left = 0
    app.z_feedback_direction = "inactive"
    app.smu_voltage = 0.0
    app.smu_current = 0.0

    with lithography.time_block(timers, "smu.init_smu"):
        smu = smumark2.init_smu()
    with lithography.time_block(timers, "smu.reset_smu"):
        smumark2.reset_smu(smu)
    with lithography.time_block(timers, "smu.use_case_1"):
        smumark2.use_case_1(smu, voltage=contact_voltage, compliance_current_ua=contact_compliance_current_ua)
    smumark2.out_on(smu)

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
        with lithography.time_block(timers, "motion.start_thread"):
            app.start_thread()
        with lithography.time_block(timers, "motion.wait_for_completion"):
            main.wait_for_motion(app)

    sync_serial_once()
    set_fast_motion(True, False)

    # Read pattern points
    try:
        if params is not None and params.get("file_content"):
            # Parse from uploaded file content
            file_content = params.get("file_content")
            lines = []
            for line in file_content.splitlines():
                parts = line.strip().replace(',', ' ').split()
                if len(parts) >= 2:
                    flag = int(float(parts[2])) if len(parts) >= 3 else 0
                    lines.append((float(parts[0]), float(parts[1]), flag))
        else:
            lines = lithography.read_pattern_file(filename)
    except Exception as e:
        print(f"[ERROR] Failed to read pattern points: {e}")
        return

    if not lines:
        print(f"[ERROR] No valid data found in {filename or 'uploaded pattern'}")
        return

    first_x, first_y, first_flag = lines[0]
    dx = first_x - x_hmc.current_x
    dy = first_y - y_hmc.current_y

    set_all_speed_fn(5000, 5000, 5000)
    move_xy(dx, dy)

    set_all_speed_fn(speed, speed, init_speed)

    step_queue = queue.Queue()
    Thread(target=main.step_size_listener, args=(step_queue,), daemon=True).start()

    z_contact_point = lithography.find_contact_point(
        app=app,
        z_hmc=z_hmc,
        initial_step_size=z_contact_step,
        smu=smu,
        threshold_current_ua=threshold_current_ua,
        step_queue=step_queue,
        timers=timers,
        ensure_output_on=True,
        verbose=True
    )
    z_hmc.current_z = z_contact_point
    z_contact_point = z_hmc.z_current_position
    max_safe_z = z_contact_point + max_safe_z_margin
    print(f"[SAFETY] Patterning will abort if Z exceeds {max_safe_z} um.")

    with lithography.time_block(timers, "smu.reset_smu"):
        smumark2.reset_smu(smu)
    with lithography.time_block(timers, "smu.use_case_2"):
        smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
    smumark2.out_on(smu)

    feedback_state = lithography.VoltageFeedbackState()
    feedback_stop = threading.Event()
    sampler = lithography.SmuVoltageSampler(
        smu,
        voltage_threshold_1,
        voltage_threshold_2,
        feedback_state,
        feedback_stop,
        sample_interval=sample_interval_ms / 1000.0,
    )
    z_worker = lithography.ZFeedbackWorker(
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

    pattern_start_time = time.perf_counter()
    xy_movement_time = 0
    xy_move_count = 0
    xy_commanded_distance = 0.0
    move_count = 0

    try:
        if disable_ramps:
            try:
                x_hmc.acceleration_and_deceleration(False, False)
                y_hmc.acceleration_and_deceleration(False, False)
                print("[INFO] Acceleration/deceleration ramps disabled.")
            except Exception as e:
                print(f"[WARN] Failed to disable ramps: {e}")

        prev_x, prev_y = first_x, first_y
        prev_flag = first_flag
        liftoff = False
        app.total_moves = len(lines)
        for idx in range(1, len(lines)):
            if app.stop or x_hmc.stop_thread or y_hmc.stop_thread or z_hmc.stop_thread:
                print("[ABORT] Async patterning run aborted mid-way by user!")
                break

            app.moves_done = idx
            app.moves_left = max(0, app.total_moves - idx)
            app.smu_voltage = feedback_state.latest_voltage if feedback_state.latest_voltage is not None else 0.0
            app.smu_current = feedback_state.latest_current if feedback_state.latest_current is not None else 0.0
            
            x, y, flag = lines[idx]
            dx = x - x_hmc.current_x
            dy = y - y_hmc.current_y
            print(f"[MOVE {idx}] Target coordinates: X={x:.1f}, Y={y:.1f}, flag={flag}")

            if flag == 1:
                app.z_feedback_direction = "liftoff"
                if prev_flag == 0:
                    print("[INFO] liftoff Initiated")
                    z_worker.enabled.clear()
                    sampler.enabled.clear()
                    try:
                        # Move Z up by liftoff_height (negative logical Z)
                        z_hmc.move(0, 0, -liftoff_height)
                        print(f"[LIFTOFF] Z moved up by {liftoff_height} um")
                    except Exception as e:
                        print(f"[WARN] Liftoff Z move failed: {e}")
                    liftoff = True
                else:
                    pass
            else:
                # flag == 0
                if liftoff:
                    app.z_feedback_direction = "finding_contact"
                    print("[INFO] Finding new contact point...")
                    z_worker.enabled.clear()
                    sampler.enabled.clear()
                    # Couple Z back temporarily for find_contact_point
                    app.z_hmc = z_hmc
                    app.hmcControl = z_hmc
                    
                    z_contact_point = lithography.find_contact_point_custom(
                        app=app,
                        z_hmc=z_hmc,
                        smu=smu,
                        contact_voltage=contact_voltage,
                        contact_compliance_current_ua=contact_compliance_current_ua,
                        threshold_current_ua=threshold_current_ua,
                        liftoff_height=liftoff_height,
                        max_safe_z=max_safe_z,
                        timers=timers,
                        stop_event=feedback_stop
                    )
                    z_hmc.current_z = z_contact_point
                    max_safe_z = z_hmc.z_current_position + max_safe_z_margin
                    print(f"[SAFETY] Updated max_safe_z: {max_safe_z} um")
                    
                    with lithography.time_block(timers, "smu.reset_smu"):
                        smumark2.reset_smu(smu)
                    with lithography.time_block(timers, "smu.use_case_2"):
                        smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
                    smumark2.out_on(smu)
                    
                    # Decouple Z axis again for XY move
                    app.z_hmc = None
                    
                    z_worker.max_safe_z = max_safe_z
                    z_worker.enabled.set()
                    sampler.enabled.set()
                    liftoff = False
                else:
                    direction = feedback_state.latest_direction
                    if direction == 2:
                        app.z_feedback_direction = "voltage_low"
                    elif direction == 3:
                        app.z_feedback_direction = "voltage_high"
                    elif direction == 1:
                        app.z_feedback_direction = "aligned"
                    else:
                        app.z_feedback_direction = "aligned"
                    z_worker.enabled.set()
                    sampler.enabled.set()

            actual_dx = round(dx / 0.2) * 0.2
            actual_dy = round(dy / 0.2) * 0.2

            # Calculate speed for XY segment
            segment_xy_distance = math.hypot(actual_dx, actual_dy)
            vx, vy = main.compute_synchronized_speeds(actual_dx, actual_dy, speed)

            # Set segment speeds
            x_hmc.set_speed(vx, 0, 0)
            y_hmc.set_speed(0, vy, 0)

            # Move X/Y
            if actual_dx != 0 or actual_dy != 0:
                move_start_time = time.perf_counter()
                move_xy(actual_dx, actual_dy)
                move_elapsed = time.perf_counter() - move_start_time
                xy_movement_time += move_elapsed
                xy_move_count += 1
                xy_commanded_distance += segment_xy_distance

            prev_x, prev_y = x, y
            prev_flag = flag
            move_count += 1
            print(f"[MOVE {idx}] Completed. Position: X={x_hmc.current_x:.1f}, Y={y_hmc.current_y:.1f}, Z={z_hmc.z_current_position:.1f}")

    finally:
        app.patterning_active = False
        if disable_ramps:
            try:
                x_hmc.acceleration_and_deceleration(True, True)
                y_hmc.acceleration_and_deceleration(True, True)
                print("[INFO] Acceleration/deceleration ramps restored.")
            except Exception as e:
                print(f"[WARN] Failed to restore ramps: {e}")
        # Re-couple Z axis controller to App
        app.z_hmc = z_hmc
        app.hmcControl = z_hmc
        feedback_stop.set()
        sampler.join(timeout=0.1)
        if sampler.is_alive():
            try:
                smu.close()
            except Exception:
                pass
            sampler.join(timeout=1.0)
        else:
            try:
                smumark2.out_off(smu)
            except Exception:
                pass
            try:
                smu.close()
            except Exception:
                pass
        z_worker.join(timeout=2)
        set_fast_motion(False, False)

    pattern_elapsed = time.perf_counter() - pattern_start_time
    print(f"[TIMER] Patterning completed in {main.format_elapsed_time(pattern_elapsed)}.")
    print(f"[TIMER] X/Y movement time: {main.format_elapsed_time(xy_movement_time)} across {xy_move_count} moves.")
    if xy_move_count:
        estimated_xy_travel = xy_commanded_distance / speed if speed else 0
        print(f"[TIMER] Commanded XY distance: {xy_commanded_distance:.2f} µm")
        print(f"[TIMER] Estimated pure XY travel at {speed} µm/s: {main.format_elapsed_time(estimated_xy_travel)}")
        print(f"[TIMER] Average XY distance per move: {xy_commanded_distance / xy_move_count:.2f} µm")
    main.print_timing_summary(timers, move_count, xy_move_count)

    print("[INFO] Threaded straight-line feedback run complete.")
    print("Final Z Position:", z_hmc.z_current_position)
    startup_all_fn()
