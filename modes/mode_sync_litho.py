# modes/mode_sync_litho.py
# ponytail: synchronous lithography with Z feedback (Mode 6).

import time
import math
import queue
from threading import Thread
from collections import defaultdict
import main
import smumark2
import lithography

def run(app, x_hmc, y_hmc, z_hmc, reset_all_serial_fn, set_all_speed_fn, startup_all_fn):
    timers = defaultdict(float)

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
    liftoff_height = float(input("Enter liftoff height in µm: "))
    max_safe_z_margin = float(input("Enter maximum allowed Z increase above first contact point (in um, e.g. 100): "))
    filename = input("Enter filename: ")

    with lithography.time_block(timers, "smu.init_smu"):
        smu = smumark2.init_smu()
    with lithography.time_block(timers, "smu.reset_smu"):
        smumark2.reset_smu(smu)
    with lithography.time_block(timers, "smu.use_case_1"):
        smumark2.use_case_1(smu, voltage=contact_voltage, compliance_current_ua=contact_compliance_current_ua)
    smumark2.out_on(smu)

    # Reconnect serial ports to clear buffers before motion
    reset_all_serial_fn()

    for h in [x_hmc, y_hmc, z_hmc]:
        h.fast_mode = True

    try:
        # Load pattern points
        try:
            lines = lithography.read_pattern_file(filename)
        except Exception as e:
            print(f"[ERROR] Failed to read pattern file: {e}")
            return

        if not lines:
            print(f"[ERROR] No valid data found in {filename}.txt")
            return

        first_x, first_y, first_flag = lines[0]
        dx = first_x - x_hmc.current_x
        dy = first_y - y_hmc.current_y

        set_all_speed_fn(5000, 5000, 5000)

        app.command = '1'
        app.x_value = round(dx / 0.2) * 0.2
        app.y_value = round(dy / 0.2) * 0.2
        app.z_value = 0
        app.start_thread()

        if app.hmcControl.run_thread:
            app.hmcControl.run_thread.join()

        print("Moved to first coordinate, starting probe contact...")

        # Setup probe steps queue and thread for user input
        step_queue = queue.Queue()
        step_listener = Thread(target=main.step_size_listener, args=(step_queue,), daemon=True)
        step_listener.start()

        z_contact_point = lithography.find_contact_point(
            app=app,
            z_hmc=z_hmc,
            initial_step_size=z_contact_step,
            smu=smu,
            threshold_current_ua=threshold_current_ua,
            step_queue=step_queue,
            timers=timers,
            ensure_output_on=False,
            verbose=False
        )

        max_safe_z = z_contact_point + max_safe_z_margin
        print(f"[SAFE LIMIT] Max safe Z: {max_safe_z} µm (contact: {z_contact_point} µm)")

        if disable_ramps:
            with lithography.time_block(timers, "motion.disable_ramps"):
                x_hmc.acceleration_and_deceleration(False, False)
                y_hmc.acceleration_and_deceleration(False, False)
                z_hmc.acceleration_and_deceleration(False, False)

        def plot_from_file(v):
            with lithography.time_block(timers, "smu.use_case_2"):
                smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
            smumark2.out_on(smu)
            prev_flag = 0
            pattern_start_time = time.perf_counter()
            xy_movement_time = 0
            xy_move_count = 0
            xy_commanded_distance = 0.0
            move_count = 0
            print("[TIMER] Patterning timer started.")

            i = 1
            prev_z = z_contact_point
            liftoff = False

            for x, y, flag in lines:
                dx = x - x_hmc.current_x
                dy = y - y_hmc.current_y
                print(f"flag:{flag}")
                if i == 1:
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
                        z = lithography.find_contact_point_custom(
                            app=app,
                            z_hmc=z_hmc,
                            smu=smu,
                            contact_voltage=contact_voltage,
                            contact_compliance_current_ua=contact_compliance_current_ua,
                            threshold_current_ua=threshold_current_ua,
                            liftoff_height=liftoff_height,
                            max_safe_z=max_safe_z,
                            timers=timers
                        )
                        with lithography.time_block(timers, "smu.use_case_2"):
                            smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
                        prev_z = z
                        liftoff = False
                    else:
                        with lithography.time_block(timers, "smu.check_voltage"):
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
                    with lithography.time_block(timers, "smu.check_voltage"):
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

                lithography.ensure_z_below_limit(z_hmc, z, max_safe_z, smu, f"Pattern move {i}")
                dz = z - prev_z
                actual_dx = round(dx / 0.2) * 0.2
                actual_dy = round(dy / 0.2) * 0.2
                actual_dz = round(dz / 0.2) * 0.2

                if actual_dx == 0 and actual_dy == 0 and actual_dz == 0:
                    prev_flag = flag
                    i += 1
                    continue

                segment_xy_distance = math.hypot(actual_dx, actual_dy)
                vx, vy = main.compute_synchronized_speeds(actual_dx, actual_dy, v)

                with lithography.time_block(timers, "motion.set_speed"):
                    set_all_speed_fn(vx, vy, 1000)
                if i == 1:
                    with lithography.time_block(timers, "motion.set_speed"):
                        set_all_speed_fn(5000, 5000, 5000)
                prev_z = z

                try:
                    app.command = '1'
                    app.x_value = actual_dx
                    app.y_value = actual_dy
                    app.z_value = actual_dz
                    move_start_time = time.perf_counter()
                    with lithography.time_block(timers, "motion.start_thread"):
                        app.start_thread()
                    with lithography.time_block(timers, "motion.wait_for_completion"):
                        main.wait_for_motion(app)
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
                    reset_all_serial_fn()
                    for h in [x_hmc, y_hmc, z_hmc]:
                        h.ser.reset_input_buffer()
                prev_flag = flag
                i += 1

            try:
                smumark2.out_off(smu)
            except Exception:
                pass
            try:
                smu.close()
            except Exception:
                pass
            pattern_elapsed = time.perf_counter() - pattern_start_time
            print(f"[TIMER] Patterning completed in {main.format_elapsed_time(pattern_elapsed)}.")
            print(f"[TIMER] X/Y movement time: {main.format_elapsed_time(xy_movement_time)} across {xy_move_count} moves.")
            if xy_move_count:
                estimated_xy_travel = xy_commanded_distance / v if v else 0
                print(f"[TIMER] Commanded XY distance: {xy_commanded_distance:.2f} µm")
                print(f"[TIMER] Estimated pure XY travel at {v} µm/s: {main.format_elapsed_time(estimated_xy_travel)}")
                print(f"[TIMER] Average XY distance per move: {xy_commanded_distance / xy_move_count:.2f} µm")
            main.print_timing_summary(timers, move_count, xy_move_count)

        plot_from_file(speed)

    except lithography.PatternAbort as pa:
        print(f"[ABORT] Lithography run aborted: {pa}")
    finally:
        if disable_ramps:
            x_hmc.acceleration_and_deceleration(True, True)
            y_hmc.acceleration_and_deceleration(True, True)
            z_hmc.acceleration_and_deceleration(True, True)

        for h in [x_hmc, y_hmc, z_hmc]:
            h.fast_mode = False

        startup_all_fn()
