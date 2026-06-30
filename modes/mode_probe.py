# modes/mode_probe.py
# ponytail: Z probe and pause/resume motion (Modes 4 and 5).

import time
import queue
from threading import Thread, Event

def user_input_listener(speed_queue):
    while True:
        try:
            user_input = input("Enter new Z speed in microns/sec: ")
            speed = float(user_input)
            speed_queue.put(speed)
        except ValueError:
            print("[WARN] Invalid input. Enter a number.")
        except (KeyboardInterrupt, EOFError):
            break

def z_probe_move(hmc, app, total_distance, init_speed, speed_queue):
    app.hmcControl = hmc
    hmc.set_speed(1, 1, init_speed)
    remaining = total_distance

    # Start initial move
    app.command = '1'
    app.x_value = 0
    app.y_value = 0
    app.z_value = remaining
    app.start_thread()

    while True:
        if not app.hmcControl.run_thread or not app.hmcControl.run_thread.is_alive():
            break

        if not speed_queue.empty():
            new_speed = speed_queue.get()
            print(f"[INFO] New speed input detected: {new_speed} µm/s")

            if app.hmcControl.run_thread and app.hmcControl.run_thread.is_alive():
                try:
                    app.stop_process()
                    print("[INFO] Movement stopped")
                    time.sleep(0.2)
                except Exception as e:
                    print(f"[WARN] Stop failed or already complete: {e}")

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

            if app.hmcControl.run_thread:
                app.hmcControl.run_thread.join()

            current = hmc.z_current_position
            remaining = total_distance - current
            print(f"[INFO] Resuming remaining distance: {remaining} µm")

            hmc.set_speed(1, 1, new_speed)
            app.command = '1'
            app.x_value = 0
            app.y_value = 0
            app.z_value = remaining
            app.start_thread()

        time.sleep(0.1)

def run_probe(app, z_hmc):
    z_move = 30000
    try:
        init_speed = int(input("Enter initial speed of the probe:"))
    except ValueError:
        print("Invalid speed.")
        return
     
    speed_queue = queue.Queue()
    Thread(target=user_input_listener, args=(speed_queue,), daemon=True).start()
    Thread(target=z_probe_move, args=(z_hmc, app, z_move, init_speed, speed_queue)).start()

def run_pause_resume(app, z_hmc):
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
            try:
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
            except (KeyboardInterrupt, EOFError):
                break

    Thread(target=input_listener, daemon=True).start()
