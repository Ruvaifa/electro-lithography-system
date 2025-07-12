import serial.tools.list_ports
from hmccontroller import HmcControlCs, App
import time
import threading
from threading import Thread
import math
import queue
import smumark1
import smumark2

# hmc = HmcControlCs()
# app = App()
# app.hmcControl=hmc

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
    hmc = HmcControlCs()
    app = App()
    app.hmcControl=hmc
    ports = list_ports()
    if not ports:
        return

    index = int(input("Enter the number of the COM port to use: "))
    selected_port = ports[index].device

    
    if not hmc.config_serial_port(selected_port):
        print("Failed to connect.")
        return

    def start_process(x,y,z):
        try:
            hmc.set_speed(5000, 5000, 5000)
            hmc.move(x, y, z)
            print("Move completed.")
            print(f"Current position: {hmc.get_position()}")
        except Exception as e:
            print(f"[ERROR] Movement failed: {e}")
        return
    
    def start_thread(x,y,z):
        move_thread = Thread(target=start_process, args=(x,y,z))
        move_thread.start()
        return move_thread

    hmc.on_startup()
    response = int(input("Enter 1 to move specific distance, 2 to send to home, 3 to import data from text file, 4 to send probe:"))
    
    if response ==1:
        hmc.ser.close()
        time.sleep(0.2)
        hmc.ser.open()
        while True:
            x = float(input("Enter X movement (microns): "))
            y = float(input("Enter Y movement (microns): "))
            z = float(input("Enter Z movement (microns): "))
            
        
            # move_thread= start_thread(x,y,z)
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
            # print(f"X: {hmc.current_x} µm")
            # print(f"Y: {hmc.current_y} µm")
            # print(f"Z: {hmc.current_z} µm")
            print(f"X: {hmc.x_current_position} µm")
            print(f"Y: {hmc.y_current_position} µm")
            print(f"Z: {hmc.current_z} µm")

    elif response ==2:
        hmc.on_startup()
    elif response ==3:
        v = int(input("Enter velocity microns/sec:"))
        hmc.ser.close()
        hmc.ser.open()
        filename = input("Enter txt filename:")
        with open(f"{filename}.txt", "r") as file:
            i=1
            prev_x, prev_y, prev_z = 0, 0, 0

            for line in file:
                parts = line.strip().replace(',', ' ').split()
                if len(parts) == 3:
                    x, y, z = map(float, parts)

                    # Calculate relative movement
                    dx = x - prev_x
                    dy = y - prev_y
                    dz = z - prev_z
                    
                    theta = math.atan2(x- prev_x, y-prev_y)
                    sin_theta = math.sin(theta)
                    cos_theta = math.cos(theta)
                    vx = v*sin_theta
                    vy = v*cos_theta
                    hmc.set_speed(vx,vy,v)
                    prev_x, prev_y, prev_z = x, y, z  # Update for next step
                    
                    try:
                        app.command = '1'
                        app.x_value = dx
                        app.y_value = dy
                        app.z_value = dz

                        app.start_thread()

                        while app.hmcControl.run_thread.is_alive():
                            #print("Waiting for move to complete...")
                            time.sleep(0.2)

                        if app.hmcControl.run_thread:
                            app.hmcControl.run_thread.join()

                        print("Final Position:")
                        print(f"X: {hmc.x_current_position} µm")
                        print(f"Y: {hmc.y_current_position} µm")
                        print(f"Z: {hmc.z_current_position} µm")
                        print(f"{i} MOVE COMPLETED")

                    except Exception as e:
                        print(f"[ERROR] Move {i} failed: {e}")
                        # Reset the serial port in case of error
                        hmc.ser.close()
                        time.sleep(0.2)
                        print("serial port reset")
                        hmc.ser.open()
                        hmc.ser.reset_input_buffer()

                    i += 1
    elif response ==4:
        z_move = 30000
        init_speed = int(input("Enter initial speed of the probe:"))
     
       
  
        speed_queue = queue.Queue()

        Thread(target=user_input_listener, args=(speed_queue,), daemon=True).start()

        # Start motion loop
        Thread(target=z_probe_move, args=(hmc, app, z_move, init_speed, speed_queue)).start()
    elif response ==5:
        from threading import Event

        z_move = 40000
        init_speed = 5000

        pause_event = Event()
        resume_event = Event()
        resume_event.set()  # Initially allow movement

        hmc.set_speed(1, 1, init_speed)

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


                    current = hmc.z_current_position
                    remaining = z_move - current
                    print(f"[INFO] Resuming movement: remaining {remaining} µm")

                    hmc.set_speed(1, 1, init_speed)
                    app.command = '1'
                    app.x_value = 0
                    app.y_value = 0
                    app.z_value = remaining
                    app.start_thread()

                    resume_event.set()
                    pause_event.clear()

        # Start input listener
        Thread(target=input_listener, daemon=True).start()

        # Keep main thread alive until motion finishes
        while True:
            if not app.hmcControl.run_thread or not app.hmcControl.run_thread.is_alive():
                if not pause_event.is_set():
                    break
            time.sleep(0.2)

        print("✅ Final Z Position:", hmc.z_current_position)

    
    elif response == 6:
        #z_move = 40000
        init_speed = 1000

        def find_contact_point_custom(smu, lower_current_ua, liftoff_height):
            step_size = liftoff_height -50
            total_distance = hmc.z_current_position
            print(f"[Z PROBE] Starting with {step_size} µm step, then 1 µm steps.")
            app.command = '1'
            app.x_value = 0
            app.y_value = 0
            app.z_value = step_size
            app.start_thread()
            app.hmcControl.run_thread.join()
            total_distance += step_size

            while True:
                app.command = '1'
                app.x_value = 0
                app.y_value = 0
                app.z_value = 1
                app.start_thread()
                app.hmcControl.run_thread.join()
                total_distance += 1

                status = smumark2.check_current(smu, threshold_current_1=lower_current_ua * 1e-6)
                if status == 1:
                    print(f"[CONTACT] Contact at {total_distance} µm")
                    hmc.z_current_position = total_distance
                    return total_distance
                time.sleep(0.1)

        def plot_from_file(v, filename, z_contact_point, smu, delta_z, voltage_threshold_1, voltage_threshold_2, volt_source, curr_comp, liftoff_height):
            smumark2.use_case_2(smu, voltage=volt_source, compliance_current_ua=curr_comp)
            if hmc.ser and hmc.ser.is_open:
                hmc.ser.close()
            time.sleep(0.2)
            hmc.ser.open()
            prev_flag =0
            with open(f"{filename}.txt", "r") as file:
                i = 1
                prev_x, prev_y = 0, 0
                prev_z = z_contact_point
                liftoff =False
                for line in file:
                    parts = line.strip().replace(',', ' ').split()
                    if len(parts) >= 3:
                        #x, y, flag = map(float, parts)
                        x, y, flag = float(parts[0]), float(parts[1]), int(parts[2])
                        dx = x - prev_x
                        dy = y - prev_y
                        print(f"flag:{flag}")
                        if i == 1:
                            prev_x, prev_y=x,y
                            prev_z= z_contact_point
                            i += 1
                            z = z_contact_point
                            continue
                        elif flag ==1:
                            if prev_flag ==0:
                                print("[INFO] liftoff Initiated")
                                z = prev_z -liftoff_height
                                liftoff = True
                            else:
                                z = prev_z
                        elif flag ==0:
                            if liftoff:
                                print("[INFO] Finding new contact point...")
                                z = find_contact_point_custom(smu, 0.2, liftoff_height)
                                prev_z = z
                                liftoff = False
                            
                            else:
                                #feedback- adjusting Z axis by a small step value depending on the current through SMU
                                direction = smumark2.check_voltage(
                                smu,
                                threshold_voltage_1=voltage_threshold_1,
                                threshold_voltage_2=voltage_threshold_2
                                )
                                if direction == 2:
                                    print("[FEEDBACK] Voltage too low — going down") #up
                                    #z = prev_z + delta_z
                                    z = prev_z - delta_z
                                    print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                                elif direction == 3:
                                    print("[FEEDBACK] Voltage too high — going up")#down
                                    #z = prev_z - delta_z
                                    z = prev_z + delta_z
                                    print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                                else:
                                    print("[FEEDBACK] Voltage in range — Z aligned")
                                    z = prev_z
                        else:
                            direction = smumark2.check_voltage(
                                smu,
                                threshold_voltage_1=voltage_threshold_1,
                                threshold_voltage_2=voltage_threshold_2
                            )
                            if direction == 2:
                                print("[FEEDBACK] Voltage too low — going down") #up
                                #z = prev_z + delta_z
                                z = prev_z - delta_z
                                print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                            elif direction == 3:
                                print("[FEEDBACK] Voltage too high — going up")#down
                                #z = prev_z - delta_z
                                z = prev_z + delta_z
                                print(f"Prev z: {prev_z}, delta z: {delta_z}, new z: {z}")
                            else:
                                print("[FEEDBACK] Voltage in range — Z aligned")
                                z = prev_z

                        dz = z - prev_z
                        # finding relative veocity in x and y direction
                        theta = math.atan2(x - prev_x, y - prev_y)
                        sin_theta = math.sin(theta)
                        cos_theta = math.cos(theta)
                        vx = v * sin_theta
                        vy = v * cos_theta
                        hmc.set_speed(vx, vy, 1000)
                        if i ==1:
                            hmc.set_speed(5000,5000,5000)
                        prev_x, prev_y, prev_z = x, y, z

                        try:
                            app.command = '1'
                            app.x_value = dx
                            app.y_value = dy
                            app.z_value = dz
                            app.start_thread()

                            while app.hmcControl.run_thread.is_alive():
                                #print("Waiting for move to complete...")
                                time.sleep(0.2)

                            if app.hmcControl.run_thread:
                                app.hmcControl.run_thread.join()

                            print("Final Position:")
                            print(f"X: {hmc.current_x} µm")
                            print(f"Y: {hmc.current_y} µm")
                            print(f"Z: {hmc.current_z} µm")
                            print(f"{i} MOVE COMPLETED")

                        except Exception as e:
                            print(f"[ERROR] Move {i} failed: {e}")
                            hmc.ser.close()
                            time.sleep(0.2)
                            print("serial port reset")
                            hmc.ser.open()
                            hmc.ser.reset_input_buffer()
                        prev_flag = flag
                        i += 1
                smumark2.out_off(smu)

        def find_contact_point(initial_step_size, smu, lower_current_ua, step_queue):
            step_size = initial_step_size
            total_distance =0
            i = 0

            print("[INFO] Starting Z probing. You can enter new step sizes any time.")

            while True:
                # Check for updated step size
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
                #smumark2.reset_smu(smu)
                status = smumark2.check_current(
                    smu,
                    threshold_current_1=lower_current_ua * 1e-6
                )

                if status == 1:
                    print("[CONTACT] Probe contact confirmed by current.")
                    hmc.z_current_position = total_distance
                    return total_distance

                time.sleep(0.1)


        hmc.set_speed(1000, 1000, init_speed)

        curr = float(input("Enter current in micro amps (use case 1): "))
        volt = float(input("Enter volt in V (use case 1): "))
        volt_source = float(input("Enter source voltage (use_case_2): "))
        curr_comp = float(input("Enter compliance current in micro amps (use_case_2): "))
        voltage_threshold_1 = float(input("Enter voltage lower threshold for feedback (V): "))
        voltage_threshold_2 = float(input("Enter voltage upper threshold for feedback (V): "))
        delta_z = float(input("Enter Z adjustment step for plotting (in µm): "))
        z_contact_step = float(input("Enter Z adjustment step for finding contact point:"))
        speed = float(input("Enter speed for xy axis: "))
        liftoff_height = float(input("Enter liftoff height in µm: "))
        filename = input("Enter filename: ")

        
        smu = smumark2.init_smu()  #initialise the SMU
        smumark2.reset_smu(smu)
        smumark2.use_case_1(smu, current_ua=curr, compliance_voltage=volt) 

        #moves x,y axis to the starting coordinates
        with open(f"{filename}.txt", "r") as file:
            for line in file:
                parts = line.strip().replace(',', ' ').split()
                if len(parts) >= 3:
                    #first_x, first_y ,= map(float, parts)
                    first_x, first_y, flag = float(parts[0]), float(parts[1]), int(parts[2])
                    break
                
        #Move to first x, y (no Z movement)
        dx = first_x - hmc.current_x
        dy = first_y - hmc.current_y
        hmc.set_speed(5000, 5000, 5000)

        #setting the x and y value in App class of hmccontroller.py
        app.command = '1'
        app.x_value = dx
        app.y_value = dy
        app.z_value = 0  # No Z movement yet
        app.start_thread() #starting the movement in a thread

        while app.hmcControl.run_thread.is_alive():
            time.sleep(0.1)
        if app.hmcControl.run_thread:
            app.hmcControl.run_thread.join()
        
        hmc.set_speed(speed, speed, init_speed)
        #Now do Z contact

        step_queue = queue.Queue()
        Thread(target=step_size_listener, args=(step_queue,), daemon=True).start()

        #making z axis movements to find the contact point using current values 
        z_contact_point = find_contact_point(initial_step_size=z_contact_step,smu=smu,lower_current_ua=0.2,step_queue=step_queue)

        hmc.current_z = z_contact_point
        z_contact_point = hmc.z_current_position
        smumark2.reset_smu(smu)

        #after the contact point is being found, starting the plott
        plot_from_file(speed, filename, z_contact_point, smu, delta_z, voltage_threshold_1, voltage_threshold_2, volt_source, curr_comp, liftoff_height)

        print("✅ Final Z Position:", hmc.z_current_position)
        hmc.on_startup() #after the plot is completed, it sends all the axis to home position
    elif response == 7:
        hmc.set_speed(5000, 5000, 5000)  

        step_size = 0.2  # microns
        steps = 1000     # number of steps
        total_distance = step_size * steps

        # Move X axis
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
        # Move Y axis
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
        # Move Z axis
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
        # Move all XYZ axes together
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


if __name__ == "__main__":
    main()
