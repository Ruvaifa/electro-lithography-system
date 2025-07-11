import serial.tools.list_ports
from hmccontroller import HmcControlCs, App
import time
import threading
from threading import Thread
import math
import queue
import smumark2

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

    hmc.on_startup() #sends all axis to home
    response = int(input("Enter 1 to move specific distance, 2 to send to home, 3 to import data from text file, 4 to send probe, 6 to start plotting:"))
    
    if response ==1:
        hmc.ser.close()
        # time.sleep(0.2)
        # hmc.ser.open()
        # while True:
        #     x = float(input("Enter X movement (microns): "))
        #     y = float(input("Enter Y movement (microns): "))
        #     z = float(input("Enter Z movement (microns): "))
            
        
        #     # move_thread= start_thread(x,y,z)
        #     app.command ='1'
        #     app.x_value =x
        #     app.y_value=y
        #     app.z_value=z
        #     app.start_thread()
        #     while app.hmcControl.run_thread.is_alive():
        #         print("Waiting for move to complete...")
        #         time.sleep(0.2)
        #     if app.hmcControl.run_thread:
        #         app.hmcControl.run_thread.join()
        #     print("Final Position:")
        #     # print(f"X: {hmc.current_x} µm")
        #     # print(f"Y: {hmc.current_y} µm")
        #     # print(f"Z: {hmc.current_z} µm")
        #     print(f"X: {hmc.x_current_position} µm")
        #     print(f"Y: {hmc.y_current_position} µm")
        #     print(f"Z: {hmc.current_z} µm")

    
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