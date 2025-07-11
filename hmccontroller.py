import threading
from threading import Thread
import time
import serial 
import serial.tools.list_ports  # To get available serial ports
from tkinter import *
from tkinter import ttk


class App(ttk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.command = ""
        # self.create_widgets()
        self.x_value = 0
        self.y_value =0
        self.z_value=0
        self.success = False

    def stop_process(self):
        # time.sleep(0.1)
        self.hmcControl.stop_thread = True
        self.stop = True

    def check_thread_is_alive(self):
        if self.hmcControl.run_thread != None:
            if self.hmcControl.run_thread.is_alive():
                print("Error,A movement already in progress!")
                return True
        return False
    
    def start_thread(self):
        self.hmcControl.run_thread = Thread(target=self.start_process)
        self.hmcControl.run_thread.start()

    def start_process(self):
        try:
            self.hmcControl.stop_thread = False
            self.hmcControl.force_stop_thread = False
            self.stop = False

            if self.hmcControl.connect:
                
                match self.command:
                    case "1":
        
                        self.hmcControl.move(self.x_value, self.y_value, -self.z_value)

                    case "2":

                        self.hmcControl.contact_point_move(self.x_value,self.y_value,self.z_value)
                
            else:
                # self.update_status("Done")
                print("exception detected")
        except Exception as e:
            print(f"[EXCEPTION] {e}")
            hmc.stop_thread = True
            if hmc.run_thread and hmc.run_thread.is_alive():
                hmc.run_thread.join()    

class HmcControlCs:
    def __init__(self):
        #GENERAL STATE
        self.dummy = False
        self.run_thread = None
        self.stop_thread = False
        self.force_stop_thread = False
        self.indata = 0                      #holds last byte read from serial port

        #SPEED CONFIG
        self.spd_min = 10       #in steps/sec
        self.spd_max = 50000    #in steps/sec

        #MOVEMENT TRACKING
        self.x_moving = 0       #Store how much distance (in microns) the system moved in the most recent command. Calculated from step count × resolution.
        self.y_moving = 0
        self.z_moving = 0

        self.com_open = False       #indicate whether the port is open nd ready for communication

        #MOTION RESOLUTIPN AND LIMITS
        self.maximum_steps = 16777216
        self.Resolution_A = 0.2  #microns per step
        self.Resolution_B = 0.2
        self.Resolution_C = 0.2

        #DEVICE ID AND COMMUNICATION
        self.device = 100      #command byte sent to ask the device to identify itself
        self.device_ack = 200   #Expected reply from the device, proving communication is OK
        self.ok = 10           #Expected after successfull command

        #LIMIT SWITCH CODED
        self.x_home_limit = 41  #this is the response sent back by the device when the respective limit is reached
        self.x_far_limit = 40
        self.y_home_limit = 43
        self.y_far_limit = 42
        self.z_home_limit = 45
        self.z_far_limit = 44

        self.move_completed = 170 #response from the device


        #MOVEMENT COMMANDS
        self.speed = 34
        self.z_speed = 34

        self.Move = 19
        self.ACCELERATION = 20
        self.DECELERATION = 30
        self.Dir_plus = 125         #command to move in positive direction
        self.Dir_minus = 175

        self.x_current_position = 0     #absolute current position in microns
        self.y_current_position = 0
        self.z_current_position = 0

        self.stop = 104
        self.stop_ack = 105

        self.READ = 163
        self.READ_ACK = 164

        # LIMIT SWITCH FLAGS
        self.x_home_reached = False
        self.x_far_reached = False
        self.y_home_reached = False
        self.y_far_reached = False
        self.z_home_reached = False
        self.z_far_reached = False

        #set current to zero
        self.current_x = 0
        self.current_y = 0
        self.current_z =0

        self.serial_lock = threading.Lock()
 
    def on_startup(self):
        print("Sending all axes to home position...")
        self.set_speed(5000, 5000, 5000)

        print("Sending Z to home")
        self.ser.close()
        self.ser.open()
        # self.move_home(0,0,-self.maximum_steps)
        self.move_home(0,0,self.maximum_steps)
        self.ser.close()
        self.ser.open()
        
        
        print("Sending X to Home")
        self.move_home(-self.maximum_steps,0,0),
        
        self.ser.close()
        self.ser.open()
        print("Sending Y to home")
        self.move_home(0,-self.maximum_steps,0)
        
        
        print("Homing complete.")
        self.ser.close()
        print("port closed")
        self.ser.open()
        print("port opend")
        self.initialise_current_position()
      

    def config_serial_port(self, port):  # Returns true if connection is successful, else false
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=19200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1,
                write_timeout=0.1  # Added write timeout for consistency
            )

            if self.ser.is_open:
                self.ser.close()
            self.ser.open()

            print("Serial port initialized")
            self.com_open = True

            return  True
        except serial.SerialException as e:
            print("Serial port not initialized")
            print(f"Error: {e}")
            self.com_open = False
            return False

    def write_port(self, value):  #Sends a single byte command to the serial port
        out = [value]
        #self.ser.write(out)
        with self.serial_lock:
            self.ser.write(out)
    def read_ack(self, ack): #Waits for the controller to send acknowledgement byte and compares it with expected ack, if mismatched, raises an exception
        In = 0
        while True:
            r = self.ser.read(1)
            #print(str(r))
            if r == b'':
                continue
            In = ord(r)
            break
        if In != ack:
            raise Exception('Invalid Ack ' + str(In) + ' given ack ' + str(ack))
    # def read_ack(self,expected_ack, retries=5):
    #     for attempt in range(retries):
    #         In = self.ser.read(1)
    #         if not In:
    #             print(f"[WARN] Timeout waiting for ACK (attempt {attempt + 1})")
    #             continue
    #         if In == bytes([expected_ack]):
    #             return
    #         else:
    #             print(f"[WARN] Invalid Ack {In} (int={ord(In)}) on attempt {attempt + 1}, expected {expected_ack}")
    #             time.sleep(0.2)
    #             self.ser.reset_input_buffer()  # Flush junk
    #     raise Exception('Invalid Ack ' + str(In) + ' given ack ' + str(expected_ack))
       

    def read_value(self): #reads a single byte from serial port, loops until a valid byte or stop_thread is detected, stores value in self.indata and returns it
        #Used for movement status and limit detection
        self.indata = 0
        while True and not self.stop_thread:
            r = self.ser.read(1)
            #print(str(r))
            if r == b'':
                continue
            self.indata = ord(r)
            return self.indata

    def axis_read_value(self):
        #same as read_value but breaks off early if force_stop_thread is detected
        self.indata = 0
        while True and not self.force_stop_thread:
            r = self.ser.read(1)
            #print(str(r))
            if r == b'':
                continue
            self.indata = ord(r)
            return self.indata


    def msb_csb_lsb(self, value): #splits a number into 3 bytes, most significant byte, center, least
        #used when 3 byte commands are required eg setting speed
        v = abs(value)
        outlsb = int((v % 65536) % 256)
        outcsb = int((v % 65536) / 256)
        outmsb = int(v / 65536)
        return [outmsb, outcsb, outlsb]

    def byte3_byte2_byte1_byte0(self,value): #splits the command into 4 bytes for 32 bit transfer
        #used in movement commands to send steps
        v = abs(int(value))
        byte0 = v % 256
        byte1 = (v % 65536) // 256
        byte2 = (v % 16777216) // 65536
        byte3 = v // 16777216
        return [byte3, byte2, byte1, byte0]

    def msb_lsb(self, value): #splits into 2 bytes, for 16 bit transfer
        v = abs(value)
        outlsb = int(v % 256)
        outmsb = int(v / 256)
        return [outmsb, outlsb]

    @property
    def connect(self): #Verifies that serial connection is working, returns true if connected, othwerwise error
        if self.dummy:
            time.sleep(0.1)
            return True

        if self.com_open == False:
            raise Exception('Com port open failed')

        out = [self.device]
        self.ser.write(out)
        In = 0
        for x in range(20):
            r = self.ser.read(1)
            #print(str(r))
            if r == b'':
                continue  # Skip empty reads
            In = ord(r)  # Only call ord() if r is not empty
            if In == self.device_ack:
                return True

        raise Exception('Device Not Connected ' + str(In))


    def move(self, move_a, move_b, move_c): #moves by given distances in microns

        # self.ser.close()
        # self.ser.open()
        if self.dummy:
            time.sleep(0.1)
            return

        self.x_home_reached = False
        self.x_far_reached = False
        self.y_home_reached = False
        self.y_far_reached = False
        self.z_home_reached = False
        self.z_far_reached = False

        self.movement_completed = False
        if ((move_a>0) and (50000- self.current_x- move_a)<0):
            print("Value exceeds possible limit for X, enter valid value")
            return
        elif((move_b>0) and (50000- self.current_y-move_b)<0):
            print("Value exceeds possible limit for Y, enter valid value")
            return
        #elif((move_c>0) and (50000- self.z_current_position-move_c)<0):
        elif((move_c>0) and (self.current_z- move_c)<0):
            print(f"hmc z_current_position: {self.z_current_position}")
            print(f"hmc current_z={self.current_z}")
            print(f"hmc move_c={move_c}")
            print(50000-self.current_z-move_c)
            print("Value exceeds possible limit for -Z, enter valid value")
            return
        
        elif((move_a<0) and(self.current_x + move_a)<0):
            print("Value exceeds possible limit for -X, enter valid value")
            return
        elif((move_b<0)and(self.current_y +move_b)<0):
            print("Value exceeds possible limit for -Y, enter valid value")
            return
        #elif((move_c<0)and(self.current_z +move_c)<0):
        elif((move_c<0)and(self.current_z +abs(move_c) - 50000> 0)):
            print("Value exceeds possible limit for +Z, enter valid value")
            return
        
        self.set_move_data(move_a, move_b, move_c) #sends these distances to controller

        #self.acceleration_and_deceleration(acc_enable, dec_enable)

        self.indata = 0
        while self.indata == 0 and not self.stop_thread:
            self.indata = self.move_status()      #waits for movement status

        #Handling limit switches or stop requests
        if self.indata == 0 and self.stop_thread:   
            print("Stop Enabled")
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)

        if self.indata == self.x_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("X home limit reached")
            self.x_home_reached = True

        if self.indata == self.x_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("X far limit reached")
            self.x_far_reached = True

        if self.indata == self.y_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Y home limit reached")
            self.y_home_reached = True

        if self.indata == self.y_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Y far limit reached")
            self.y_far_reached = True

        if self.indata == self.z_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Z home limit reached")
            self.z_home_reached = True

        if self.indata == self.z_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Z far limit reached")
            self.z_far_reached = True


        if self.indata == self.move_completed:
            self.movement_completed = True

        
        self.read_move_bytes()

        #---------------------------
        
        #Updates internal positions
        if self.indata == self.x_home_limit:
            self.x_current_position = 0
            #print(f"...1{self.x_current_position}")
        else:
            if move_a > 0:
                self.x_current_position = self.x_current_position + self.x_moving
                #print(f"...2{self.x_current_position}")
            else:
                self.x_current_position = self.x_current_position - self.x_moving
                #print(f"...3{self.x_current_position}")
        if self.indata == self.y_home_limit:
            self.y_current_position = 0
        else:
            if move_b > 0:
                self.y_current_position = self.y_current_position + self.y_moving
            else:
                self.y_current_position = self.y_current_position- self.y_moving

        if self.indata == self.z_home_limit:
            self.z_current_position = 0
        else:
            if move_c > 0:
                self.z_current_position = self.z_current_position+ self.z_moving
            else:
                self.z_current_position = self.z_current_position- self.z_moving

        # print(f"indata: {self.indata}")
        # print(f"z_moving: {self.z_moving}")
        print(f"z_current_position: {self.z_current_position}")
        self.update_current_position(move_a, move_b , move_c)
        
    def acceleration_and_deceleration(self, acc_enable: bool, dec_enable: bool) -> bool:
        if acc_enable:
            self.write_port(self.ACCELERATION)
        else:
            self.write_port(0)

        self.read_ack(self.ok)

        if dec_enable:
            self.write_port(self.DECELERATION)
        else:
            self.write_port(0)

        self.read_ack(self.ok)

        return True

    def move_status(self): #fetches the status of last move
        self.read_value()

        return self.indata

    def set_move_data(self, A_steps, B_steps, C_steps): #converts user input in microns into steps and send them axis by axis
        if self.dummy:
            time.sleep(0.1)
            return

        self.write_port(self.Move)
        self.read_ack(self.ok)
        
        steps = (16777216) if abs(A_steps) == 16777216 else int(A_steps / self.Resolution_A)
        self.send_axis_data(steps,A_steps)
        self.move_x = steps
        #print(f"x_steps:{self.move_x} ")
        
        steps = (16777216) if abs(B_steps) == 16777216 else int(B_steps / self.Resolution_B)
        self.send_axis_data(steps,B_steps)
        self.move_y = steps
        
        steps = (16777216) if abs(C_steps) == 16777216 else int(C_steps / self.Resolution_C)
        self.send_axis_data(steps,C_steps)
        self.move_z = steps

        return

    def send_axis_data(self, data, direction): #sends step size and direction for one axis

        datas = self.byte3_byte2_byte1_byte0(abs(data)) #self.msb_csb_lsb(abs(data))

        self.write_port(datas[0])
        self.read_ack(self.ok)
        
        self.write_port(datas[1])
        self.read_ack(self.ok)

        self.write_port(datas[2])
        self.read_ack(self.ok)

        self.write_port(datas[3])
        self.read_ack(self.ok)

        if direction < 0:
            self.write_port(self.Dir_minus)
        else:
            self.write_port(self.Dir_plus)
        

        self.read_ack(self.ok)

    def read_move_bytes(self): #asks controller to send actual number of steps moved on each axis, calculate and stores how far each axis has moved
        # self.ser.reset_input_buffer()
        self.write_port(self.READ)
        self.read_ack(self.READ_ACK)


        self.x_moving = self.Read_axis_data()
        #self.x_moving = abs(self.move_x - self.x_moving)
        self.x_moving = self.x_moving
        self.x_moving *= self.Resolution_A

        self.y_moving = self.Read_axis_data()
        #self.y_moving = abs(self.move_y - self.y_moving)
        
        self.y_moving *= self.Resolution_B

        self.z_moving = self.Read_axis_data()
        #print(f"Z moving from read move byte : {self.z_moving}")
        #print(f"move_z = {self.move_z}")
        self.z_moving = abs(self.move_z - self.z_moving)
        self.z_moving *= self.Resolution_C


    def Read_axis_data(self): #reads 4 bytes for one axis and combine them into a full step value
        self.write_port(self.ok)
        self.axis_read_value()
        byte0 = self.indata
        #print(f"bytr 0 : {byte0}")

        self.write_port(self.ok)
        self.axis_read_value()
        byte1 = self.indata
        #print(f"bytr 1 : {byte0}")

        self.write_port(self.ok)
        self.axis_read_value()
        byte2 = self.indata
        #print(f"bytr 2 : {byte0}")

        self.write_port(self.ok)
        self.axis_read_value()
        byte3 = self.indata
        #print(f"bytr 3 : {byte0}")
        # (byte2 * 65535) + (byte1 * 255) + byte0
        steps = (byte3 * 16777216) + (byte2 * 65536) + (byte1 * 256) + byte0
        return steps


    def set_speed(self, x_value, y_value, z_value): #sends speed settings to the controller in steps/sec, convert microns/sec to step/sec
        print(f"[DEBUG] z_speed_steps = {z_value}")

        if self.dummy:
            time.sleep(0.1)
            return

        x_value = int(float(x_value) / self.Resolution_A)
        y_value = int(float(y_value) / self.Resolution_B)
        z_value = int(float(z_value) / self.Resolution_C)

        #xy_speed = xy_speed * 120
        self.write_port(self.speed)
        self.read_ack(self.ok)

        datas = self.msb_csb_lsb(abs(x_value))
        self.write_port(datas[0])
        self.read_ack(self.ok)

        self.write_port(datas[1])
        self.read_ack(self.ok)

        self.write_port(datas[2])
        self.read_ack(self.ok)

        datas = self.msb_csb_lsb(abs(y_value))
        self.write_port(datas[0])
        self.read_ack(self.ok)

        self.write_port(datas[1])
        self.read_ack(self.ok)

        self.write_port(datas[2])
        self.read_ack(self.ok)

        datas = self.msb_csb_lsb(abs(z_value))
        self.write_port(datas[0])
        self.read_ack(self.ok)

        self.write_port(datas[1])
        self.read_ack(self.ok)

        self.write_port(datas[2])
        self.read_ack(self.ok)

    def get_position(self):
        return {
            # "x": self.x_current_position,
            # "y": self.y_current_position,
            # "z": self.z_current_position
            "x" : self.current_x,
            "y" : self.current_y,
            "z" : self.current_z
    }

    def initialise_current_position(self):
        self.current_x =0
        self.current_y=0
        self.current_z=0
    def update_current_position(self,x,y,z):
        z= -z
        self.current_x += x
        self.current_y += y
        self.current_z += z
        self.z_current_position = self.current_z

    def safe_move(self, x, y, z):
        try:
            self.set_speed(5000, 5000, 5000)
            self.move(x, y, z)
        except Exception as e:
            print(f"[ERROR] Move error: {e}")
        # Check if the controller is in a bad state
            print("[INFO] Resetting serial port...")
            self.ser.close()
            #time.sleep(0.2)
            self.ser.open()
            self.ser.reset_input_buffer()
            raise  # re-raise so caller knows move failed
    

    def move_home(self, move_a, move_b, move_c):

        # self.ser.close()
        # self.ser.open()
        if self.dummy:
            time.sleep(0.1)
            return

        self.x_home_reached = False
        self.x_far_reached = False
        self.y_home_reached = False
        self.y_far_reached = False
        self.z_home_reached = False
        self.z_far_reached = False

        self.movement_completed = False
        
        self.set_move_data(move_a, move_b, move_c) #sends these distances to controller

        #self.acceleration_and_deceleration(acc_enable, dec_enable)

        self.indata = 0
        while self.indata == 0 and not self.stop_thread:
            self.indata = self.move_status()      #waits for movement status

        #Handling limit switches or stop requests
        if self.indata == 0 and self.stop_thread:   
            print("Stop Enabled")
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)

        if self.indata == self.x_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("X home limit reached")
            self.x_home_reached = True

        if self.indata == self.x_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("X far limit reached")
            self.x_far_reached = True

        if self.indata == self.y_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Y home limit reached")
            self.y_home_reached = True

        if self.indata == self.y_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Y far limit reached")
            self.y_far_reached = True

        if self.indata == self.z_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Z home limit reached")
            self.z_home_reached = True

        if self.indata == self.z_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Z far limit reached")
            self.z_far_reached = True


        if self.indata == self.move_completed:
            self.movement_completed = True

        
        self.read_move_bytes()

        #---------------------------
        

        #print(f"indata: {self.indata}")
        #print(f"x_moving: {self.x_moving}")
        #print(f"x_current_position: {self.x_current_position}")
        self.update_current_position(move_a, move_b , move_c)


    def contact_point_move(self, move_x, move_y, move_z):

        if self.dummy:
            time.sleep(0.1)
            return

        self.x_home_reached = False
        self.x_far_reached = False
        self.y_home_reached = False
        self.y_far_reached = False
        self.z_home_reached = False
        self.z_far_reached = False

        self.movement_completed = False
        
        self.set_move_data(move_x, move_y, move_z) #sends these distances to controller

        self.indata = 0
        while self.indata == 0 and not self.stop_thread:
            self.indata = self.move_status()      #waits for movement status

        #Handling limit switches or stop requests
        if self.indata == 0 and self.stop_thread:   
            print("Stop Enabled")
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)

        if self.indata == self.x_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("X home limit reached")
            self.x_home_reached = True

        if self.indata == self.x_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("X far limit reached")
            self.x_far_reached = True

        if self.indata == self.y_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Y home limit reached ")
            self.y_home_reached = True

        if self.indata == self.y_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Y far limit reached")
            self.y_far_reached = True

        if self.indata == self.z_home_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Z home limit reached")
            self.z_home_reached = True

        if self.indata == self.z_far_limit:
            self.write_port(self.stop)
            self.read_ack(self.stop_ack)
            print("Z far limit reached")
            self.z_far_reached = True


        if self.indata == self.move_completed:
            self.movement_completed = True

        self.update_current_position(move_x,move_y,move_z)
    

if __name__ == "__main__":
    hmc = HmcControlCs()
    if hmc.config_serial_port("COM3"):
        print("Connected!")

hmc = HmcControlCs()
root = Tk()
root.withdraw()  # Hide the GUI
app = App(master=root)
app.hmcControl = hmc