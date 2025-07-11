import pyvisa
import time

def init_smu():
    rm = pyvisa.ResourceManager()
    smu_address = 'USB0::0x2184::0x007D::gey853397::INSTR'  # Replace with your address
    smu = rm.open_resource(smu_address)
    smu.timeout = 5000
    smu.write_termination = '\n'
    smu.read_termination = '\n'
    smu.write('*CLS')
    smu.write('*RST')
    print("[INFO] SMU Reset complete.")
    return smu

def reset_smu(smu):
    smu.write('*CLS')
    smu.write('*RST')
    print("[INFO] SMU Reset complete.")

def use_case_1(smu,current_ua = 105,compliance_voltage = 1): #set complaince volt and source curr
    reset_smu(smu)
    smu.write("SOUR:FUNC CURR")
    smu.write(f"SOUR:CURR {current_ua * 1e-6}")
    smu.write(f"SENS:VOLT:PROT {compliance_voltage}")

    print(f"[INFO] Current set to {current_ua} micro A, Compliance Voltage: {compliance_voltage} V")
    return smu
def use_case_2(smu,voltage = 1,compliance_current_ua = 1000): #set complaince curr nd source volt
    reset_smu(smu)
    smu.write("SOUR:FUNC VOLT")
    smu.write(f"SOUR:VOLT {voltage}")
    smu.write(f"SENS:CURR:PROT {compliance_current_ua * 1e-6}")
    print(f"Voltage set as {voltage} V, Current set as {compliance_current_ua} micro A ")
    return smu


def check_voltage(smu, threshold_voltage_1, threshold_voltage_2):
    alpha = 0
    try:
        smu.write("OUTP ON")
        response = smu.query('READ?').strip()
        voltage, current = map(float, response.split(',')[:2])
        print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
        if threshold_voltage_1 <voltage < threshold_voltage_2:
            print(" Voltage in the range!")
            #smu.write('OUTP OFF')
            alpha = 1
            return alpha
        elif voltage < threshold_voltage_1:
            #print('voltage below the band, please lift the probe a little up!!')
            alpha = 2
            return alpha
        elif voltage > threshold_voltage_2:
            #print('voltage above the band, please get the probe a little down!!')
            alpha = 3
            return alpha
        return alpha
    except Exception as e:
        print(f"[ERROR] SMU read error: {e}")
        smu.write('OUTP OFF')
        return alpha

def check_current(smu, threshold_current_1):
    beta = 0
    smu.write("OUTP ON")
    response = smu.query('READ?').strip()
    voltage, current = map(float, response.split(',')[:2])
    print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
    if  current > threshold_current_1:
        print(" current reached the right spot !!")
        beta = 1
        return beta
    return beta

def check_voltage_testing(smu,threshold_voltage = 0.9):
    gamma = 0
    response = smu.query('READ?').strip()
    voltage, current = map(float, response.split(',')[:2])
    print(f"[VOLTAGE]{voltage:.4f} V, [CURRENT] {current:.4e} A")
    if voltage > threshold_voltage:
        print(" no touch confirmed , make Z axis move down ")
        gamma = 1
        return gamma
    elif voltage < threshold_voltage:
        print(" no touch confirmed , make Z axis move up ")
        gamma = 2
        return gamma
    return gamma

def out_off(smu):
    smu.write('OUTP OFF')


# volt = float(input("Enter voltage:"))
# curr = float(input("Enter curr:"))
# smu = init_smu()
# use_case_1(smu, curr, volt)
# while True:
#     smu.write("OUTP ON")
#     response = smu.query('READ?').strip()
#     voltage, current = map(float, response.split(',')[:2])
#     print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
#     #time.sleep(0.1)


# import pyvisa
# import time

# def init_smu():
#     rm = pyvisa.ResourceManager()
#     smu_address = 'USB0::0x05E6::0x2450::04424906::INSTR'  # Replace with your actual VISA address
#     smu = rm.open_resource(smu_address)
#     smu.timeout = 5000
#     smu.write_termination = '\n'
#     smu.read_termination = '\n'

#     smu.write("*RST")               # Reset device
#     smu.write("*CLS")               # Clear errors
#     smu.write(":SYST:BEEP:STAT OFF")
#     print("[INFO] Keithley 2450 Reset complete.")
#     return smu

# def reset_smu(smu):
#     smu.write("*CLS")
#     smu.write("*RST")
#     print("[INFO] SMU Reset complete.")

# def use_case_1(smu, current_ua=105, compliance_voltage=1.0):
#     smu.write(":SOUR:FUNC CURR")
#     smu.write(f":SOUR:CURR {current_ua * 1e-6:.4e}")
#     smu.write(":SENS:FUNC 'VOLT','CURR'")
#     smu.write(f":SENS:VOLT:PROT {compliance_voltage}")
#     smu.write(":FORM:ELEM VOLT,CURR")
#     smu.write(":OUTP ON")
#     print(f"[INFO] Keithley 2450 sourcing {current_ua} µA with compliance {compliance_voltage} V")
#     return smu

# def use_case_2(smu, voltage=1.0, compliance_current_ua=1000):
#     smu.write(":SOUR:FUNC VOLT")
#     smu.write(f":SOUR:VOLT {voltage}")
#     smu.write(":SENS:FUNC 'VOLT','CURR'")
#     smu.write(f":SENS:CURR:PROT {compliance_current_ua * 1e-6:.4e}")
#     smu.write(":FORM:ELEM VOLT,CURR")
#     smu.write(":OUTP ON")
#     print(f"[INFO] Keithley 2450 sourcing {voltage} V with compliance {compliance_current_ua} µA")
#     return smu

# def check_voltage(smu, threshold_voltage_1, threshold_voltage_2):
#     try:
#         smu.write(":OUTP ON")
#         response = smu.query(":READ?").strip()
#         print(f"[DEBUG] Raw response: {response}")
#         parts = response.split(',')
#         if len(parts) >= 2:
#             voltage = float(parts[0])
#             current = float(parts[1])
#             print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
#             if threshold_voltage_1 < voltage < threshold_voltage_2:
#                 return 1
#             elif voltage < threshold_voltage_1:
#                 return 2
#             elif voltage > threshold_voltage_2:
#                 return 3
#         else:
#             print("[ERROR] Incomplete response from SMU.")
#             return 0
#     except Exception as e:
#         print(f"[ERROR] SMU read error: {e}")
#         smu.write(":OUTP OFF")
#         return 0

# def check_current(smu, threshold_current_1):
#     try:
#         smu.write(":OUTP ON")
#         response = smu.query(":READ?").strip()
#         print(f"[DEBUG] Raw response: {response}")
#         parts = response.split(',')
#         if len(parts) >= 2:
#             voltage = float(parts[0])
#             current = float(parts[1])
#             print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
#             return 1 if current > threshold_current_1 else 0
#         else:
#             print("[ERROR] Incomplete response from SMU.")
#             return 0
#     except Exception as e:
#         print(f"[ERROR] SMU read error: {e}")
#         return 0

# def check_voltage_testing(smu, threshold_voltage=0.9):
#     try:
#         response = smu.query(":READ?").strip()
#         print(f"[DEBUG] Raw response: {response}")
#         parts = response.split(',')
#         if len(parts) >= 2:
#             voltage = float(parts[0])
#             current = float(parts[1])
#             print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
#             return 1 if voltage > threshold_voltage else 2
#         else:
#             print("[ERROR] Incomplete response from SMU.")
#             return 0
#     except Exception as e:
#         print(f"[ERROR] SMU read error: {e}")
#         return 0

# def out_off(smu):
#     smu.write(":OUTP OFF")
#     print("[INFO] Output turned off.")

# volt = float(input("Enter voltage:"))
# curr = float(input("Enter curr:"))
# smu = init_smu()
# use_case_1(smu, curr, volt)
# while True:
#     smu.write(":OUTP ON")
#     response = smu.query(':READ?').strip()
#     # voltage, current = map(float, response.split(',')[:2])
#     # print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
#     print(response)
#     time.sleep(0.1)