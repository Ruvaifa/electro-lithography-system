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
    smu.write(':SYST:FRES OFF')
    print("[INFO] SMU Reset complete (2-wire mode forced).")
    return smu

def reset_smu(smu):
    smu.write('*CLS')
    smu.write('*RST')
    smu.write(':SYST:FRES OFF')
    print("[INFO] SMU Reset complete (2-wire mode forced).")

def use_case_1(smu, voltage=1, compliance_current_ua=105): #set compliance current and source voltage
    reset_smu(smu)
    smu.write("SOUR:FUNC VOLT")
    smu.write(f"SOUR:VOLT {voltage}")
    smu.write(f"SENS:CURR:PROT {compliance_current_ua * 1e-6}")

    print(f"[INFO] Voltage set to {voltage} V, Compliance Current: {compliance_current_ua} micro A")
    return smu
def use_case_2(smu,voltage = 1,compliance_current_ua = 1000): #set complaince curr nd source volt
    reset_smu(smu)
    smu.write("SOUR:FUNC VOLT")
    smu.write(f"SOUR:VOLT {voltage}")
    smu.write(f"SENS:CURR:PROT {compliance_current_ua * 1e-6}")
    print(f"Voltage set as {voltage} V, Current set as {compliance_current_ua} micro A ")
    return smu

def out_on(smu):
    smu.write('OUTP ON')
    setattr(smu, "_output_enabled", True)

def _read_sample(smu, ensure_output_on=True):
    if ensure_output_on or not getattr(smu, "_output_enabled", False):
        smu.write("OUTP ON")
        setattr(smu, "_output_enabled", True)
    response = smu.query('READ?').strip()
    if not response:
        smu.write("OUTP ON")
        setattr(smu, "_output_enabled", True)
        time.sleep(0.02)
        response = smu.query('READ?').strip()
    voltage, current = map(float, response.split(',')[:2])
    return voltage, current

def read_voltage_sample(smu, ensure_output_on=True):
    return _read_sample(smu, ensure_output_on=ensure_output_on)

def read_current_sample(smu, ensure_output_on=True):
    return _read_sample(smu, ensure_output_on=ensure_output_on)


def check_voltage(smu, threshold_voltage_1, threshold_voltage_2, ensure_output_on=True, verbose=True):
    alpha = 0
    try:
        voltage, current = read_voltage_sample(smu, ensure_output_on=ensure_output_on)
        if verbose:
            print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
        if threshold_voltage_1 <voltage < threshold_voltage_2:
            if verbose:
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
        setattr(smu, "_output_enabled", False)
        return alpha

def check_current(smu, threshold_current_1, ensure_output_on=True, verbose=True):
    beta = 0
    voltage, current = read_current_sample(smu, ensure_output_on=ensure_output_on)
    if verbose:
        print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
    if abs(current) > threshold_current_1:
        if verbose:
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
    setattr(smu, "_output_enabled", False)


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
