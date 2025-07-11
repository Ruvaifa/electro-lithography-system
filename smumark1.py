import pyvisa
import time

def init_smu(current_ua=10, compliance_voltage=1.0):
    rm = pyvisa.ResourceManager()
    smu_address = 'USB0::0x2184::0x007D::gey853397::INSTR'  # Replace with your address
    smu = rm.open_resource(smu_address)
    smu.timeout = 5000
    smu.write_termination = '\n'
    smu.read_termination = '\n'

    smu.write('*CLS')
    smu.write('*RST')
    print("[INFO] SMU Reset complete.")

    smu.write("SOUR:FUNC CURR")
    smu.write(f"SOUR:CURR {current_ua * 1e-6}")
    smu.write(f"SENS:VOLT:PROT {compliance_voltage}")
    smu.write('OUTP ON')
    print(f"[INFO] Current set to {current_ua} µA, Compliance Voltage: {compliance_voltage} V")

    return smu

def check_voltage(smu, threshold=0.9):
    try:
        response = smu.query('READ?').strip()
        voltage, current = map(float, response.split(',')[:2])
        print(f"[VOLTAGE] {voltage:.4f} V, [CURRENT] {current:.4e} A")
        if voltage < threshold:
            print(" Voltage OUT OF RANGE!")
            smu.write('OUTP OFF')
            return 0
        return 1
    except Exception as e:
        print(f"[ERROR] SMU read error: {e}")
        smu.write('OUTP OFF')
        return 0
