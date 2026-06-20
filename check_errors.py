import pyvisa

def check_errors():
    rm = pyvisa.ResourceManager()
    smu_address = 'USB0::0x2184::0x007D::gey853397::INSTR'
    try:
        smu = rm.open_resource(smu_address)
        smu.timeout = 2000
        smu.write_termination = '\n'
        smu.read_termination = '\n'
        
        print("Reading SMU Error Queue...")
        errors = []
        while True:
            err = smu.query(":SYST:ERR?").strip()
            errors.append(err)
            if "No error" in err or err.startswith("0,"):
                break
        print("Errors in queue:")
        for e in errors:
            print(f"  {e}")
            
        smu.close()
    except Exception as e:
        print(f"Error communicating with SMU: {e}")

if __name__ == "__main__":
    check_errors()
