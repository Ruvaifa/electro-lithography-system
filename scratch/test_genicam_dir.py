try:
    from pypylon import pylon, genicam
    print("Imported pylon and genicam successfully.")
    
    print("\n--- genicam members containing 'Ptr' or 'Enum' ---")
    for name in dir(genicam):
        if "Ptr" in name or "Enum" in name or "Float" in name:
            print(f"genicam.{name}")
            
    print("\n--- pylon members containing 'Ptr' or 'Enum' ---")
    for name in dir(pylon):
        if "Ptr" in name or "Enum" in name or "Float" in name:
            print(f"pylon.{name}")
            
except Exception as e:
    print("Failed to inspect:", e)
