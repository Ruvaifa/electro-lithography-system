from pypylon import pylon, genicam

print("--- pylon functions/members ---")
for name in dir(pylon):
    if "Writable" in name or "Readable" in name or "Available" in name or "Is" in name:
        print(f"pylon.{name}")

print("\n--- genicam functions/members ---")
for name in dir(genicam):
    if "Writable" in name or "Readable" in name or "Available" in name or "Is" in name:
        print(f"genicam.{name}")
