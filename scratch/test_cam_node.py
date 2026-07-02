import sys
import time
try:
    from pypylon import pylon
    print("pypylon imported successfully.")
except ImportError as e:
    print("pypylon import failed:", e)
    sys.exit(1)

try:
    factory = pylon.TlFactory.GetInstance()
    devices = factory.EnumerateDevices()
    if not devices:
        print("No Basler camera found.")
        sys.exit(1)
    
    camera = pylon.InstantCamera(factory.CreateDevice(devices[0]))
    camera.Open()
    print("Camera opened successfully.")
    
    # 1. Check getattr(camera, "ExposureAuto")
    attr_node = getattr(camera, "ExposureAuto", None)
    print("getattr(camera, 'ExposureAuto') returned:", attr_node)
    if attr_node:
        print("attr_node type:", type(attr_node))
        try:
            print("attr_node writable:", attr_node.GetAccessMode())
        except Exception as e:
            print("attr_node.GetAccessMode() failed:", e)
            
    # 2. Check NodeMap lookup
    node_map = camera.GetNodeMap()
    nodemap_node = node_map.GetNode("ExposureAuto")
    print("node_map.GetNode('ExposureAuto') returned:", nodemap_node)
    if nodemap_node:
        print("nodemap_node type:", type(nodemap_node))
        try:
            print("IsWritable(nodemap_node):", pylon.IsWritable(nodemap_node))
        except Exception as e:
            print("pylon.IsWritable failed:", e)

    # 3. Try to set ExposureAuto to Continuous
    print("\nAttempting to set ExposureAuto to Continuous...")
    
    # Try pattern A: camera.ExposureAuto.SetValue("Continuous")
    try:
        camera.ExposureAuto.SetValue("Continuous")
        print("Pattern A (camera.ExposureAuto.SetValue) succeeded!")
    except Exception as e:
        print("Pattern A failed:", e)

    # Try pattern B: camera.ExposureAuto.Value = "Continuous"
    try:
        camera.ExposureAuto.Value = "Continuous"
        print("Pattern B (camera.ExposureAuto.Value = ...) succeeded!")
    except Exception as e:
        print("Pattern B failed:", e)

    # Try pattern C: NodeMap SetValue
    try:
        if nodemap_node:
            nodemap_node.SetValue("Continuous")
            print("Pattern C (nodemap_node.SetValue) succeeded!")
    except Exception as e:
        print("Pattern C failed:", e)

    # Try pattern D: NodeMap SetIntValue
    try:
        if nodemap_node:
            nodemap_node.SetIntValue(nodemap_node.GetEntryByName("Continuous").GetValue())
            print("Pattern D (nodemap_node.SetIntValue) succeeded!")
    except Exception as e:
        print("Pattern D failed:", e)

    print("\nCurrent ExposureAuto value:", camera.ExposureAuto.GetValue() if hasattr(camera, "ExposureAuto") else "unknown")

    camera.Close()
except Exception as e:
    print("An exception occurred during execution:", e)
