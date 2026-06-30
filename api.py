import logging
import time
import serial.tools.list_ports
from typing import Optional, Dict, Any
from hmccontroller import HmcControlCs, App
from modes import mode_async_litho
import config

logger = logging.getLogger(__name__)

class LithographySystem:
    """Core api class for the electro-lithography system."""

    def __init__(self):
        self.x_hmc: Optional[HmcControlCs] = None
        self.y_hmc: Optional[HmcControlCs] = None
        self.z_hmc: Optional[HmcControlCs] = None
        self.app: Optional[App] = None
        self.connected = False

    def list_ports(self) -> list[dict]:
        """Return available COM ports as list of dicts."""
        ports = serial.tools.list_ports.comports()
        return [{"device": p.device, "description": p.description} for p in ports]

    def connect(self, x_port: str, y_port: str, z_port: str) -> dict:
        """Connect to all three axes. Returns success status."""
        logger.info(f"Connecting to axes: X={x_port}, Y={y_port}, Z={z_port}")
        self.x_hmc = HmcControlCs(axis='x')
        self.y_hmc = HmcControlCs(axis='y')
        self.z_hmc = HmcControlCs(axis='z')

        if not self.x_hmc.config_serial_port(x_port):
            return {"success": False, "error": f"Failed to connect X axis on {x_port}"}
        if not self.y_hmc.config_serial_port(y_port):
            return {"success": False, "error": f"Failed to connect Y axis on {y_port}"}
        if not self.z_hmc.config_serial_port(z_port):
            return {"success": False, "error": f"Failed to connect Z axis on {z_port}"}

        self.app = App()
        self.app.x_hmc = self.x_hmc
        self.app.y_hmc = self.y_hmc
        self.app.z_hmc = self.z_hmc
        self.app.hmcControl = self.z_hmc

        self.connected = True
        return {"success": True}

    def home_all(self) -> dict:
        """Home all axes in startup sequence."""
        if not self.connected:
            return {"success": False, "error": "Not connected to system."}
        try:
            logger.info("Starting homing sequence...")
            self.z_hmc.on_startup()
            self.x_hmc.on_startup()
            self.y_hmc.on_startup()
            return {"success": True, "position": self.get_position()}
        except Exception as e:
            logger.error(f"Homing failed: {e}")
            return {"success": False, "error": str(e)}

    def move(self, x_um: float, y_um: float, z_um: float) -> dict:
        """Move logical Z distance (relative move)."""
        if not self.connected:
            return {"success": False, "error": "Not connected to system."}
        
        self.app.command = '1'
        self.app.x_value = x_um
        self.app.y_value = y_um
        self.app.z_value = z_um
        self.app.start_thread()

        while self.app.hmcControl.run_thread.is_alive():
            time.sleep(0.1)
        self.app.hmcControl.run_thread.join()

        return {"success": True, "position": self.get_position()}

    def get_position(self) -> dict:
        """Get absolute current position of all axes."""
        if not self.connected:
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        return {
            "x": self.x_hmc.x_current_position,
            "y": self.y_hmc.y_current_position,
            "z": self.z_hmc.z_current_position
        }

    def stop(self) -> dict:
        """Stop all axis movements immediately."""
        if not self.connected:
            return {"success": False, "error": "Not connected."}
        logger.warning("Emergency stop triggered.")
        self.app.stop_process()
        return {"success": True}

    def disconnect(self) -> dict:
        """Clean shutdown and disconnect serial ports."""
        logger.info("Disconnecting serial ports...")
        if self.x_hmc:
            self.x_hmc.disconnect()
        if self.y_hmc:
            self.y_hmc.disconnect()
        if self.z_hmc:
            self.z_hmc.disconnect()
        self.connected = False
        return {"success": True}
