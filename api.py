import logging
import time
import serial.tools.list_ports
from typing import Optional, Dict, Any
from threading import Thread
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

    def reset_abort_flags(self):
        """Reset emergency stop and thread abort flags for a clean movement run."""
        if self.connected:
            if self.app:
                self.app.stop = False
            if self.x_hmc:
                self.x_hmc.stop_thread = False
            if self.y_hmc:
                self.y_hmc.stop_thread = False
            if self.z_hmc:
                self.z_hmc.stop_thread = False

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
        self.reset_abort_flags()

        # Automatically home all axes on connection startup
        logger.info("Automatically homing all axes on connection...")
        home_res = self.home_all()
        if not home_res["success"]:
            # De-register connected flag if homing failed
            self.connected = False
            return home_res

        return {"success": True}

    def home_all(self) -> dict:
        """Home all axes in startup sequence."""
        if not self.connected:
            return {"success": False, "error": "Not connected to system."}
        self.reset_abort_flags()
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
        self.reset_abort_flags()
        
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
        # Set stop on App-level (covers hmcControl / z axis)
        self.app.stop_process()
        # Explicitly set stop flags on ALL individual axis controllers.
        # app.stop_process() only iterates app._get_controllers() which only
        # contains hmcControl (Z). X and Y are separate instances stored here.
        for h in [self.x_hmc, self.y_hmc, self.z_hmc]:
            if h:
                h.stop_thread = True
                h.force_stop_thread = True
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

    def run_pattern(self, mode_num: int, params: Dict[str, Any]) -> dict:
        """Starts a patterning run (Mode 6 or Mode 8) inside a non-blocking background thread."""
        if not self.connected:
            return {"success": False, "error": "Not connected to system."}
        
        if getattr(self.app, "patterning_active", False):
            return {"success": False, "error": "Patterning is already in progress."}

        self.reset_abort_flags()

        def reset_all_serial():
            self.x_hmc.config_serial_port(self.x_hmc.ser.port)
            self.y_hmc.config_serial_port(self.y_hmc.ser.port)
            self.z_hmc.config_serial_port(self.z_hmc.ser.port)

        def set_all_speed(vx, vy, vz):
            self.x_hmc.set_speed(vx, 0, 0)
            self.y_hmc.set_speed(0, vy, 0)
            self.z_hmc.set_speed(0, 0, vz)

        def startup_all():
            self.z_hmc.on_startup()
            self.x_hmc.on_startup()
            self.y_hmc.on_startup()

        # Set initial status flags
        self.app.patterning_active = True
        self.app.patterning_mode = mode_num
        self.app.total_moves = 0
        self.app.moves_done = 0
        self.app.moves_left = 0
        self.app.z_feedback_direction = "inactive"
        self.app.smu_voltage = 0.0
        self.app.smu_current = 0.0

        def worker():
            try:
                # Reset abort and stop flags inside the worker thread
                self.reset_abort_flags()
                
                # Automatically home all axes before starting pattern
                logger.info("Automatically homing all axes before starting pattern...")
                home_res = self.home_all()
                if not home_res["success"]:
                    logger.error(f"Homing failed before pattern start: {home_res.get('error')}")
                    return

                if mode_num == 6:
                    from modes import mode_sync_litho
                    mode_sync_litho.run(
                        self.app,
                        self.x_hmc,
                        self.y_hmc,
                        self.z_hmc,
                        reset_all_serial,
                        set_all_speed,
                        startup_all,
                        params=params
                    )
                elif mode_num == 8:
                    from modes import mode_async_litho
                    mode_async_litho.run(
                        self.app,
                        self.x_hmc,
                        self.y_hmc,
                        self.z_hmc,
                        reset_all_serial,
                        set_all_speed,
                        startup_all,
                        params=params
                    )
                else:
                    logger.error(f"Unsupported patterning mode: {mode_num}")
            except Exception as e:
                logger.error(f"Exception in patterning worker thread: {e}")
            finally:
                self.app.patterning_active = False

        t = Thread(target=worker, name="PatterningWorker", daemon=True)
        t.start()
        return {"success": True}

    def get_status(self) -> dict:
        """Get full system status, including axis positions and real-time patterning telemetry."""
        pos = self.get_position()
        
        status = {
            "connected": self.connected,
            "position": pos,
            "patterning": {
                "active": False,
                "mode": None,
                "total_moves": 0,
                "moves_done": 0,
                "moves_left": 0,
                "z_feedback_direction": "inactive",
                "smu_voltage": 0.0,
                "smu_current": 0.0
            }
        }
        
        if self.app is not None:
            status["patterning"] = {
                "active": getattr(self.app, "patterning_active", False),
                "mode": getattr(self.app, "patterning_mode", None),
                "total_moves": getattr(self.app, "total_moves", 0),
                "moves_done": getattr(self.app, "moves_done", 0),
                "moves_left": getattr(self.app, "moves_left", 0),
                "z_feedback_direction": getattr(self.app, "z_feedback_direction", "inactive"),
                "smu_voltage": getattr(self.app, "smu_voltage", 0.0),
                "smu_current": getattr(self.app, "smu_current", 0.0)
            }
            
        return status
