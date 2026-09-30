from Phidget22.Phidget import *
from Phidget22.Devices.Spatial import *
from Phidget22.LogLevel import *
from Phidget22.Devices.Log import *
import time
import traceback


class PhidgetHeading:
    def __init__(self, serial=None, data_interval=10):
        self.spatial = Spatial()
        self.serial = serial
        self.data_interval = data_interval

        self.heading = None
        self.timestamp = None

        # Attach handlers
        self.spatial.setOnAlgorithmDataHandler(self._onAlgorithmData)
        self.spatial.setOnAttachHandler(self._onAttach)
        self.spatial.setOnDetachHandler(self._onDetach)
        self.spatial.setOnErrorHandler(self._onError)

    # ================== Event Handlers ==================

    def _onAlgorithmData(self, spatial, quaternion, timestamp):
        euler = spatial.getEulerAngles()
        self.heading = euler.heading
        self.timestamp = timestamp / 1000.0  # seconds

    def _onAttach(self, spatial):
        print("PhidgetSpatial attached")

    def _onDetach(self, spatial):
        print("PhidgetSpatial detached")

    def _onError(self, spatial, code, description):
        print(f"Error {code}: {description}")

    # ================== Public API ==================

    def start(self):
        try:
            Log.enable(LogLevel.PHIDGET_LOG_INFO, "phidgetlog.log")

            if self.serial is not None:
                self.spatial.setDeviceSerialNumber(self.serial)

            self.spatial.openWaitForAttachment(5000)
            self.spatial.setDataInterval(self.data_interval)
            self.spatial.setHeatingEnabled(True)

        except Exception:
            traceback.print_exc()
            raise

    def get_heading(self, wait_for_data=True):
        """
        Returns the latest heading (degrees).
        If wait_for_data=True, blocks until first value is received.
        """
        if wait_for_data:
            while self.heading is None:
                time.sleep(0.01)

        return self.heading

    def stop(self):
        try:
            self.spatial.close()
        except Exception:
            pass
