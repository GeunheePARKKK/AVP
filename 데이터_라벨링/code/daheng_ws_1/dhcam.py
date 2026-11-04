"""Daheng Galaxy (MER2-240-159U3C) camera wrapper -> BGR numpy frames for OpenCV.

The Galaxy SDK ships its Python API as a plain source folder rather than a pip
package, so ``vendor/gxipy`` holds a patched copy (see vendor/README).
"""

import os
import sys

import cv2

_VENDOR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor")
if _VENDOR not in sys.path:
    sys.path.insert(0, _VENDOR)

import gxipy as gx  # noqa: E402


class DahengCamera:
    """Context manager that yields BGR frames from the first Galaxy device."""

    def __init__(self, sn=None, exposure_us=None, gain_db=None, auto=True,
                 auto_exposure_max_us=20000.0):
        self.sn = sn
        self.exposure_us = exposure_us
        self.gain_db = gain_db
        self.auto = auto
        # Left uncapped, auto-exposure settles around 70 ms in ordinary indoor light,
        # which caps acquisition at ~14 fps and smears any hand-held board. Capping it
        # and letting auto-gain make up the difference gives ~50 fps.
        self.auto_exposure_max_us = auto_exposure_max_us
        self._dm = None
        self._cam = None
        self._convert = None

    def __enter__(self):
        self._dm = gx.DeviceManager()
        count, dev_list = self._dm.update_device_list()
        if count == 0:
            raise RuntimeError("No Daheng camera found. Check the USB3 cable and that "
                               "GalaxyView is not holding the device open.")

        if self.sn:
            self._cam = self._dm.open_device_by_sn(self.sn)
        else:
            self._cam = self._dm.open_device_by_index(1)
        self.info = dev_list[0]

        rc = self._cam.get_remote_device_feature_control()

        # Full frame, free-running.
        _try(rc, "TriggerMode", "Off")
        _try(rc, "AcquisitionMode", "Continuous")

        if self.auto and self.exposure_us is None:
            _try(rc, "ExposureAuto", "Continuous")
            if self.auto_exposure_max_us:
                _try_float(rc, "AutoExposureTimeMax", self.auto_exposure_max_us)
        else:
            _try(rc, "ExposureAuto", "Off")
            if self.exposure_us is not None:
                _try_float(rc, "ExposureTime", self.exposure_us)

        if self.auto and self.gain_db is None:
            _try(rc, "GainAuto", "Continuous")
        else:
            _try(rc, "GainAuto", "Off")
            if self.gain_db is not None:
                _try_float(rc, "Gain", self.gain_db)

        # Color cameras: let the camera balance white so the board stays neutral.
        _try(rc, "BalanceWhiteAuto", "Continuous")

        self.width = _read_int(rc, "Width")
        self.height = _read_int(rc, "Height")
        self._rc = rc

        self._cam.stream_on()
        return self

    def read(self, timeout_ms=2000):
        """Return one frame as an HxWx3 uint8 BGR array (or HxW mono)."""
        raw = self._cam.data_stream[0].get_image(timeout_ms)
        if raw is None:
            return None
        rgb = raw.convert("RGB")
        if rgb is None:  # mono sensor
            arr = raw.get_numpy_array()
            return None if arr is None else arr.copy()
        arr = rgb.get_numpy_array()
        if arr is None:
            return None
        # cvtColor rather than arr[:, :, ::-1].copy() -- the reversed-stride copy
        # costs ~15 ms on a 2048x1200 frame, cvtColor ~2 ms.
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

    def exposure_gain(self):
        """Current (exposure_us, gain_db) -- auto modes settle a second or two in."""
        return (_read_float(self._rc, "ExposureTime"), _read_float(self._rc, "Gain"))

    def __exit__(self, *exc):
        try:
            if self._cam is not None:
                self._cam.stream_off()
                self._cam.close_device()
        finally:
            self._cam = None
            self._dm = None


def _try(rc, name, value):
    try:
        rc.get_enum_feature(name).set(value)
    except Exception:
        pass


def _try_float(rc, name, value):
    try:
        rc.get_float_feature(name).set(float(value))
    except Exception:
        pass


def _read_int(rc, name):
    try:
        return int(rc.get_int_feature(name).get())
    except Exception:
        return None


def _read_float(rc, name):
    try:
        return float(rc.get_float_feature(name).get())
    except Exception:
        return None


if __name__ == "__main__":
    with DahengCamera() as cam:
        print(f"{cam.info['model_name']} sn={cam.info['sn']} {cam.width}x{cam.height}")
        for _ in range(20):
            frame = cam.read()
        exp, gain = cam.exposure_gain()
        print(f"exposure {exp:.0f} us, gain {gain:.1f} dB")
        print("frame:", None if frame is None else (frame.shape, frame.dtype))
