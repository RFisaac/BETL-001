"""Read-only access to the reSpeaker Flex / XMOS XVF3800 control interface over USB.

Same vendor control-transfer protocol as Seeed's reSpeaker_Flex/python_control/xvf_host.py.
On Linux the user needs permission for the device: see udev/99-betl.rules.
"""

import math
import struct
import time

XVF_VID = 0x2886
CONTROL_SUCCESS = 0
SERVICER_COMMAND_RETRY = 64

# name: (resid, cmdid, count, type)
PARAMS = {
    "VERSION":             (48, 0, 3, "uint8"),
    "DOA_VALUE":           (20, 18, 2, "uint16"),   # [doa 0-359, speech 0/1]
    "AEC_AZIMUTH_VALUES":  (33, 75, 4, "float"),    # radians: beam1, beam2, free-running, auto-select
    "AEC_SPENERGY_VALUES": (33, 80, 4, "float"),    # speech energy per beam (same order)
    "AEC_MIC_ARRAY_TYPE":  (33, 73, 1, "int32"),
    "AEC_AECCONVERGED":    (33, 3, 1, "int32"),
}
_FMT = {"uint8": ("B", 1), "uint16": ("H", 2), "int32": ("i", 4), "uint32": ("I", 4), "float": ("f", 4)}


class XVF:
    TIMEOUT_MS = 1000

    def __init__(self, vid=XVF_VID, pid=None):
        import usb.core
        import usb.util
        self._util = usb.util
        kwargs = {"idVendor": vid}
        if pid:
            kwargs["idProduct"] = pid
        devs = sorted(usb.core.find(find_all=True, **kwargs) or [], key=lambda d: d.idProduct)
        if not devs:
            raise RuntimeError(f"no USB device with VID 0x{vid:04x}")
        self.dev = devs[0]

    def read(self, name):
        resid, cmdid, count, typ = PARAMS[name]
        fmt, size = _FMT[typ]
        length = count * size + 1
        u = self._util
        req = u.CTRL_IN | u.CTRL_TYPE_VENDOR | u.CTRL_RECIPIENT_DEVICE
        for _ in range(100):
            resp = self.dev.ctrl_transfer(req, 0, 0x80 | cmdid, resid, length, self.TIMEOUT_MS)
            if resp[0] == CONTROL_SUCCESS:
                return struct.unpack("<" + fmt * count, bytes(resp[1:length]))
            if resp[0] != SERVICER_COMMAND_RETRY:
                raise RuntimeError(f"{name}: status {resp[0]}")
            time.sleep(0.005)
        raise RuntimeError(f"{name}: device kept asking to retry")

    def doa(self):
        """(degrees 0-359, speech_detected bool)"""
        d, s = self.read("DOA_VALUE")
        return d % 360, bool(s)

    def beams(self):
        """(azimuths in degrees x4, speech energies x4)"""
        az = [math.degrees(a) % 360 for a in self.read("AEC_AZIMUTH_VALUES")]
        return az, list(self.read("AEC_SPENERGY_VALUES"))

    def close(self):
        try:
            self._util.dispose_resources(self.dev)
        except Exception:
            pass


def circular_mean_deg(values):
    if not values:
        return None
    s = sum(math.sin(math.radians(v)) for v in values)
    c = sum(math.cos(math.radians(v)) for v in values)
    return math.degrees(math.atan2(s, c)) % 360
