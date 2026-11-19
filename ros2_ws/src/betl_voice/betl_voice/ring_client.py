"""Talks to the RP2350 LED ring firmware (firmware/rp2350_led_ring/main.py) over USB serial."""

import threading
import time

RP2_VID = 0x2E8A          # Raspberry Pi (RP2040 / RP2350 boards running MicroPython)


def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == RP2_VID:
            return p.device
    return None


class RingClient:
    def __init__(self, port="auto", num_leds=16, log=print):
        self.port_cfg = port
        self.num_leds = num_leds
        self.log = log
        self.ser = None
        self.lock = threading.Lock()
        self.last_try = 0.0

    def connect(self):
        import serial
        if time.time() - self.last_try < 2.0:
            return False
        self.last_try = time.time()
        port = find_port() if self.port_cfg == "auto" else self.port_cfg
        if not port:
            return False
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.2, write_timeout=0.2)
            self.ser.reset_input_buffer()
            self.ser.write(b"\n?\n")
            reply = self.ser.readline().decode(errors="ignore").strip()
            if not reply.startswith("BETL-RING"):
                reply = self.ser.readline().decode(errors="ignore").strip()
            if reply.startswith("BETL-RING"):
                parts = reply.split()
                if len(parts) >= 3:
                    self.num_leds = int(parts[2])
                self.log(f"LED ring on {port}: {reply}")
            else:
                self.log(f"{port} answered '{reply}' (is the BETL firmware on it?); sending anyway")
            return True
        except Exception as e:
            self.log(f"LED ring connect failed on {port}: {e}")
            self.ser = None
            return False

    def send(self, line):
        with self.lock:
            if self.ser is None and not self.connect():
                return False
            try:
                self.ser.write((line + "\n").encode())
                # drain replies so the buffer never fills
                if self.ser.in_waiting:
                    self.ser.read(self.ser.in_waiting)
                return True
            except Exception as e:
                self.log(f"LED ring write failed ({e}); will reconnect")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                return False
