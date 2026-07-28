#!/usr/bin/env python3
"""
beetle_agent.py — the always-on process. Runs from boot, needs no ROS.

Replaces beetle_beacon.py and does three jobs:

  1. DISCOVERY   UDP beacon on 5010 so the Deck finds the robot anywhere
  2. WEB         serves beetle_console.html on 8080
  3. LAUNCHER    /api/start runs beetle_up.py; /api/status streams its output

WHY THESE THREE TOGETHER
  The console speaks rosbridge — which does not exist until the stack is up.
  So the console cannot be the thing that starts the stack, unless something
  outside ROS serves it. That is this. Boot the X5 and you immediately have
  a findable robot and a working page, with no ROS running at all; the page
  then starts ROS on request and watches it come up.

  It also means a failed bringup is VISIBLE. The gate output from beetle_up
  streams to the browser instead of vanishing into a terminal nobody has
  attached to.

INSTALL (run on boot)
    sudo cp beetle-agent.service /etc/systemd/system/
    sudo systemctl enable --now beetle-agent
    journalctl -u beetle-agent -f

RUN BY HAND
    python3 /userdata/beetle_agent.py --dir /root/beetle_hud

CONTROL ACCESS
  Anything that can reach port 8080 can start the robot. On the portable
  router that is the intended behaviour. On shared wifi, pass --token <str>
  (or put one in ~/.beetle_token) and the console will ask for it once.

API
    GET  /                 -> beetle_console.html
    GET  /api/status       -> {state, pid, uptime, args, lines[]}
    POST /api/start        -> {"args":["--spin"],"token":"..."}
    POST /api/stop         -> {"token":"..."}
"""

import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

BEACON_PORT = 5010
HTTP_PORT = 8080
PROBE = b"BEETLE?"
SERVICES = {"teleop": 5005, "telem": 5006, "rosbridge": 9090, "http": 8080}

BRINGUP = "/userdata/beetle_up.py"
CONTROL_PORT = 5011      # beetle_up's control channel, localhost only
TOKEN_FILE = Path.home() / ".beetle_token"

# Only these may be passed to beetle_up. An allow-list, not a filter — the
# args arrive over the network, and anything not named here is dropped rather
# than escaped, so no input can become a shell fragment.
ALLOWED_FLAGS = {
    "--drive-only", "--no-cloud", "--spin", "--console", "--foxglove",
    "--no-beacon", "--no-arp", "--yes",
}
ALLOWED_OPTS = {
    "--gyro-scale", "--stage-zero", "--max-wheel-tps",
    "--lidar-port", "--console-dir", "--retries",
}


# ============================================================ supervisor

class Bringup:
    """Owns at most one beetle_up.py child and its recent output."""

    def __init__(self, script=BRINGUP):
        self.script = script
        self.proc = None
        self.lines = deque(maxlen=400)
        self.args = []
        self.started = None
        self.exit_code = None
        self.stop_requested = False   # a requested stop is not a failure
        self.lock = threading.Lock()

    @property
    def state(self):
        if self.proc is None:
            if self.stop_requested or self.exit_code in (None, 0):
                return "idle"
            return "failed"
        if self.proc.poll() is not None:
            if self.stop_requested or self.proc.returncode == 0:
                return "idle"
            return "failed"
        # beetle_up prints this once every gate has passed.
        for ln in reversed(self.lines):
            if "BEETLE UP" in ln:
                return "up"
            if "UP, WITH SKIPPED STAGES" in ln:
                return "degraded"
        return "starting"

    def sanitize(self, raw):
        """Allow-list. Unknown tokens are dropped, values are never shelled."""
        out, i = [], 0
        raw = [str(x) for x in (raw or [])]
        while i < len(raw):
            a = raw[i]
            if a in ALLOWED_FLAGS:
                out.append(a)
            elif a in ALLOWED_OPTS and i + 1 < len(raw):
                v = raw[i + 1]
                if len(v) < 64 and all(c.isalnum() or c in "._-/" for c in v):
                    out += [a, v]
                i += 1
            i += 1
        return out

    def start(self, raw_args):
        with self.lock:
            if self.proc and self.proc.poll() is None:
                return False, "already running"
            if not Path(self.script).exists():
                return False, f"{self.script} not found"
            args = self.sanitize(raw_args)
            # No stdin when launched remotely, so beetle_up must not stop to
            # ask. --yes makes gate failures abort instead of prompting.
            if "--yes" not in args:
                args.append("--yes")
            # The agent already serves HTTP on 8080; don't let bringup bind it.
            self.lines.clear()
            self.args = args
            self.exit_code = None
            self.stop_requested = False
            self.started = time.time()
            # -u is essential. Python BLOCK-buffers stdout when it is a pipe,
            # so without it the entire bringup log sits in a 4-8 KB buffer and
            # only appears when the process exits — which looks exactly like a
            # hang. bufsize=1 below only affects our reading end.
            cmd = [sys.executable, "-u", self.script] + args
            self.lines.append("$ " + " ".join(cmd))
            self.proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True,
                cwd="/userdata")
            threading.Thread(target=self._pump, daemon=True).start()
            return True, "started"

    def _pump(self):
        p = self.proc
        try:
            for line in p.stdout:
                self.lines.append(line.rstrip("\n"))
        except (OSError, ValueError):
            pass
        p.wait()
        self.exit_code = p.returncode
        self.lines.append(f"-- exited with code {p.returncode} --")

    def stop(self):
        with self.lock:
            if not self.proc or self.proc.poll() is not None:
                return False, "not running"
            self.stop_requested = True
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                self.proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except OSError:
                    pass
            except OSError:
                pass
            return True, "stopped"

    def status(self, n=120):
        return {
            "state": self.state,
            "pid": self.proc.pid if self.proc and self.proc.poll() is None else None,
            "uptime": round(time.time() - self.started, 1) if self.started else 0,
            "args": self.args,
            "exit_code": self.exit_code,
            "lines": list(self.lines)[-n:],
        }


# ============================================================ usb recovery

# ODrive native USB. The ODESC boards clone it, which is why the library
# prints "Not a genuine ODrive!" on connect.
ODRIVE_VID, ODRIVE_PID = "1209", "0d32"
USBDEVFS_RESET = (ord("U") << 8) | 20      # _IO('U', 20)


def usb_reset(vid=ODRIVE_VID, pid=ODRIVE_PID):
    """Host-side USB port reset for every matching device.

    The last resort, and the reason it lives HERE rather than on the tuner:
    when the drives fall off the bus, odrive.find_any() blocks and the ROS
    node cannot answer a service call at all. The agent holds no ROS state and
    keeps running regardless, so it is the only thing that can still act.

    Equivalent to the `usbreset` utility — re-enumerates the device without
    rebooting the board. Stop the drive node first if it is responsive;
    resetting the bus underneath a process holding the handle works, but it
    will have to reconnect afterwards either way."""
    import fcntl
    out = []
    base = Path("/sys/bus/usb/devices")
    if not base.exists():
        return ["no /sys/bus/usb/devices on this system"]
    for dev in sorted(base.glob("*")):
        try:
            if (dev / "idVendor").read_text().strip().lower() != vid.lower():
                continue
            if (dev / "idProduct").read_text().strip().lower() != pid.lower():
                continue
            bus = int((dev / "busnum").read_text())
            num = int((dev / "devnum").read_text())
        except (OSError, ValueError):
            continue
        node = f"/dev/bus/usb/{bus:03d}/{num:03d}"
        try:
            fd = os.open(node, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                out.append(f"reset {dev.name} at {node}")
            finally:
                os.close(fd)
        except OSError as e:
            out.append(f"{node}: {e}")
    return out or [f"no USB device matching {vid}:{pid} — are they powered?"]


# ============================================================ node control

def control(req, timeout=12.0):
    """Ask beetle_up to act on one node.

    A thin proxy on purpose. beetle_up owns every child process and holds the
    rclpy context needed to re-run that node's gates after a restart, so it is
    the only thing that CAN do this correctly. Duplicating the command table
    here is how the two would drift apart."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(timeout)
        s.sendto(json.dumps(req).encode(), ("127.0.0.1", CONTROL_PORT))
        data, _ = s.recvfrom(65535)
        return json.loads(data.decode())
    except socket.timeout:
        return {"ok": False, "msg": "supervisor did not answer — restarting a "
                                    "node can take a few seconds"}
    except OSError:
        return {"ok": False, "msg": "supervisor not running — start the robot first"}
    except ValueError:
        return {"ok": False, "msg": "bad reply from supervisor"}
    finally:
        try:
            s.close()
        except Exception:
            pass


# ============================================================ system monitor

class SysMon:
    """Samples /proc and /sys once a second and keeps a short history.

    Lives in the agent rather than a ROS node on purpose: the agent is always
    up, so load is observable BEFORE, DURING and AFTER bringup. Watching the
    cost of each stage appear as it starts is the whole point.

    Everything here is a plain file read — no psutil, no subprocess. The board
    has no internet on the portable router, so a pip dependency is the one
    thing this must not have."""

    BPU_RATIO = "/sys/devices/system/bpu/bpu0/ratio"   # X5 has one BPU, no bpu1

    def __init__(self, history=180):
        self.hist = deque(maxlen=history)   # ~3 min at 1 Hz
        self.ncpu = os.cpu_count() or 8
        self._cpu_prev = None
        self._core_prev = None
        self._net_prev = None
        self._proc_prev = {}
        self._latest = {}
        self.lock = threading.Lock()

    # ---- individual readings ----

    @staticmethod
    def _read(path, default=None):
        try:
            return Path(path).read_text().strip()
        except OSError:
            return default

    def _cpu(self):
        """Aggregate and per-core busy percentage from /proc/stat deltas."""
        try:
            lines = Path("/proc/stat").read_text().splitlines()
        except OSError:
            return 0.0, []
        tot, cores = None, []
        for ln in lines:
            if not ln.startswith("cpu"):
                break
            f = ln.split()
            vals = [int(x) for x in f[1:11]]
            idle = vals[3] + vals[4]              # idle + iowait
            busy = sum(vals) - idle
            if f[0] == "cpu":
                tot = (busy, busy + idle)
            else:
                cores.append((busy, busy + idle))

        def pct(now, prev):
            if not prev:
                return 0.0
            db, dt = now[0] - prev[0], now[1] - prev[1]
            return round(100.0 * db / dt, 1) if dt > 0 else 0.0

        agg = pct(tot, self._cpu_prev) if tot else 0.0
        per = [pct(c, p) for c, p in zip(cores, self._core_prev or [])] \
            if self._core_prev else [0.0] * len(cores)
        self._cpu_prev, self._core_prev = tot, cores
        return agg, per

    def _mem(self):
        info = {}
        try:
            for ln in Path("/proc/meminfo").read_text().splitlines():
                k, _, v = ln.partition(":")
                info[k] = int(v.split()[0])       # kB
        except (OSError, ValueError, IndexError):
            return 0, 0, 0.0
        total = info.get("MemTotal", 0) / 1024.0
        avail = info.get("MemAvailable", 0) / 1024.0
        used = total - avail
        return round(used), round(total), round(100.0 * used / total, 1) if total else 0.0

    def _temp(self):
        """Hottest thermal zone, and which one it was."""
        best, name = None, ""
        base = Path("/sys/class/thermal")
        if base.exists():
            for z in sorted(base.glob("thermal_zone*")):
                raw = self._read(z / "temp")
                if not raw:
                    continue
                try:
                    c = int(raw) / 1000.0
                except ValueError:
                    continue
                if c > 200:                       # some zones report milli-C oddly
                    c /= 1000.0
                if best is None or c > best:
                    best, name = c, self._read(z / "type", z.name) or z.name
        return (round(best, 1) if best is not None else None), name

    def _bpu(self):
        raw = self._read(self.BPU_RATIO)
        try:
            return int(raw)
        except (TypeError, ValueError):
            return None

    def _freq(self):
        raw = self._read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        try:
            return round(int(raw) / 1000)         # MHz
        except (TypeError, ValueError):
            return None

    def _net(self, now):
        """Wireless throughput. Foxglove image panels cost ~14 MB/s, which is
        invisible in CPU but obvious here."""
        rx = tx = 0
        try:
            for ln in Path("/proc/net/dev").read_text().splitlines()[2:]:
                name, _, rest = ln.partition(":")
                name = name.strip()
                if name == "lo" or name.startswith(("docker", "veth", "br-")):
                    continue
                f = rest.split()
                rx += int(f[0])
                tx += int(f[8])
        except (OSError, ValueError, IndexError):
            return 0.0, 0.0
        out = (0.0, 0.0)
        if self._net_prev:
            dt = now - self._net_prev[2]
            if dt > 0:
                out = (round((rx - self._net_prev[0]) / dt / 1024, 1),
                       round((tx - self._net_prev[1]) / dt / 1024, 1))   # kB/s
        self._net_prev = (rx, tx, now)
        return out

    def _procs(self, dt, top=6):
        """Top CPU consumers. comm in /proc/pid/stat truncates to 15 chars and
        every node here is 'python3', so resolve the real name from cmdline —
        but only for the few that made the cut."""
        cur, ranked = {}, []
        for d in Path("/proc").iterdir():
            if not d.name.isdigit():
                continue
            try:
                f = (d / "stat").read_text().rsplit(") ", 1)[-1].split()
                jiff = int(f[11]) + int(f[12])          # utime + stime
            except (OSError, ValueError, IndexError):
                continue
            cur[d.name] = jiff
            prev = self._proc_prev.get(d.name)
            if prev is not None and dt > 0:
                pct = 100.0 * (jiff - prev) / (os.sysconf("SC_CLK_TCK") * dt)
                if pct > 1.0:
                    ranked.append((pct, d.name))
        self._proc_prev = cur
        ranked.sort(reverse=True)
        out = []
        for pct, pid in ranked[:top]:
            name = "?"
            try:
                parts = (Path("/proc") / pid / "cmdline").read_bytes().split(b"\0")
                args = [p.decode(errors="replace") for p in parts if p]
                name = Path(args[0]).name if args else "?"
                for a in args[1:]:
                    if a.endswith(".py") or ("/" in a and not a.startswith("-")):
                        name = Path(a).name
                        break
            except (OSError, IndexError):
                pass
            out.append({"pid": int(pid), "cpu": round(pct, 1), "name": name[:22]})
        return out

    # ---- loop ----

    def sample(self):
        now = time.time()
        dt = now - self._latest.get("t", now - 1.0)
        cpu, cores = self._cpu()
        used, total, mempct = self._mem()
        temp, tname = self._temp()
        rx, tx = self._net(now)
        try:
            load = os.getloadavg()[0]
        except OSError:
            load = 0.0
        snap = {
            "t": round(now, 1),
            "cpu": cpu, "cores": cores, "ncpu": self.ncpu,
            "load": round(load, 2),
            "mem_used": used, "mem_total": total, "mem_pct": mempct,
            "temp": temp, "temp_zone": tname,
            "bpu": self._bpu(), "freq": self._freq(),
            "rx": rx, "tx": tx,
            "procs": self._procs(dt),
        }
        with self.lock:
            self._latest = snap
            self.hist.append((snap["t"], cpu, snap["bpu"] or 0,
                              temp or 0, mempct))
        return snap

    def snapshot(self):
        with self.lock:
            h = list(self.hist)
            return {
                "now": dict(self._latest),
                "hist": {
                    "t":    [x[0] for x in h],
                    "cpu":  [x[1] for x in h],
                    "bpu":  [x[2] for x in h],
                    "temp": [x[3] for x in h],
                    "mem":  [x[4] for x in h],
                },
            }

    def loop(self, period=1.0):
        while True:
            try:
                self.sample()
            except Exception:
                pass          # telemetry must never take the agent down
            time.sleep(period)


# ============================================================ interfaces

def ipv4_of(ifname):
    import fcntl
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(), 0x8915, struct.pack("256s", ifname[:15].encode()))[20:24])
    except OSError:
        return None
    finally:
        s.close()


def interfaces():
    out = []
    net = Path("/sys/class/net")
    if not net.exists():
        return out
    for dev in sorted(net.iterdir()):
        n = dev.name
        if n == "lo" or n.startswith(("docker", "veth", "br-")):
            continue
        try:
            mac = (dev / "address").read_text().strip()
        except OSError:
            continue
        if not mac or mac == "00:00:00:00:00:00":
            continue
        ip = ipv4_of(n)
        if ip:
            out.append({"name": n, "ip": ip, "mac": mac})
    return out


def broadcast_addrs(ifaces):
    a = ["255.255.255.255"]
    for i in ifaces:
        p = i["ip"].split(".")
        if len(p) == 4:
            a.append(".".join(p[:3] + ["255"]))
    return sorted(set(a))


# ============================================================ beacon thread

def beacon_loop(bringup, port, interval):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.settimeout(0.25)

    ifaces = interfaces()
    last = refreshed = 0.0

    def payload():
        return json.dumps({
            "beetle": 1,
            "host": socket.gethostname(),
            "t": round(time.time(), 1),
            "state": bringup.state,      # so the Deck knows before connecting
            "ifaces": ifaces,
            "svc": SERVICES,
        }).encode()

    while True:
        now = time.time()
        if now - refreshed > 10.0:       # roaming changes the IP
            ifaces = interfaces()
            refreshed = now
        if now - last >= interval:
            data = payload()
            for addr in broadcast_addrs(ifaces):
                try:
                    sock.sendto(data, (addr, port))
                except OSError:
                    pass
            last = now
        try:
            msg, src = sock.recvfrom(2048)
        except (socket.timeout, OSError):
            continue
        if msg.startswith(PROBE):
            try:
                sock.sendto(payload(), src)
            except OSError:
                pass


# ============================================================ http

def make_handler(bringup, root, token, sysmon):
    class H(SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=str(root), **kw)

        def log_message(self, *a):
            pass                      # journal is noisy enough

        def end_headers(self):
            # SimpleHTTPRequestHandler sets no cache headers, so a browser can
            # hold an old beetle_console.html for as long as it likes. That is
            # how two machines on the same server end up showing different
            # pages. This file changes often; never cache it.
            self.send_header("Cache-Control", "no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            super().end_headers()

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _authed(self, data):
            return (not token) or (data.get("token") == token)

        def do_GET(self):
            if self.path.startswith("/api/log.txt"):
                body = "\n".join(bringup.status(400)["lines"]).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                return self.wfile.write(body)
            if self.path.startswith("/api/nodes"):
                return self._json(control({"action": "list"}, timeout=3.0))
            if self.path.startswith("/api/sys"):
                return self._json(sysmon.snapshot())
            if self.path.startswith("/api/status"):
                st = bringup.status()
                st["token_required"] = bool(token)
                return self._json(st)
            if self.path in ("/", ""):
                self.path = "/beetle_console.html"
            return super().do_GET()

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, OSError):
                return self._json({"ok": False, "msg": "bad request"}, 400)
            if not self._authed(data):
                return self._json({"ok": False, "msg": "bad token"}, 403)
            if self.path.startswith("/api/start"):
                ok, msg = bringup.start(data.get("args", []))
                return self._json({"ok": ok, "msg": msg})
            if self.path.startswith("/api/stop"):
                ok, msg = bringup.stop()
                return self._json({"ok": ok, "msg": msg})
            if self.path.startswith("/api/usbreset"):
                lines = usb_reset()
                ok = any(l.startswith("reset ") for l in lines)
                return self._json({"ok": ok, "msg": "; ".join(lines),
                                   "lines": lines})
            if self.path.startswith("/api/node"):
                act = data.get("action", "")
                if act not in ("start", "stop", "restart", "check", "list"):
                    return self._json({"ok": False, "msg": "bad action"}, 400)
                return self._json(control({"action": act,
                                           "name": str(data.get("name", ""))[:40]}))
            return self._json({"ok": False, "msg": "unknown endpoint"}, 404)
    return H


# ============================================================ main

def install_signal_handlers(bringup, srv):
    """systemd sends SIGTERM, which is NOT KeyboardInterrupt — so the cleanup
    in main()'s finally never ran on `systemctl restart`, and beetle_up plus
    every node it started survived as orphans. The next START ROBOT then
    launched a second full stack alongside the first."""
    def bye(signum, _frame):
        print(f"signal {signum} — stopping bringup before exit", flush=True)
        try:
            bringup.stop()
        finally:
            srv.shutdown()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        try:
            signal.signal(sig, bye)
        except (OSError, ValueError):
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/root/beetle_hud",
                    help="directory holding beetle_console.html and roslib.min.js")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--beacon-port", type=int, default=BEACON_PORT)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--bringup", default=BRINGUP)
    ap.add_argument("--token", default=None,
                    help=f"shared secret; falls back to {TOKEN_FILE} if present")
    ap.add_argument("--no-control", action="store_true",
                    help="serve and beacon, but refuse to start anything")
    a = ap.parse_args()

    token = a.token
    if token is None and TOKEN_FILE.exists():
        token = TOKEN_FILE.read_text().strip() or None

    root = Path(a.dir)
    root.mkdir(parents=True, exist_ok=True)

    sysmon = SysMon()
    sysmon.sample()                      # prime the deltas
    threading.Thread(target=sysmon.loop, daemon=True).start()

    bringup = Bringup(a.bringup)
    if a.no_control:
        bringup.start = lambda *_: (False, "control disabled")

    threading.Thread(target=beacon_loop,
                     args=(bringup, a.beacon_port, a.interval),
                     daemon=True).start()

    for i in interfaces():
        print(f"  {i['name']:<8} {i['ip']:<15} {i['mac']}")
    print(f"beacon  udp/{a.beacon_port}")
    print(f"console http://<ip>:{a.http_port}/  serving {root}")
    print(f"control {'token required' if token else 'OPEN on this network'}"
          f"{' (disabled)' if a.no_control else ''}")

    srv = ThreadingHTTPServer(("", a.http_port),
                              make_handler(bringup, root, token, sysmon))
    install_signal_handlers(bringup, srv)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bringup.stop()
        srv.server_close()


if __name__ == "__main__":
    main()
