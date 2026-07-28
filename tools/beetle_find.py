#!/usr/bin/env python3
"""
beetle_find.py — find the robot on any network, from the Deck or a laptop.

CLI:
    python3 beetle_find.py                 # print the IP, or exit 1
    python3 beetle_find.py --verbose       # show every candidate and method
    python3 beetle_find.py --url           # print the console URL
    python3 beetle_find.py --pin           # remember this robot's MAC

Import:
    from beetle_find import find_robot
    host = find_robot() or "192.168.86.58"

HOW IT LOOKS, IN ORDER
  1. BEACON     broadcast probe on UDP 5010, robot answers with IP + MAC.
                ~50 ms. Needs beetle_beacon.py running on the X5.
  2. CACHE      last known good address, verified by an actual TCP connect
                before it is trusted. Instant when nothing moved.
  3. MDNS       <hostname>.local, if avahi is around.
  4. ARP        the fallback you asked for — sweep the subnet and match a
                pinned MAC. Slowest and same-subnet only, but it works when
                the beacon is not running yet.

  Every candidate is verified by opening a real TCP connection to a port the
  robot actually serves, so a stale ARP entry or a recycled DHCP lease cannot
  hand you the wrong machine.

PINNING
  --pin writes the discovered MAC to ~/.beetle_host. After that, discovery
  only accepts that machine. That is the "search by MAC" behaviour, without
  needing a scan to do it.
"""

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

BEACON_PORT = 5010
PROBE = b"BEETLE?"
CACHE = Path.home() / ".beetle_host"
VERIFY_PORTS = (5005, 8080, 9090, 22)   # UDP teleop is 5005; TCP ones below
TCP_VERIFY = (8080, 9090, 22)


def log(v, *a):
    if v:
        print("   ", *a, file=sys.stderr)


# ---------------------------------------------------------------- cache

def load_cache():
    try:
        return json.loads(CACHE.read_text())
    except (OSError, ValueError):
        return {}


def save_cache(d):
    try:
        merged = load_cache()
        merged.update(d)
        CACHE.write_text(json.dumps(merged, indent=2) + "\n")
    except OSError:
        pass


# ---------------------------------------------------------------- verify

def tcp_alive(ip, port, timeout=0.4):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def verify(ip, verbose=False):
    """Something on the robot must actually answer. A ping is not enough —
    DHCP recycles addresses and ARP caches go stale."""
    for p in TCP_VERIFY:
        if tcp_alive(ip, p):
            log(verbose, f"verified {ip} on tcp/{p}")
            return True
    return False


# ---------------------------------------------------------------- 1. beacon

def by_beacon(timeout=1.5, verbose=False):
    """Broadcast a probe; collect every answer."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.25)
    try:
        sock.bind(("", 0))
    except OSError:
        pass

    targets = ["255.255.255.255"]
    try:
        # Our own address tells us the local /24 to direct-broadcast into.
        s2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s2.connect(("8.8.8.8", 53))          # no traffic sent; just routing
        mine = s2.getsockname()[0]
        s2.close()
        targets.append(".".join(mine.split(".")[:3] + ["255"]))
    except OSError:
        pass

    for t in set(targets):
        try:
            sock.sendto(PROBE, (t, BEACON_PORT))
        except OSError:
            pass

    found, end = [], time.time() + timeout
    while time.time() < end:
        try:
            data, src = sock.recvfrom(4096)
        except (socket.timeout, OSError):
            continue
        try:
            d = json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            continue
        if d.get("beetle") != 1:
            continue
        for i in d.get("ifaces", []):
            if i.get("ip"):
                found.append({"ip": i["ip"], "mac": i.get("mac", ""),
                              "host": d.get("host", ""), "via": "beacon",
                              "svc": d.get("svc", {})})
        if not d.get("ifaces") and src:
            found.append({"ip": src[0], "mac": "", "host": d.get("host", ""),
                          "via": "beacon", "svc": d.get("svc", {})})
    sock.close()
    # Both the global and the directed broadcast get answered, so the same
    # robot arrives twice. Dedupe on ip+mac, keeping first-seen order.
    uniq, seen_keys = [], set()
    for c in found:
        k = (c["ip"], c["mac"])
        if k not in seen_keys:
            seen_keys.add(k)
            uniq.append(c)
    log(verbose, f"beacon: {len(uniq)} candidate(s)")
    return uniq


# ---------------------------------------------------------------- 2. mdns

def by_mdns(names=("ubuntu.local", "beetle.local"), verbose=False):
    out = []
    for n in names:
        try:
            ip = socket.gethostbyname(n)
            out.append({"ip": ip, "mac": "", "host": n, "via": "mdns", "svc": {}})
            log(verbose, f"mdns: {n} -> {ip}")
        except OSError:
            pass
    return out


# ---------------------------------------------------------------- 3. arp

def arp_table():
    """Parse `arp -a`. Format differs between Linux and Windows, so handle
    both — this gets run from the Deck and from a Windows laptop."""
    try:
        txt = subprocess.run(["arp", "-a"], capture_output=True, text=True,
                             timeout=6).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    rows = []
    for line in txt.splitlines():
        parts = line.replace("(", " ").replace(")", " ").split()
        ip = mac = None
        for p in parts:
            if p.count(".") == 3 and p[0].isdigit():
                ip = p
            elif p.count(":") == 5 or p.count("-") == 5:
                mac = p.lower().replace("-", ":")
        if ip and mac:
            rows.append((ip, mac))
    return rows


def by_arp(pinned_mac, verbose=False, sweep=True):
    """Match a pinned MAC in the ARP table, sweeping first to populate it."""
    if not pinned_mac:
        log(verbose, "arp: no pinned MAC — run --pin once while the beacon is up")
        return []
    if sweep:
        prefix = local_prefix()
        if prefix:
            log(verbose, f"arp: sweeping {prefix}0/24 (a few seconds)")
            sweep_subnet(prefix)
    hits = [{"ip": ip, "mac": mac, "host": "", "via": "arp", "svc": {}}
            for ip, mac in arp_table() if mac == pinned_mac.lower()]
    log(verbose, f"arp: {len(hits)} match(es) for {pinned_mac}")
    return hits


def local_prefix():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 53))
        ip = s.getsockname()[0]
        s.close()
        return ".".join(ip.split(".")[:3]) + "."
    except OSError:
        return None


def sweep_subnet(prefix, port=22, workers=64):
    """Touch every host so the kernel fills its ARP cache. TCP connects
    rather than ICMP, because unprivileged ping is not available everywhere
    and this needs no root."""
    from concurrent.futures import ThreadPoolExecutor
    def poke(n):
        try:
            s = socket.socket()
            s.settimeout(0.25)
            s.connect_ex((f"{prefix}{n}", port))
            s.close()
        except OSError:
            pass
    with ThreadPoolExecutor(max_workers=workers) as ex:
        list(ex.map(poke, range(1, 255)))


# ---------------------------------------------------------------- driver

def find_robot(timeout=1.5, pinned_mac=None, verbose=False, use_arp=True):
    """Return the robot's IP, or None."""
    cache = load_cache()
    pinned_mac = pinned_mac or cache.get("mac")

    def acceptable(c):
        if pinned_mac and c.get("mac") and c["mac"].lower() != pinned_mac.lower():
            log(verbose, f"rejecting {c['ip']} — MAC {c['mac']} is not the pinned one")
            return False
        return verify(c["ip"], verbose)

    for c in by_beacon(timeout, verbose):
        if acceptable(c):
            save_cache({"ip": c["ip"], "mac": c.get("mac") or cache.get("mac", ""),
                        "host": c.get("host", ""), "seen": time.time()})
            log(verbose, f"found via beacon: {c['ip']}")
            return c["ip"]

    if cache.get("ip"):
        log(verbose, f"trying cached {cache['ip']}")
        if verify(cache["ip"], verbose):
            return cache["ip"]

    for c in by_mdns(verbose=verbose):
        if acceptable(c):
            save_cache({"ip": c["ip"], "seen": time.time()})
            return c["ip"]

    if use_arp:
        for c in by_arp(pinned_mac, verbose):
            if verify(c["ip"], verbose):
                save_cache({"ip": c["ip"], "seen": time.time()})
                return c["ip"]

    return None


def main():
    ap = argparse.ArgumentParser(description="Locate the Beetle robot.")
    ap.add_argument("--timeout", type=float, default=1.5)
    ap.add_argument("--verbose", "-v", action="store_true")
    ap.add_argument("--url", action="store_true", help="print the console URL")
    ap.add_argument("--pin", action="store_true",
                    help="remember the discovered MAC; later runs accept only it")
    ap.add_argument("--unpin", action="store_true")
    ap.add_argument("--no-arp", action="store_true", help="skip the subnet sweep")
    a = ap.parse_args()

    if a.unpin:
        save_cache({"mac": ""})
        print("MAC unpinned")
        return 0

    if a.pin:
        for c in by_beacon(a.timeout, a.verbose):
            if c.get("mac"):
                save_cache({"mac": c["mac"], "ip": c["ip"],
                            "host": c.get("host", ""), "seen": time.time()})
                print(f"pinned {c['mac']}  ({c.get('host','?')} at {c['ip']})")
                return 0
        print("no beacon answered — is beetle_beacon.py running?", file=sys.stderr)
        return 1

    ip = find_robot(a.timeout, verbose=a.verbose, use_arp=not a.no_arp)
    if not ip:
        print("robot not found", file=sys.stderr)
        print("  try: start beetle_beacon.py on the X5, or --verbose to see why",
              file=sys.stderr)
        return 1
    print(f"http://{ip}:8080/beetle_console.html" if a.url else ip)
    return 0


if __name__ == "__main__":
    sys.exit(main())
