#!/usr/bin/env python3
"""Minimal VXI-11 / SCPI controller for a Siglent SDG1032X.

No third-party packages required.  It speaks the VXI-11 core channel directly
(TCP 715, program 0x0607AF, device "inst0") because this generator does not
expose SCPI on raw TCP 5025.

Examples:
    python sdg1032x.py idn
    python sdg1032x.py get
    python sdg1032x.py set --channel 1 --wave sine --freq 1k --amp 0.5 --offset 0
    python sdg1032x.py set --channel 1 --wave square --freq 10k --amp 1 --output on
    python sdg1032x.py output --channel 1 --state off
    python sdg1032x.py query "C1:BSWV?"
    python sdg1032x.py send "C1:OUTP OFF"

Amplitude is Vpp (the generator's AMP parameter), not Vrms.  The generator's
current load setting is normally HZ (high impedance); into 50 ohm the actual
voltage is half the displayed Vpp.
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys

DEFAULT_HOST = os.environ.get("SDG1032X_HOST", "10.11.9.230")
VXI_PORT = 715
VXI_PROG = 0x0607AF
VXI_VERS = 1
FLAG_END = 8
FLAG_TERMCHRSET = 128
TERM_CHAR = 10


class Vxi11Error(RuntimeError):
    pass


def _pad4(data: bytes) -> bytes:
    return data + b"\x00" * ((-len(data)) % 4)


def _xdr_string(text: str) -> bytes:
    data = text.encode("ascii")
    return struct.pack("!I", len(data)) + _pad4(data)


def _xdr_opaque(data: bytes) -> bytes:
    return struct.pack("!I", len(data)) + _pad4(data)


class Vxi11:
    def __init__(self, host: str, device: str = "inst0", timeout: float = 3.0):
        self.host = host
        self.device = device
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._xid = 0x40000000
        self._lid: int | None = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def connect(self) -> None:
        self._sock = socket.create_connection((self.host, VXI_PORT), self.timeout)
        self._sock.settimeout(self.timeout)
        try:
            # Create_LinkParms: clientId, lockDevice(bool), lock_timeout, device
            args = struct.pack("!III", 1, 0, 0) + _xdr_string(self.device)
            reply = self._call(10, args)
            error, lid, _abort_port, _max_recv = struct.unpack("!IIII", reply[:16])
            if error != 0:
                raise Vxi11Error(f"create_link failed: error={error}")
            self._lid = lid
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        if self._sock is not None and self._lid is not None:
            try:
                self._call(23, struct.pack("!I", self._lid))
            except Exception:
                pass
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._lid = None

    def _call(self, proc: int, args: bytes = b"") -> bytes:
        if self._sock is None:
            raise Vxi11Error("not connected")
        self._xid = (self._xid + 1) & 0xFFFFFFFF
        # RPC CALL: xid, CALL, rpcvers=2, prog, vers, proc, null auth x2
        header = struct.pack(
            "!IIIIIIIIII",
            self._xid, 0, 2, VXI_PROG, VXI_VERS, proc, 0, 0, 0, 0,
        )
        message = header + args
        self._sock.sendall(struct.pack("!I", 0x80000000 | len(message)) + message)

        record = b""
        while True:
            head = self._recv_exact(4)
            marker = struct.unpack("!I", head)[0]
            length = marker & 0x7FFFFFFF
            record += self._recv_exact(length)
            if marker & 0x80000000:
                break

        offset = 0

        def u32() -> int:
            nonlocal offset
            value = struct.unpack_from("!I", record, offset)[0]
            offset += 4
            return value

        _xid = u32()
        msg_type = u32()
        reply_stat = u32()
        _verf_flavor = u32()
        verf_len = u32()
        offset += verf_len
        accept_stat = u32()
        if msg_type != 1 or reply_stat != 0 or accept_stat != 0:
            raise Vxi11Error(
                f"RPC failed: msg_type={msg_type} reply={reply_stat} "
                f"accept={accept_stat} raw={record.hex()}"
            )
        return record[offset:]

    def _recv_exact(self, count: int) -> bytes:
        if self._sock is None:
            raise Vxi11Error("not connected")
        data = b""
        while len(data) < count:
            chunk = self._sock.recv(count - len(data))
            if not chunk:
                raise Vxi11Error("connection closed")
            data += chunk
        return data

    def write(self, command: str) -> None:
        if self._lid is None:
            raise Vxi11Error("no VXI-11 link")
        if not command.endswith("\n"):
            command += "\n"
        # Device_WriteParms: lid, io_timeout, lock_timeout, flags, data<>
        args = struct.pack("!IIII", self._lid, 3000, 0, FLAG_END)
        args += _xdr_opaque(command.encode("ascii"))
        reply = self._call(11, args)
        error, _size = struct.unpack("!II", reply[:8])
        if error != 0:
            raise Vxi11Error(f"device_write failed: error={error}")

    def read(self, size: int = 4096) -> str:
        if self._lid is None:
            raise Vxi11Error("no VXI-11 link")
        # Device_ReadParms: lid, requestSize, io_timeout, lock_timeout, flags, termChar
        args = struct.pack("!IIIII", self._lid, size, 3000, 0, FLAG_TERMCHRSET)
        args += struct.pack("!I", TERM_CHAR)
        reply = self._call(12, args)
        error, reason, length = struct.unpack("!III", reply[:12])
        if error != 0:
            raise Vxi11Error(f"device_read failed: error={error} reason={reason}")
        return reply[12:12 + length].decode("ascii", errors="replace")

    def query(self, command: str) -> str:
        self.write(command)
        return self.read().strip()


def _freq_value(text: str) -> str:
    """Accept 1000, 1k, 1kHz, 10k, 100000 and return a plain Hz string."""
    value = text.strip().lower().replace("hz", "")
    multiplier = 1
    if value.endswith("k"):
        multiplier = 1000
        value = value[:-1]
    elif value.endswith("m"):
        multiplier = 1_000_000
        value = value[:-1]
    try:
        number = float(value) * multiplier
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid frequency: {text}") from exc
    if number <= 0:
        raise argparse.ArgumentTypeError("frequency must be positive")
    return f"{number:g}"


def build_commands(args) -> list[str]:
    channel = args.channel
    parts: list[str] = []
    if args.wave:
        wave = args.wave.upper()
        aliases = {
            "SIN": "SINE",
            "SINE": "SINE",
            "SQU": "SQUARE",
            "SQUARE": "SQUARE",
            "TRI": "TRIANGLE",
            "TRIANGLE": "TRIANGLE",
            "RAMP": "RAMP",
            "PULSE": "PULSE",
            "NOISE": "NOISE",
            "ARB": "ARB",
        }
        if wave not in aliases:
            raise SystemExit(f"unsupported wave: {args.wave}")
        parts += ["WVTP", aliases[wave]]
    if args.freq:
        parts += ["FRQ", _freq_value(args.freq)]
    if args.amp is not None:
        if args.amp <= 0:
            raise SystemExit("amplitude must be positive")
        parts += ["AMP", f"{args.amp:g}"]
    if args.offset is not None:
        parts += ["OFST", f"{args.offset:g}"]

    commands: list[str] = []
    if parts:
        commands.append(f"C{channel}:BSWV " + ",".join(parts))
    if args.output:
        commands.append(f"C{channel}:OUTP {'ON' if args.output == 'on' else 'OFF'}")
    return commands


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SDG1032X VXI-11 SCPI controller")
    parser.add_argument("--host", default=DEFAULT_HOST, help="instrument IP")
    parser.add_argument("--device", default="inst0", help="VXI-11 device name")
    parser.add_argument("--timeout", type=float, default=3.0, help="socket timeout (s)")
    sub = parser.add_subparsers(dest="action", required=True)

    sub.add_parser("idn", help="query *IDN?")

    get = sub.add_parser("get", help="show channel waveform and output state")
    get.add_argument("--channel", type=int, choices=(1, 2), default=1)

    set_p = sub.add_parser("set", help="set waveform/frequency/amplitude/offset")
    set_p.add_argument("--channel", type=int, choices=(1, 2), default=1)
    set_p.add_argument("--wave", help="sine/square/triangle/ramp/pulse/noise/arb")
    set_p.add_argument("--freq", help="frequency, e.g. 1k, 10k, 1000")
    set_p.add_argument("--amp", type=float, help="amplitude in Vpp")
    set_p.add_argument("--offset", type=float, help="DC offset in V")
    set_p.add_argument("--output", choices=("on", "off"), help="also switch output")
    set_p.add_argument("--dry-run", action="store_true", help="print SCPI only")

    output = sub.add_parser("output", help="switch channel output on/off")
    output.add_argument("--channel", type=int, choices=(1, 2), default=1)
    output.add_argument("--state", choices=("on", "off"), required=True)

    query = sub.add_parser("query", help="send a SCPI query and print the response")
    query.add_argument("command", help='e.g. "C1:BSWV?"')

    send = sub.add_parser("send", help="send a raw SCPI command without reading")
    send.add_argument("command", help='e.g. "C1:OUTP OFF"')

    return parser


def main() -> int:
    args = make_parser().parse_args()

    if args.action == "set":
        commands = build_commands(args)
        if not commands:
            raise SystemExit("set: specify at least one of --wave/--freq/--amp/--offset/--output")
        for command in commands:
            print(command)
        if args.dry_run:
            return 0
        with Vxi11(args.host, args.device, args.timeout) as inst:
            for command in commands:
                inst.write(command)
            print(inst.query(f"C{args.channel}:BSWV?").strip())
            print(inst.query(f"C{args.channel}:OUTP?").strip())
        return 0

    with Vxi11(args.host, args.device, args.timeout) as inst:
        if args.action == "idn":
            print(inst.query("*IDN?"))
        elif args.action == "get":
            print(inst.query(f"C{args.channel}:BSWV?").strip())
            print(inst.query(f"C{args.channel}:OUTP?").strip())
        elif args.action == "output":
            state = "ON" if args.state == "on" else "OFF"
            inst.write(f"C{args.channel}:OUTP {state}")
            print(inst.query(f"C{args.channel}:OUTP?").strip())
        elif args.action == "query":
            print(inst.query(args.command))
        elif args.action == "send":
            inst.write(args.command)
            print(f"sent: {args.command}")
        else:
            raise SystemExit(f"unknown action: {args.action}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (Vxi11Error, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
