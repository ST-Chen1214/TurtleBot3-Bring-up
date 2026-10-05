#!/usr/bin/env python3
"""Minimal Linux RFCOMM maintenance client for the local technician path."""
import argparse
import socket


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", required=True, help="Robot Bluetooth MAC")
    parser.add_argument("--channel", type=int, default=1)
    args = parser.parse_args()

    if not hasattr(socket, "AF_BLUETOOTH"):
        raise SystemExit("This Python build does not expose AF_BLUETOOTH")
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.connect((args.address, args.channel))
    f = sock.makefile("rwb", buffering=0)
    print(f.readline().decode().strip())
    try:
        while True:
            cmd = input("bt> ").strip()
            if cmd.lower() in {"quit", "exit"}:
                break
            f.write((cmd + "\n").encode())
            print(f.readline().decode().strip())
    finally:
        f.close()
        sock.close()


if __name__ == "__main__":
    main()
