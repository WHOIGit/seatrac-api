#!/usr/bin/env python3
import argparse
import socket

from seatrac.client import connect, listen, loop


def main():
    parser = argparse.ArgumentParser()

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--listen", action="store_true")
    group.add_argument("--connect", action="store_true")

    parser.add_argument("--server", default="3.213.3.223")
    parser.add_argument("--port", type=int)
    parser.add_argument("--auth", nargs=2, metavar=("CERT", "KEY"))
    parser.add_argument("--relay", nargs=2, metavar=("DEST", "PORT"))

    args = parser.parse_args()

    if not args.port:
        if args.listen:
            args.port = 62001
        else:
            # read-only 45107  # cell2 "listen" (42100) + SN8 (7)
            # read-write 42107  # cell2 "pilot" (45100) + SN8 (7)
            args.port = 45107

    if args.connect and not args.auth:
        parser.error('--auth is required with --connect')

    if args.listen:
        sock, recv = listen(args.port)
        print(f'Listening on UDP port {args.port}...')
    elif args.connect:
        sock, recv = connect(args.server, args.port, *args.auth)
        print(f'Connected to {args.server}:{args.port} over SSL...')

    relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for msg in loop(sock, recv):
        print(msg)
        if args.relay and msg.is_checksum_valid:
            relay.sendto(bytes(msg), (args.relay[0], int(args.relay[1])))


if __name__ == "__main__":
    main()
