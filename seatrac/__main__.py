#!/usr/bin/env python3
import argparse

from seatrac.client import listen, connect


def main():
    parser = argparse.ArgumentParser()

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--listen", action="store_true")
    group.add_argument("--connect", action="store_true")

    parser.add_argument("--server", default="3.213.3.223")
    parser.add_argument("--port", type=int)
    parser.add_argument("--auth", nargs=2, metavar=("CERT", "KEY"))

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
        return listen(args.port)

    if args.connect:
        return connect(args.server, args.port, *args.auth)


if __name__ == "__main__":
    main()
