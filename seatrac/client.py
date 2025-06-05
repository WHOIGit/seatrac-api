#!/usr/bin/env python3
import socket
import ssl
import traceback

from typing import Callable, Generator, Tuple, Union

from seatrac.protocol import SeaTracMessage


Socket = Union[socket.socket, ssl.SSLContext.sslsocket_class]
RecvCallable = Callable[[int], bytes]


def loop(sock: Socket, recv: RecvCallable) \
-> Generator[SeaTracMessage, None, None]:
    buffer = bytearray()
    try:
        while True:
            buffer.extend(recv(1024))
            if not (ready := SeaTracMessage.peek_length(buffer)):
                continue
            packet, buffer = buffer[:ready], buffer[ready:]
            try:
                yield SeaTracMessage.from_bytes(packet)
            except:
                print(f'Exception while handling packet {packet.hex()}:')
                traceback.print_exc()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        sock.close()


def listen(port=62001) -> Tuple[Socket, RecvCallable]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', port))
    return (sock, lambda l: sock.recvfrom(l)[0])


def connect(server: str, port: int, certfile: str, keyfile: str) \
-> Tuple[Socket, RecvCallable]:
    context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)

    # Butcher the security settings to allow us to connect
    ciphers = ":".join([
        "@SECLEVEL=1",
        "ALL",
    ])
    context.set_ciphers(ciphers)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE

    # Load the client certificate and key
    context.load_cert_chain(certfile=certfile, keyfile=keyfile)

    sock = socket.create_connection((server, port))
    sock = context.wrap_socket(sock, server_hostname=server)
    return (sock, sock.recv)
