#!/usr/bin/env python3
'''
Start a camera stream on the boat and write the video out as an H.264 Annex B
elementary stream, suitable for piping into ffmpeg (and from there into a
MediaMTX server for WebRTC / RTSP / HLS playback).
'''
import argparse
import enum
import select
import signal
import subprocess
import sys
import time

from typing import BinaryIO, Dict, List, Optional, Tuple

from seatrac.camera import (
    Frame,
    H264Reassembler,
    Link,
    LINK_SEND_STREAM,
    RESOLUTION_SIZES,
    image_ack,
    pc_port,
    start_stream,
    stop_stream,
)
from seatrac.client import Socket, RecvCallable, connect
from seatrac.h264 import NAL_TYPE_SPS, nal_type, parse_sps
from seatrac.protocol import (
    CameraLocation,
    CameraPeriod,
    CameraResolution,
    CameraStatusMessage,
    ImageDataMessage,
    SeaTracMessage,
    SendStream,
)


DEFAULT_SERVER = '3.213.3.223'


def log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def interrupt_on_sigterm() -> None:
    '''
    Treat SIGTERM like Ctrl-C, so that a stream started under a supervisor
    (MediaMTX signals with SIGINT, systemd with SIGTERM) is always stopped on
    the boat rather than left running on a metered link.
    '''
    def handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, handler)


EXIT_RESTART = 75  # EX_TEMPFAIL: nothing is wrong, but start us again


class GeometryChanged(Exception):
    pass


class Verdict(enum.Enum):
    PASS = 'pass'
    SKIP = 'skip'
    STOP = 'stop'


class GeometryGate:
    '''
    Keeps the picture size constant for the life of the stream.

    Two things make this necessary. The boat keeps sending the old resolution
    for up to a second after a start command, so a stream joined at the wrong
    moment begins with parameter sets that do not describe what was asked for.
    And the resolution can change underneath us if someone works the Dashboard
    at the same time. Either way the size changes mid-stream, which an RTSP
    session cannot follow, since it negotiates the parameter sets once.

    So: hold everything back until a parameter set for the requested resolution
    arrives, then treat any later change as a reason to start over.
    '''

    def __init__(self, resolution: CameraResolution, timeout: float) -> None:
        self.wanted = RESOLUTION_SIZES.get(resolution)
        self.deadline = time.monotonic() + timeout
        self.size: Optional[Tuple[int, int]] = None
        self.description = ''

    def admit(self, frame: Frame) -> Verdict:
        size = self._picture_size(frame)

        if self.size is None:
            # Until a parameter set commits us to a size, nothing else is worth
            # keeping: picture data belongs to a resolution we may be about to
            # discard, and a decoder has no parameter sets to decode it with.
            if size is None:
                return Verdict.SKIP
            if self.wanted and size != self.wanted \
            and time.monotonic() < self.deadline:
                log(f'Ignoring a {size[0]}x{size[1]} parameter set while '
                    f'waiting for {self.wanted[0]}x{self.wanted[1]}')
                return Verdict.SKIP
            self.size = size
            log(f'Picture size {size[0]}x{size[1]}'
                + ('' if size == self.wanted else ' (not what was requested)'))
            return Verdict.PASS

        if size is not None and size != self.size:
            self.description = (f'Picture size changed from '
                f'{self.size[0]}x{self.size[1]} to {size[0]}x{size[1]}')
            return Verdict.STOP

        return Verdict.PASS

    @staticmethod
    def _picture_size(frame: Frame) -> Optional[Tuple[int, int]]:
        if not frame.is_setup_frame or nal_type(frame.data) != NAL_TYPE_SPS:
            return None
        try:
            return parse_sps(frame.data)
        except ValueError as e:
            log(f'Could not read the parameter set: {e}')
            return None


class Feed:
    '''
    A connection to one of the boat's PC board ports, parsing messages as they
    arrive without blocking on a partial read.
    '''

    def __init__(self, link: Link, sock: Socket, recv: RecvCallable) -> None:
        self.link = link
        self.sock = sock
        self.recv = recv
        self.buffer = bytearray()

    def send(self, message: SeaTracMessage) -> None:
        self.sock.sendall(bytes(message))

    def read(self) -> List[SeaTracMessage]:
        data = self.recv(65536)
        if not data:
            raise ConnectionError(f'{self.link.value} connection closed')
        self.buffer.extend(data)

        received = []
        while ready := SeaTracMessage.peek_length(self.buffer):
            packet, self.buffer = self.buffer[:ready], self.buffer[ready:]
            received.append(SeaTracMessage.from_bytes(packet))
        return received


def open_feed(link: Link, args: argparse.Namespace) -> Feed:
    port = pc_port(link, args.boat)
    sock, recv = connect(args.server, port, *args.auth)
    log(f'Connected to {args.server}:{port} ({link.value})')
    return Feed(link, sock, recv)


def select_feeds(feeds: List[Feed], timeout: float) -> List[Feed]:
    '''
    Return the feeds with data waiting. Buffered TLS records are checked first,
    because select() only sees what has yet to be read off the socket.
    '''
    ready = [f for f in feeds if f.sock.pending()]
    if ready:
        return ready

    socks = {f.sock: f for f in feeds}
    readable, _, _ = select.select(list(socks), [], [], timeout)
    return [socks[s] for s in readable]


def open_output(args: argparse.Namespace) \
-> Tuple[BinaryIO, Optional[subprocess.Popen]]:
    '''
    Return the sink for the elementary stream, and the ffmpeg process feeding
    the RTSP server if one was started.

    The boat emits frames whenever it manages to, and an Annex B stream carries
    no timing of its own, so ffmpeg is asked to timestamp frames as they arrive.
    That keeps playback honest for a continuous stream and is what makes the
    periodic still image modes work at all.
    '''
    if args.publish is None:
        if args.output == '-':
            return sys.stdout.buffer, None
        return open(args.output, 'wb'), None

    command = [
        args.ffmpeg, '-hide_banner', '-loglevel', args.ffmpeg_loglevel,
        '-fflags', '+genpts', '-use_wallclock_as_timestamps', '1',
        '-f', 'h264', '-i', 'pipe:0',
        '-c:v', 'copy',
        '-f', 'rtsp', '-rtsp_transport', 'tcp', args.publish,
    ]
    log(f'Publishing to {args.publish}')
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    return process.stdin, process


def close_output(output: BinaryIO, ffmpeg: Optional[subprocess.Popen]) -> None:
    if ffmpeg is None:
        if output is not sys.stdout.buffer:
            output.close()
        return

    # Closing stdin lets ffmpeg flush and exit on its own; only insist if it
    # does not take the hint.
    try:
        output.close()
    except OSError:
        pass
    try:
        ffmpeg.wait(timeout=5)
    except subprocess.TimeoutExpired:
        ffmpeg.terminate()


def stream(args: argparse.Namespace) -> int:
    link = Link(args.link)
    send_stream = args.send_stream
    if send_stream is None:
        send_stream = LINK_SEND_STREAM[link]

    feed = open_feed(link, args)
    output, ffmpeg = open_output(args)

    command = start_stream(
        location=args.camera,
        send_stream=send_stream,
        resolution=args.resolution,
        period=args.period,
    )
    log(f'Starting {args.camera.name.lower()} camera over {link.value}: '
        f'{args.resolution.name.lower()} resolution, period {int(args.period)}, '
        f'send stream {int(send_stream)}')
    feed.send(command)

    reassembler = H264Reassembler()
    gate = GeometryGate(args.resolution, args.geometry_timeout)
    frames = written = 0
    started = last_report = time.monotonic()
    status = 0

    confirmed = False
    try:
        while True:
            for message in feed.read():
                # The boat reports its actual settings once a second. They lag
                # the command by about a second, so this is the first solid
                # sign that the camera is doing what was asked.
                if isinstance(message.payload, CameraStatusMessage) \
                and not confirmed:
                    confirmed = message.payload.resolution == args.resolution
                    if confirmed:
                        log('Boat confirms '
                            f'{args.resolution.name.lower()} resolution')

                for frame in reassembler.feed(message):
                    # Acknowledge everything we reassemble, including frames
                    # held back by the gate: the boat may be relying on it.
                    if not frame.is_setup_frame:
                        feed.send(image_ack(frame, send_stream))

                    admit = gate.admit(frame)
                    if admit is Verdict.STOP:
                        raise GeometryChanged(gate.description)
                    if admit is Verdict.SKIP:
                        continue

                    output.write(frame.data)
                    output.flush()
                    written += len(frame.data)
                    if not frame.is_setup_frame:
                        frames += 1

            now = time.monotonic()
            if args.stats and now - last_report >= 5:
                elapsed = now - started
                log(f'{frames} frames, {written / 1024:.0f} KiB, '
                    f'{frames / elapsed:.1f} fps, '
                    f'{written * 8 / elapsed / 1000:.0f} kbps, '
                    f'{reassembler.dropped_frames} dropped')
                last_report = now
    except KeyboardInterrupt:
        pass
    except GeometryChanged as e:
        log(f'{e}; exiting so the stream can be renegotiated')
        status = EXIT_RESTART
    except ConnectionError as e:
        log(str(e))
        status = 1
    finally:
        log(f'Stopping {args.camera.name.lower()} camera '
            f'({frames} frames, {reassembler.dropped_frames} dropped)')
        try:
            feed.send(stop_stream(args.camera, send_stream))
        except OSError as e:
            log(f'Failed to stop the stream: {e}')
        feed.sock.close()
        close_output(output, ffmpeg)

    return status


def probe(args: argparse.Namespace) -> int:
    '''
    Determine which Send Stream bit corresponds to which link.

    Bit 1 (Cell2) and bit 4 (Starlink) are known from a packet capture, but
    Certus has never been observed and Rev5 of the interface document predates
    the Starlink option, so its bitfield table cannot be taken at face value.
    Request a single still image at the lowest resolution for each candidate bit
    and see which link delivers it. One low resolution still per bit is cheap
    enough to run even over a metered link.
    '''
    feeds = {}
    for link in Link:
        try:
            feeds[link] = open_feed(link, args)
        except OSError as e:
            log(f'Could not connect for {link.value}: {e}')

    if not feeds:
        log('No connections available')
        return 1

    control = feeds.get(Link(args.link)) or next(iter(feeds.values()))
    log(f'Sending probes over the {control.link.value} connection')

    results: Dict[int, List[str]] = {}
    for bit in (SendStream.CELL, SendStream.CERTUS, SendStream.STARLINK):
        control.send(start_stream(
            location=args.camera,
            send_stream=bit,
            resolution=CameraResolution.LOW,
            period=CameraPeriod.SINGLE,
        ))

        seen = set()
        deadline = time.monotonic() + args.probe_timeout
        while (remaining := deadline - time.monotonic()) > 0:
            for feed in select_feeds(list(feeds.values()), remaining):
                for message in feed.read():
                    if isinstance(message.payload, ImageDataMessage):
                        seen.add(feed.link.value)

        control.send(stop_stream(args.camera, bit))
        results[int(bit)] = sorted(seen)
        log(f'  send stream {int(bit)} -> {", ".join(sorted(seen)) or "(nothing)"}')

    log('')
    log('Send Stream bitfield:')
    for bit, links in results.items():
        log(f'  {bit} = {", ".join(links) or "unknown (no image data received)"}')

    for feed in feeds.values():
        feed.sock.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--server', default=DEFAULT_SERVER)
    parser.add_argument('--auth', nargs=2, metavar=('CERT', 'KEY'),
        required=True)
    parser.add_argument('--boat', type=int, required=True,
        help='boat model number, e.g. 5 for SN5')
    parser.add_argument('--link', default=Link.STARLINK.value,
        choices=[l.value for l in Link],
        help='link that should carry the video')

    parser.add_argument('--camera', default=CameraLocation.BOW.name.lower(),
        type=lambda s: CameraLocation[s.upper()],
        choices=list(CameraLocation),
        metavar='{' + ','.join(c.name.lower() for c in CameraLocation) + '}')
    parser.add_argument('--resolution', default=CameraResolution.LOW.name.lower(),
        type=lambda s: CameraResolution[s.upper()],
        choices=list(CameraResolution),
        metavar='{' + ','.join(r.name.lower() for r in CameraResolution) + '}')
    parser.add_argument('--period', type=int, default=CameraPeriod.STREAM,
        help='0 for a continuous stream, 1-998 seconds between frames, '
             'or 999 for a single still image')

    parser.add_argument('--send-stream', type=int, default=None,
        help='override the Send Stream bitfield implied by --link')
    parser.add_argument('--output', default='-',
        help='where to write the Annex B stream (default: stdout)')
    parser.add_argument('--publish', metavar='RTSP_URL',
        help='run ffmpeg to publish the stream to this URL instead of '
             'writing it to --output')
    parser.add_argument('--ffmpeg', default='ffmpeg',
        help='ffmpeg executable used by --publish')
    parser.add_argument('--ffmpeg-loglevel', default='warning')
    parser.add_argument('--geometry-timeout', type=float, default=20.0,
        help='seconds to wait for a picture matching --resolution before '
             'accepting whatever the boat is sending')
    parser.add_argument('--stats', action='store_true',
        help='periodically report frame rate and bitrate on stderr')

    parser.add_argument('--probe', action='store_true',
        help='discover the Send Stream bit for each link and exit')
    parser.add_argument('--probe-timeout', type=float, default=15.0,
        help='seconds to wait for a still image during --probe')

    args = parser.parse_args()

    if args.output == '-' and args.publish is None \
    and sys.stdout.isatty() and not args.probe:
        parser.error('refusing to write video to a terminal; use --publish, '
                     '--output, or redirect stdout')

    interrupt_on_sigterm()
    return probe(args) if args.probe else stream(args)


if __name__ == '__main__':
    sys.exit(main())
