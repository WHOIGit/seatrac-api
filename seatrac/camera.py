#!/usr/bin/env python3
'''
Camera stream control and H.264 reassembly.

The boat uploads video as a series of Image Data messages (board 10, sink 95,
function 5). Parameter sets arrive as standalone "setup frames"; picture data is
split into chunks that must be concatenated before being handed to a decoder.
This module turns that message stream back into an Annex B elementary stream.
'''
import dataclasses
import datetime
import enum

from typing import Any, Dict, List, Optional, Tuple, Union

from seatrac.protocol import (
    BoardID,
    CameraFrameTiming,
    CameraImageAckCommand,
    CameraLocation,
    CameraPeriod,
    CameraResolution,
    CameraStartStreamCommand,
    CameraStopStreamCommand,
    ImageAttitude,
    ImageDataMessage,
    MessageType,
    PC_CAMERA_MGR_FunctionID,
    Relay,
    SeaTracMessage,
    SendStream,
    SinkID,
)


# The Dashboard app sends camera commands over the PC connection belonging to
# the link that should carry the video, and the image data comes back on that
# same connection. Base port numbers are from docs/ports.md; add the boat number
# (SN8 = 7, SN5 = 4, i.e. one less than the model number).
class Link(enum.Enum):
    CELL2 = 'cell2'
    CERTUS = 'certus'
    STARLINK = 'starlink'

LINK_PC_PORTS: Dict[Link, Tuple[int, int]] = {
    # link: (read-only "listen" base, read-write "pilot" base)
    Link.CELL2: (46000, 43000),
    Link.CERTUS: (49000, 48000),
    Link.STARLINK: (49100, 48100),
}

# Which bit of the Send Stream bitfield selects each link. Cell2 and Starlink
# are confirmed from a packet capture; Certus is inferred from the interface
# document and can be checked with seatrac.video --probe.
LINK_SEND_STREAM: Dict[Link, SendStream] = {
    Link.CELL2: SendStream.CELL,
    Link.CERTUS: SendStream.CERTUS,
    Link.STARLINK: SendStream.STARLINK,
}


def pc_port(link: Link, boat: int, pilot: bool = True) -> int:
    '''
    Return the server port carrying PC board traffic for the given link and boat
    (5 for SN5, 8 for SN8, ...).
    '''
    listen_base, pilot_base = LINK_PC_PORTS[link]
    return (pilot_base if pilot else listen_base) + boat - 1


def _camera_command(function_id: PC_CAMERA_MGR_FunctionID, payload: Any) \
-> SeaTracMessage:
    '''
    Wrap a camera manager payload. The interface document gives the relay byte
    as 0, but the Dashboard sends every camera manager message with an outbound
    relay of 3, which is what the boat is known to accept.
    '''
    return SeaTracMessage(
        outbound_relay=Relay.PC,
        return_relay=Relay.DEFAULT,
        msg_type=MessageType.COMMAND,
        board_id=BoardID.PC,
        sink_id=SinkID.PC_CAMERA_MGR,
        function_id=function_id,
        payload=payload,
    )


def start_stream(
    location: Union[CameraLocation, int],
    send_stream: Union[SendStream, int],
    resolution: Union[CameraResolution, int] = CameraResolution.LOW,
    period: CameraFrameTiming = CameraPeriod.STREAM,
) -> SeaTracMessage:
    '''
    Build the command to start (or restart with new settings) a camera stream.
    '''
    return _camera_command(
        PC_CAMERA_MGR_FunctionID.START,
        CameraStartStreamCommand(
            location=location,
            resolution=resolution,
            period=period,
            send_stream=send_stream,
        ),
    )


def stop_stream(
    location: Union[CameraLocation, int],
    send_stream: Union[SendStream, int],
) -> SeaTracMessage:
    '''
    Build the command to stop one camera stream on one or more links.
    '''
    return _camera_command(
        PC_CAMERA_MGR_FunctionID.STOP,
        CameraStopStreamCommand(location=location, send_stream=send_stream),
    )


def stop_all_streams() -> SeaTracMessage:
    '''
    Build the command to stop every camera stream on every link.
    '''
    return _camera_command(PC_CAMERA_MGR_FunctionID.STOP_ALL, b'')


def image_ack(frame: 'Frame', send_stream: Union[SendStream, int]) \
-> SeaTracMessage:
    '''
    Build the acknowledgement the Dashboard sends after reassembling a frame.
    '''
    return _camera_command(
        PC_CAMERA_MGR_FunctionID.IMAGE_ACK,
        CameraImageAckCommand(
            location=frame.location,
            send_stream=send_stream,
            timestamp=frame.timestamp,
        ),
    )


# Picture size the boat actually encodes at each resolution setting, measured
# from the parameter sets in a packet capture.
RESOLUTION_SIZES: Dict[CameraResolution, Tuple[int, int]] = {
    CameraResolution.LOW: (640, 360),
    CameraResolution.MEDIUM: (1280, 720),
    CameraResolution.HIGH: (1920, 1080),
}


ANNEXB_START_CODE = b'\x00\x00\x00\x01'


def as_annex_b(data: bytes) -> bytes:
    '''
    Prefix a start code unless the payload already carries one. The boat sends
    bare NAL units, so in practice a start code is always added; the check is
    there in case some camera or firmware does it differently.
    '''
    if data.startswith(ANNEXB_START_CODE) or data.startswith(b'\x00\x00\x01'):
        return data
    return ANNEXB_START_CODE + data


@dataclasses.dataclass
class Frame:
    '''
    A fully reassembled unit of the elementary stream, ready for a decoder.
    '''
    location: Union[CameraLocation, int]
    data: bytes  # Annex B, with start code
    is_setup_frame: bool = False
    attitude: Optional[ImageAttitude] = None
    timestamp: Optional[datetime.datetime] = None


@dataclasses.dataclass
class _Partial:
    n_chunks: int
    next_chunk: int
    chunks: List[bytes]
    attitude: Optional[ImageAttitude]
    timestamp: Optional[datetime.datetime]


class H264Reassembler:
    '''
    Accumulates Image Data messages and emits whole frames.

    Each camera location is tracked independently, so a single connection
    carrying several streams can be demultiplexed. An image frame whose chunks
    do not arrive in order, or that is interrupted by the start of another
    frame, is dropped in its entirety; dropped_frames counts those.
    '''

    def __init__(self) -> None:
        self._partial: Dict[int, _Partial] = {}
        self.dropped_frames = 0

    def feed(self, message: SeaTracMessage) -> List[Frame]:
        '''
        Consume one message and return the frames it completed, if any. Messages
        that are not image data are ignored, so the whole receive stream can be
        fed in without filtering.
        '''
        payload = message.payload
        if not isinstance(payload, ImageDataMessage):
            return []
        if not message.is_checksum_valid:
            return []

        # Setup frames stand alone and are always passed straight through.
        if payload.is_setup_frame:
            return [Frame(
                location=payload.location,
                data=as_annex_b(payload.data),
                is_setup_frame=True,
            )]

        location = int(payload.location)
        partial = self._partial.get(location)

        if payload.chunk == 1:
            if partial is not None:
                self.dropped_frames += 1  # previous frame never completed
            partial = _Partial(
                n_chunks=payload.n_chunks,
                next_chunk=2,
                chunks=[payload.data],
                attitude=payload.attitude,
                timestamp=payload.timestamp,
            )
            self._partial[location] = partial
        elif partial is None:
            return []  # joined mid-frame, or the first chunk was lost
        elif payload.chunk != partial.next_chunk \
        or payload.n_chunks != partial.n_chunks:
            del self._partial[location]
            self.dropped_frames += 1
            return []
        else:
            partial.chunks.append(payload.data)
            partial.next_chunk += 1

        if partial.next_chunk <= partial.n_chunks:
            return []

        del self._partial[location]
        return [Frame(
            location=payload.location,
            data=as_annex_b(b''.join(partial.chunks)),
            attitude=partial.attitude,
            timestamp=partial.timestamp,
        )]
