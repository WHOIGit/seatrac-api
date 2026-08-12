#!/usr/bin/env python3
import dataclasses
import datetime
import enum
import struct

from typing import Any, Dict, List, Optional, Tuple, Type, Union


class MessageType(enum.IntEnum):
    PING = 1  # inferred
    STATUS_REQUEST = 7
    STATUS_REPLY = 8
    REQUEST = 9
    REPLY = 10
    COMMAND = 11

class Relay(enum.IntEnum):
    DEFAULT = 0
    PC = 3  # observed on every camera manager message from the Dashboard
    CAN_BUS = 8

class BoardID(enum.IntEnum):
    COM = 1
    BATTERY1 = 2
    BATTERY2 = 3
    BATTERY3 = 4
    CHARGERS1 = 5
    CHARGERS2 = 6
    PMS = 7
    MOTOR = 8
    RCREMOTE = 9
    PC = 10
    DBSERVER = 11
    WINCH = 12

class SinkID(enum.IntEnum):
    COM_WRITER = 1
    BATTERY1_WRITER = 2
    BATTERY2_WRITER = 3
    BATTERY3_WRITER = 4
    CHARGERS1_WRITER = 5
    CHARGERS2_WRITER = 6
    PMS_WRITER = 7
    MOTOR_WRITER = 8
    WINCH_WRITER = 9
    COM_TIME = 10
    COM_TRACER = 11
    COM_CAN_STATUS = 12
    COM_LOG_REPORTER = 13
    COM_REPORTER = 14
    COM_UDPPC_REPORTER = 15  # Reports just to PC
    COM_CELL1_REPORTER = 16
    COM_UDPSTARLINK_REPORTER = 17
    COM_SBD_REPORTER = 18
    COM_UDPCELL2_REPORTER = 19
    COM_SWITCHES = 20
    COM_CELL1_MODEM = 21
    COM_SBD_MODEM = 23
    COM_UDPCERTUS_REPORTER = 24
    COM_SDFAT = 25
    COM_AIS = 26
    COM_AIS_TARGET = 27  # Not used: AISTarget is not a MsgSink
    COM_STD_SENSOR = 28
    COM_ALERTER = 29
    BATTERY1_TIME = 30
    BATTERY1_TRACER = 31
    BATTERY1_CAN_STATUS = 32
    BATTERY1_PACK = 33
    BATTERY2_TIME = 35
    BATTERY2_TRACER = 36
    BATTERY2_CAN_STATUS = 37
    BATTERY2_PACK = 38
    BATTERY3_TIME = 40
    BATTERY3_TRACER = 41
    BATTERY3_CAN_STATUS = 42
    BATTERY3_PACK = 43
    CHARGERS1_TIME = 50
    CHARGERS1_TRACER = 51
    CHARGERS1_CAN_STATUS = 52
    CHARGERS1 = 53
    CHARGERS2_TIME = 55
    CHARGERS2_TRACER = 56
    CHARGERS2_CAN_STATUS = 57
    CHARGERS2 = 58
    PMS_TIME = 60
    PMS_TRACER = 61
    PMS_CAN_STATUS = 62
    PMS_SWITCHES = 63
    PMS_BMS = 64
    MOTOR_TIME = 70
    MOTOR_TRACER = 71
    MOTOR_CAN_STATUS = 72
    MOTOR_SWITCHES = 73
    MOTOR_PROP_MOTOR = 74
    MOTOR_STEER_MOTOR = 75
    MOTOR_ATTITUDE = 76
    MOTOR_GPS = 77
    MOTOR_WIND = 78
    MOTOR_BU_ATTITUDE = 79
    MOTOR_BU_GPS = 80
    MOTOR_WATERSPEED = 81
    MOTOR_PROPULSION = 82
    MOTOR_NAVIGATOR = 83
    MOTOR_LOGGER = 84
    MOTOR_SIMULATOR = 85
    MOTOR_IMU = 86
    MOTOR_WATERDEPTH = 87
    RCREMOTE_WRITER = 90
    PC_BSDRIVER = 92
    PC_STARLINK_REPORTER = 93
    PC_CERTUS_REPORTER = 94
    PC_CAMERA_MGR = 95
    PC_CELL2_REPORTER = 97
    PC_REMOTE_PC = 98
    PC_SDFAT = 99
    PC_TIME = 101
    PC_ADCP = 102
    PC_WAVE_SENSOR = 103
    PC_AML_SONDE = 104
    PC_NMEASTREAM = 105
    PC_ALERTER = 106
    DBSERVER_ALERTER = 100
    WINCH_MOTOR = 110
    WINCH_MANAGER = 111
    WINCH_TIME = 112
    WINCH_TRACER = 113
    WINCH_LOGGER = 114

class COM_SWITCHES_FunctionID(enum.IntEnum):
    SET = 0
    GET_BOARD_INFO = 1

class COM_AIS_FunctionID(enum.IntEnum):
    POSITION_REPORT = 2
    TARGET_DETAILS_UPDATE = 4

class PMS_SWITCHES_FunctionID(enum.IntEnum):
    SET = 0

class MOTOR_PROPULSION_FunctionID(enum.IntEnum):
    RUDDER_ANGLE_RPM = 1
    HEADING_RPM = 2
    CHANGE_STATE = 5
    MOVE_TO_POINT = 7

class PC_CAMERA_MGR_FunctionID(enum.IntEnum):
    START_ALL = 0
    IMAGE_ACK = 1  # undocumented; see CameraImageAckCommand
    START = 2
    STOP = 3
    STOP_ALL = 4
    IMAGE_DATA_START = 5
    PTZ = 6

class CameraLocation(enum.IntEnum):
    BOW = 0
    PORT = 1
    STARBOARD = 2
    STERN = 3
    PTZ_IR = 4
    PTZ_EO = 5

class CameraResolution(enum.IntEnum):
    LOW = 0     # 360p
    MEDIUM = 1  # 720p
    HIGH = 2    # 1080p

class SendStream(enum.IntFlag):
    '''
    Bitfield identifying the link(s) used to upload image data.

    CELL and STARLINK are confirmed from a decrypted capture of the Dashboard.
    Rev5 of the interface document calls bit 4 the "Silvus Link"; the Dashboard
    uses it for Starlink. CERTUS is taken from the document and has not been
    observed; seatrac.video --probe can confirm it.
    '''
    NONE = 0
    CELL = 1
    CERTUS = 2
    STARLINK = 4

class CameraPeriod(enum.IntEnum):
    '''
    Frame timing. Values between the two extremes are seconds between frames.
    '''
    STREAM = 0  # continuous, nominally 10 fps
    SINGLE = 999  # a single still image


# Use a registry mapping board, sink, function -> payload class so that the
# SeaTracMessage deserializer can delegate to the appropriate payload class.
FunctionID = Union[
    COM_AIS_FunctionID,
    COM_SWITCHES_FunctionID,
    MOTOR_PROPULSION_FunctionID,
    PC_CAMERA_MGR_FunctionID,
    PMS_SWITCHES_FunctionID,
]
CameraFrameTiming = Union[CameraPeriod, int]
MessageKey = Tuple[MessageType, Optional[BoardID], Optional[SinkID],
    Optional[FunctionID]]
_PAYLOAD_REGISTRY: Dict[MessageKey, Type] = {}


def register_message(
    msg_type: MessageType,
    board_id: Optional[BoardID] = None,
    sink_id: Optional[SinkID] = None,
    function_id: Optional[FunctionID] = None,
) -> Any:
    def decorator(cls: Type) -> Type:
        _PAYLOAD_REGISTRY[(msg_type, board_id, sink_id, function_id)] = cls
        return cls
    return decorator


@dataclasses.dataclass
class SeaTracMessage:
    HEADER_PATTERN = '<BBHBB'
    COMMAND_PATTERN = '<BBB'
    SYNC_BYTES = (0x00, 0xFF)

    outbound_relay: Union[Relay, int]
    return_relay: Union[Relay, int]
    msg_type: Union[MessageType, int]

    board_id: Optional[BoardID] = None
    sink_id: Optional[SinkID] = None
    function_id: Optional[int] = None
    timestamp: Optional[datetime.datetime] = None

    is_checksum_valid: bool = True

    payload: Any = None

    @classmethod
    def peek_length(cls, buffer: Union[bytearray, bytes]) -> Optional[int]:
        '''
        If a message is ready to be consumed from the buffer, return the number
        of bytes. Otherwise, return None.
        '''
        sync1 = buffer[0] if len(buffer) > 0 else None
        sync2 = buffer[1] if len(buffer) > 1 else None
        if (sync1, sync2) == (None, None):
            return None
        elif (sync1, sync2) == (cls.SYNC_BYTES[0], None):
            return None
        elif (sync1, sync2) == cls.SYNC_BYTES:
            pass
        else:
            raise ValueError('Invalid sync bytes')

        header_size = struct.calcsize(cls.HEADER_PATTERN)
        if len(buffer) < header_size + 2:
            return None

        _, _, length, _, _ = struct.unpack_from(cls.HEADER_PATTERN, buffer)
        if len(buffer) < header_size + length + 2:
            return None
        return header_size + length + 2

    @classmethod
    def from_bytes(cls, data: bytes) -> 'SeaTracMessage':
        header_size = struct.calcsize(cls.HEADER_PATTERN)
        if len(data) < header_size + 2:
            raise ValueError('Invalid data length')

        sync1, sync2, length, relay, msg_type = struct.unpack_from(
            cls.HEADER_PATTERN, data)
        if (sync1, sync2) != cls.SYNC_BYTES:
            raise ValueError('Invalid sync bytes')
        if len(data) != length + header_size + 2:
            raise ValueError('Invalid data length')

        try:
            outbound_relay = Relay(relay & 0x0F)
        except ValueError:
            outbound_relay = relay & 0x0F

        try:
            return_relay = Relay(relay >> 4)
        except ValueError:
            return_relay = relay >> 4

        try:
            msg_type = MessageType(msg_type)
        except ValueError:
            pass  # fallback to using the raw integer value

        payload = data[header_size:-2]

        is_checksum_valid = verify_checksum(data)
        if not is_checksum_valid:
            return cls(
                outbound_relay=outbound_relay,
                return_relay=return_relay,
                msg_type=msg_type,
                is_checksum_valid=False,
                payload=payload
            )

        board_id = sink_id = function_id = None
        timestamp = None

        if msg_type in (MessageType.REQUEST,
                        MessageType.REPLY,
                        MessageType.COMMAND):
            b, s, f = struct.unpack_from(cls.COMMAND_PATTERN, payload)
            board_id, sink_id, function_id = BoardID(b), SinkID(s), f
            payload = payload[struct.calcsize(cls.COMMAND_PATTERN):]
        elif msg_type == MessageType.STATUS_REPLY:
            sink_id = SinkID(payload[0])
            dtsize = struct.calcsize(datetime_PATTERN)
            timestamp = datetime_frombytes(payload[1:1+dtsize])
            payload = payload[1+dtsize:]

        # Dispatch to registered payload deserializer if available
        key = (msg_type, board_id, sink_id, function_id)
        payload_type = _PAYLOAD_REGISTRY.get(key)
        if payload_type is not None:
            payload = payload_type.from_bytes(payload)

        return cls(
            outbound_relay=outbound_relay,
            return_relay=return_relay,
            msg_type=msg_type,
            is_checksum_valid=is_checksum_valid,
            board_id=board_id,
            sink_id=sink_id,
            function_id=function_id,
            timestamp=timestamp,
            payload=payload,
        )

    def __bytes__(self) -> bytes:
        # Construct appropriate header per the message type
        msg_type_header = b''
        if self.msg_type in (MessageType.REQUEST,
                             MessageType.REPLY,
                             MessageType.COMMAND):
            msg_type_header = struct.pack(
                self.COMMAND_PATTERN,
                self.board_id,
                self.sink_id,
                self.function_id,
            )
        elif self.msg_type == MessageType.STATUS_REPLY:
            msg_type_header = bytes([
                self.sink_id,
                *datetime___bytes__(self.timestamp)
            ])

        body = bytes(self.payload) if self.payload is not None else b''

        header = struct.pack(
            self.HEADER_PATTERN,
            0x00, 0xFF,
            len(msg_type_header) + len(body),
            (self.return_relay << 4) | self.outbound_relay,
            self.msg_type,
        )

        packet = header + msg_type_header + body
        packet += bytes(calculate_checksum(packet))
        return packet



@register_message(MessageType.STATUS_REPLY, sink_id=SinkID.PMS_BMS)
@dataclasses.dataclass
class PowerLevelMessage:
    PATTERN = '<20shhHH4s'

    pack_current: float
    load_current: float
    pack_voltage: float
    soc_percentage: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'PowerLevelMessage':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')

        _, pack_current, load_current, pack_voltage, soc_percentage, _ = \
            struct.unpack(cls.PATTERN, data)

        return cls(
            pack_current=pack_current * 0.002,
            load_current=load_current * 0.002,
            pack_voltage=pack_voltage * 0.001,
            soc_percentage=soc_percentage * 0.002
        )

    def __bytes__(self) -> bytes:
        pack_current = int(self.pack_current / 0.002)
        load_current = int(self.load_current / 0.002)
        pack_voltage = int(self.pack_voltage / 0.001)
        soc_percentage = int(self.soc_percentage / 0.002)
        return struct.pack(self.PATTERN, b'', pack_current, load_current,
            pack_voltage, soc_percentage, b'')


@register_message(
    MessageType.COMMAND,
    board_id=BoardID.COM,
    sink_id=SinkID.COM_SWITCHES,
    function_id=COM_SWITCHES_FunctionID.SET,
)
@register_message(
    MessageType.COMMAND,
    board_id=BoardID.PMS,
    sink_id=SinkID.PMS_SWITCHES,
    function_id=PMS_SWITCHES_FunctionID.SET,
)
@dataclasses.dataclass
class SwitchSetCommand:
    PATTERN = '<BB'

    switch: int
    state: bool

    @classmethod
    def from_bytes(cls, data: bytes) -> 'SwitchSetCommand':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')

        switch, state = struct.unpack(cls.PATTERN, data)
        return cls(switch=switch, state=bool(state))

    def __bytes__(self) -> bytes:
        return struct.pack(self.PATTERN, self.switch, int(self.state))


@register_message(MessageType.STATUS_REPLY, sink_id=SinkID.PMS_SWITCHES)
@dataclasses.dataclass
class PMSSwitchStatusMessage:
    PATTERN = '<18sBBB'
    NUM_SWITCHES = 18

    # Encoding of the switch states, least-significant bit first:
    #   [ Nav Light, COM, Motor, CAN, 12V Bus, 12V #1, 12V #2, 12V #3 ]
    #   [ 24V Bus, 24V #1, 24V #2, 24V #3, VBat #1, VBat #2, VBat #3, 24V #4 ]
    #   [ Anchor Light, VBat #4, ... reserved ... ]

    states: List[bool] = dataclasses.field(
        # Sneakily add a parameter n with default value NUM_SWITCHES to capture
        # its value at class definition time.
        default_factory=lambda n=NUM_SWITCHES: [False] * n
    )

    @classmethod
    def from_bytes(cls, data: bytes) -> 'PMSSwitchStatusMessage':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')

        _, a, b, c = struct.unpack(cls.PATTERN, data)
        packed = int.from_bytes([a, b, c], 'little')
        states = [bool((packed >> i) & 1) for i in range(cls.NUM_SWITCHES)]
        return cls(states=states)

    def __bytes__(self) -> bytes:
        if len(self.states) != self.NUM_SWITCHES:
            raise ValueError(f'Expected {self.NUM_SWITCHES} states')

        # Pack the states into a little-endian integer
        packed = sum(int(b) << i for i, b in enumerate(self.states))
        n = (self.NUM_SWITCHES + 7) // 8  # number of bytes needed
        return struct.pack(self.PATTERN, b'', *packed.to_bytes(n, 'little'))


@register_message(
    MessageType.COMMAND,
    board_id=BoardID.PC,
    sink_id=SinkID.PC_CAMERA_MGR,
    function_id=PC_CAMERA_MGR_FunctionID.START,
)
@dataclasses.dataclass
class CameraStartStreamCommand:
    PATTERN = '<BBHB'

    location: Union[CameraLocation, int]
    resolution: Union[CameraResolution, int] = CameraResolution.LOW
    period: CameraFrameTiming = CameraPeriod.STREAM
    send_stream: Union[SendStream, int] = SendStream.NONE

    @classmethod
    def from_bytes(cls, data: bytes) -> 'CameraStartStreamCommand':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')

        location, resolution, period, send_stream = \
            struct.unpack(cls.PATTERN, data)
        return cls(
            location=_coerce(CameraLocation, location),
            resolution=_coerce(CameraResolution, resolution),
            period=_coerce(CameraPeriod, period),
            send_stream=SendStream(send_stream),
        )

    def __bytes__(self) -> bytes:
        return struct.pack(self.PATTERN, self.location, self.resolution,
            self.period, self.send_stream)


@register_message(
    MessageType.COMMAND,
    board_id=BoardID.PC,
    sink_id=SinkID.PC_CAMERA_MGR,
    function_id=PC_CAMERA_MGR_FunctionID.STOP,
)
@dataclasses.dataclass
class CameraStopStreamCommand:
    PATTERN = '<BB'

    location: Union[CameraLocation, int]
    send_stream: Union[SendStream, int] = SendStream.NONE

    @classmethod
    def from_bytes(cls, data: bytes) -> 'CameraStopStreamCommand':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')

        location, send_stream = struct.unpack(cls.PATTERN, data)
        return cls(
            location=_coerce(CameraLocation, location),
            send_stream=SendStream(send_stream),
        )

    def __bytes__(self) -> bytes:
        return struct.pack(self.PATTERN, self.location, self.send_stream)


@register_message(MessageType.STATUS_REPLY, sink_id=SinkID.PC_CAMERA_MGR)
@dataclasses.dataclass
class CameraStatusMessage:
    '''
    Once-per-second report of what the cameras are actually doing, sent while a
    stream is running on the connection carrying it.

    This is how the Dashboard knows a Start Stream command took effect: the
    reported resolution lags the command by a second or so, which is the delay
    before its resolution buttons stop being highlighted. Only the two fields
    below have been identified; the rest of the 186 byte payload is kept as-is.
    '''
    PATTERN = '<3sBB'

    resolution: Union[CameraResolution, int]
    send_stream: Union[SendStream, int]
    remainder: bytes = b''

    @classmethod
    def from_bytes(cls, data: bytes) -> 'CameraStatusMessage':
        header_size = struct.calcsize(cls.PATTERN)
        if len(data) < header_size:
            raise ValueError('Invalid data length')

        _, resolution, send_stream = struct.unpack_from(cls.PATTERN, data)
        return cls(
            resolution=_coerce(CameraResolution, resolution),
            send_stream=SendStream(send_stream),
            remainder=data[header_size:],
        )

    def __bytes__(self) -> bytes:
        return struct.pack(self.PATTERN, b'', self.resolution,
            self.send_stream) + self.remainder


@register_message(
    MessageType.COMMAND,
    board_id=BoardID.PC,
    sink_id=SinkID.PC_CAMERA_MGR,
    function_id=PC_CAMERA_MGR_FunctionID.IMAGE_ACK,
)
@dataclasses.dataclass
class CameraImageAckCommand:
    '''
    Acknowledgement of one received image frame, identified by the timestamp the
    boat put in the frame's first chunk.

    This message is not in the interface document. It was found in a packet
    capture of the Dashboard, which sends exactly one per frame it reassembles,
    a few milliseconds after the last chunk arrives, and sends none when no
    stream is running. Whether the boat requires it in order to keep sending is
    not known, so it is best to imitate the Dashboard and send it.
    '''
    PATTERN = '<BB'

    location: Union[CameraLocation, int]
    send_stream: Union[SendStream, int]
    timestamp: Optional[datetime.datetime] = None

    @classmethod
    def from_bytes(cls, data: bytes) -> 'CameraImageAckCommand':
        header_size = struct.calcsize(cls.PATTERN)
        if len(data) != header_size + struct.calcsize(datetime_PATTERN):
            raise ValueError('Invalid data length')

        location, send_stream = struct.unpack_from(cls.PATTERN, data)
        return cls(
            location=_coerce(CameraLocation, location),
            send_stream=SendStream(send_stream),
            timestamp=datetime_frombytes(data[header_size:]),
        )

    def __bytes__(self) -> bytes:
        return struct.pack(self.PATTERN, self.location, self.send_stream) \
            + datetime___bytes__(self.timestamp)


@dataclasses.dataclass
class ImageAttitude:
    '''
    Camera and vessel geometry accompanying the first chunk of an image frame.
    The distortion correction fields are not used on the PTZ cameras, and the
    PTZ fields are not meaningful for the fixed cameras.
    '''
    PATTERN = '<9f'

    strength: float = 0.0
    zoom: float = 0.0
    camera_height: float = 0.0  # feet above the water
    heading: float = 0.0  # the vessel's heading
    ptz_azimuth: float = 0.0
    ptz_elevation: float = 0.0
    ptz_view_angle: float = 0.0  # total horizontal field of view
    image_rotation: float = 0.0
    image_height: float = 0.0

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ImageAttitude':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')
        return cls(*struct.unpack(cls.PATTERN, data))

    def __bytes__(self) -> bytes:
        return struct.pack(self.PATTERN,
            *(getattr(self, f.name) for f in dataclasses.fields(self)))


@register_message(
    MessageType.REPLY,
    board_id=BoardID.PC,
    sink_id=SinkID.PC_CAMERA_MGR,
    function_id=PC_CAMERA_MGR_FunctionID.IMAGE_DATA_START,
)
@dataclasses.dataclass
class ImageDataMessage:
    '''
    One chunk of H.264 data uploaded by the boat.

    A message is one of three shapes, distinguished by n_chunks and chunk:

      - a setup frame (n_chunks == 1, chunk == 0) carrying parameter sets that
        must be handed to the decoder before any image data;
      - the first chunk of an image frame (chunk == 1), which also carries the
        attitude and the capture timestamp; or
      - a continuation chunk (chunk > 1), which is data only.

    Note that chunk numbering for image frames is 1-based, so the last chunk is
    chunk == n_chunks.
    '''
    START_PATTERN = '<BBHH'
    SIZE_PATTERN = '<H'

    location: Union[CameraLocation, int]
    send_stream: Union[SendStream, int]
    n_chunks: int
    chunk: int
    data: bytes = b''

    # Present only on the first chunk of an image frame
    attitude: Optional[ImageAttitude] = None
    timestamp: Optional[datetime.datetime] = None

    @property
    def is_setup_frame(self) -> bool:
        return self.chunk == 0

    @classmethod
    def from_bytes(cls, data: bytes) -> 'ImageDataMessage':
        start_size = struct.calcsize(cls.START_PATTERN)
        if len(data) < start_size:
            raise ValueError('Invalid data length')

        location, send_stream, n_chunks, chunk = \
            struct.unpack_from(cls.START_PATTERN, data)
        offset = start_size

        attitude = timestamp = None
        if chunk == 1:
            attsize = struct.calcsize(ImageAttitude.PATTERN)
            dtsize = struct.calcsize(datetime_PATTERN)
            attitude = ImageAttitude.from_bytes(data[offset:offset+attsize])
            offset += attsize
            timestamp = datetime_frombytes(data[offset:offset+dtsize])
            offset += dtsize

        size_size = struct.calcsize(cls.SIZE_PATTERN)
        if len(data) < offset + size_size:
            raise ValueError('Invalid data length')

        size, = struct.unpack_from(cls.SIZE_PATTERN, data, offset)
        offset += size_size

        if len(data) - offset != size:
            raise ValueError('Image data size does not match message length')

        return cls(
            location=_coerce(CameraLocation, location),
            send_stream=SendStream(send_stream),
            n_chunks=n_chunks,
            chunk=chunk,
            data=data[offset:offset+size],
            attitude=attitude,
            timestamp=timestamp,
        )

    def __bytes__(self) -> bytes:
        packet = struct.pack(self.START_PATTERN, self.location,
            self.send_stream, self.n_chunks, self.chunk)
        if self.chunk == 1:
            attitude = self.attitude or ImageAttitude()
            packet += bytes(attitude) + datetime___bytes__(self.timestamp)
        packet += struct.pack(self.SIZE_PATTERN, len(self.data)) + self.data
        return packet


def _coerce(enum_type: Type, value: int) -> Any:
    '''
    Return value as a member of enum_type, or the raw integer if the value is
    not a known member.
    '''
    try:
        return enum_type(value)
    except ValueError:
        return value


# Can't monkey patch the datetime.datetime class :(
datetime_PATTERN = '<HBBBBBB'

def datetime_frombytes(data: bytes) -> Optional[datetime.datetime]:
    if len(data) != struct.calcsize(datetime_PATTERN):
        raise ValueError('Invalid data length')

    if not any(data):
        return None

    year, month, day, hour, minute, second, hundredths = \
        struct.unpack(datetime_PATTERN, data)
    timestamp = datetime.datetime(year, month, day, hour, minute, second,
        hundredths * 10000, tzinfo=datetime.timezone.utc)
    return timestamp

def datetime___bytes__(self: Optional[datetime.datetime]) -> bytes:
    if self is None:
        return b'\x00' * struct.calcsize(datetime_PATTERN)
    return struct.pack(datetime_PATTERN, self.year, self.month, self.day,
        self.hour, self.minute, self.second, self.microsecond // 10000)


def calculate_checksum(data: bytes) -> Tuple[int, int]:
    sum1 = sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255

    check1 = 255 - ((sum1 + sum2) % 255)
    check2 = 255 - ((sum1 + check1) % 255)
    return (check1, check2)

def verify_checksum(data: bytes) -> bool:
    if len(data) < 2:
        return False
    return calculate_checksum(data[:-2]) == (data[-2], data[-1])
