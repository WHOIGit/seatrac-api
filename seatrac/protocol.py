#!/usr/bin/env python3
import dataclasses
import datetime
import enum
import struct

from typing import Any, Dict, Optional, Tuple, Type, Union


class MessageType(enum.IntEnum):
    PING = 1  # inferred
    STATUS_REQUEST = 7
    STATUS_REPLY = 8
    REQUEST = 9
    REPLY = 10
    COMMAND = 11

class Relay(int):
    # TODO
    pass

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
    START = 2
    STOP = 3
    STOP_ALL = 4
    IMAGE_DATA_START = 5
    PTZ = 6


# Use a registry mapping board, sink, function -> payload class so that the
# SeaTracMessage deserializer can delegate to the appropriate payload class.
FunctionID = Union[
    COM_AIS_FunctionID,
    COM_SWITCHES_FunctionID,
    MOTOR_PROPULSION_FunctionID,
    PC_CAMERA_MGR_FunctionID,
    PMS_SWITCHES_FunctionID,
]
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

    relay: Relay
    msg_type: MessageType
    is_checksum_valid: bool = True
    board_id: Optional[BoardID] = None
    sink_id: Optional[SinkID] = None
    function_id: Optional[int] = None
    timestamp: Optional[datetime.datetime] = None
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

        relay = Relay(relay)
        msg_type = MessageType(msg_type)
        is_checksum_valid = verify_checksum(data)
        payload = data[header_size:-2]

        if not is_checksum_valid:
            return cls(
                relay=relay,
                msg_type=msg_type,
                is_checksum_valid=False,
                payload=payload
            )

        board_id = sink_id = function_id = None
        timestamp = None

        msg_type = MessageType(msg_type)
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
            relay=relay,
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
            self.relay,
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
        hundredths * 10000)
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

    check1 = (255 - (sum1 + sum2) % 255) % 255
    check2 = (255 - (sum1 + check1) % 255) % 255
    return (check1, check2)

def verify_checksum(data: bytes) -> bool:
    if len(data) < 2:
        return False
    return calculate_checksum(data[:-2]) == (data[-2], data[-1])
