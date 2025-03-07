#!/usr/bin/env python3
import dataclasses
import datetime
import enum
import math
import struct

from typing import Optional, Tuple


class MessageType(enum.IntEnum):
    PING = 1  # inferred
    STATUS_REQUEST = 7
    STATUS_REPLY = 8
    REQUEST = 9
    REPLY = 10
    COMMAND = 11

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


@dataclasses.dataclass
class PowerLevelMessage:
    PATTERN = '<hhHH'

    pack_current: float
    load_current: float
    pack_voltage: float
    soc_percentage: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'PowerLevelMessage':
        if len(data) != struct.calcsize(cls.PATTERN):
            raise ValueError('Invalid data length')

        (self.pack_current, self.load_current, self.pack_voltage,
            self.soc_percentage) = struct.unpack(cls.PATTERN, data)

        self.pack_current = self.pack_current * 0.002,
        self.load_current = self.load_current * 0.002,
        self.pack_voltage = self.pack_voltage * 0.001,
        self.soc_percentage = self.soc_percentage * 0.002

    def __bytes__(self) -> bytes:
        pack_current = int(self.pack_current / 0.002)
        load_current = int(self.load_current / 0.002)
        pack_voltage = int(self.pack_voltage / 0.001)
        soc_percentage = int(self.soc_percentage / 0.002)
        return struct.pack(self.PATTERN, pack_current, load_current,
            pack_voltage, soc_percentage)


@dataclasses.dataclass
class GPSPositionMessage:
    timestamp: datetime.datetime
    latitude: float
    longitude: float
    speed: float
    heading: float
    current_speed: float
    current_heading: float
    wind_speed: float
    wind_heading: float


@dataclasses.dataclass
class SwitchSetCommand:
    switch: int
    state: bool


# Can't monkey patch the datetime.datetime class :(
datetime_PATTERN = '<HBBBBBB'

def datetime_frombytes(data: bytes) -> datetime.datetime:
    if len(data) != struct.calcsize(datetime_PATTERN):
        raise ValueError('Invalid data length')

    year, month, day, hour, minute, second, hundredths = struct.unpack(datetime_PATTERN, data)
    timestamp = datetime.datetime(year + 1900, month, day, hour, minute, second,
        hundredths * 1e4)
    return timestamp

def datetime___bytes__(self) -> bytes:
    return struct.pack(datetime_PATTERN, self.year - 1900, self.month, self.day,
        self.hour, self.minute, self.second, self.microsecond // 1e4)


def verify_checksum(data: bytes) -> bool:
    if len(data) < 2:
        return False

    sum1 = sum2 = 0
    for byte in data[:-2]:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255

    check1 = (255 - (sum1 + sum2) % 255) % 255
    check2 = (255 - (sum1 + check1) % 255) % 255

    return (check1, check2) == (data[-2], data[-1])


def parse_power_level(data: bytes) -> Optional[PowerLevelMessage]:
    if len(data) < 47:
        return None

    if not verify_checksum(data):
        return None

    message_type = data[5]
    if message_type != MessageType.STATUS_REPLY:
        return None

    sink_id = data[6]
    if sink_id != SinkID.PMS_BMS:
        return None

    pack_current, load_current, pack_voltage, soc_percentage = \
        struct.unpack_from('<hhHH', data, 33)

    return PowerLevelMessage(
        pack_current=pack_current * 0.002,
        load_current=load_current * 0.002,
        pack_voltage=pack_voltage * 0.001,
        soc_percentage=soc_percentage * 0.002
    )


def parse_gps_position(data: bytes) -> Optional[GPSPositionMessage]:
    if len(data) < 47:
        return None

    if not verify_checksum(data):
        return None

    message_type = data[5]
    if message_type != MessageType.STATUS_REPLY:
        return None

    sink_id = data[6]
    if sink_id not in (SinkID.MOTOR_GPS, SinkID.MOTOR_BU_GPS):
        return None

    offset = 7
    timestamp = datetime_frombytes(data[offset:])
    offset += struct.calcsize(datetime_PATTERN)

    latitude, longitude = struct.unpack_from('<dd', data, offset)
    offset += 16

    kts, heading, current_kts, current_heading, wind_kts, wind_heading = \
        struct.unpack_from('<HHHHHH', data, offset)

    return GPSPositionMessage(
        timestamp=timestamp,
        latitude=math.degrees(latitude),
        longitude=math.degrees(longitude),
        speed=kts * 0.002,
        heading=heading * 0.01,
        current_speed=current_kts * 0.002,
        current_heading=current_heading * 0.01,
        wind_speed=wind_kts * 0.002,
        wind_heading=wind_heading * 0.01
    )


def pop_message(buffer) -> Tuple[bytearray, bytearray]:
    # Look for the sync sequence
    sync_offset = buffer.find(b'\x00\xff')
    if sync_offset == -1:
        buffer.clear()
        return None, buffer
    if sync_offset >= 1:
        buffer = buffer[sync_offset:]

    # Look for a complete message
    if len(buffer) < 4:
        return None, buffer
    length, = struct.unpack_from('<H', buffer, 2)
    length += 8  # include header and checksum footer
    if len(buffer) < length:
        return None, buffer
    packet, buffer = buffer[:length], buffer[length:]

    # Reject any packets without a valid checksum
    if not verify_checksum(packet):
        return None, buffer
    return packet, buffer


def decode_message(packet):
    msg_type = packet[5]
    if msg_type == MessageType.PING:
        print(f'Received ping')
    elif msg_type == MessageType.STATUS_REPLY:
        print(f'Received status reply: {packet.hex()}')
    elif msg_type == MessageType.REPLY:
        print(f'Received reply: {packet.hex()}')
    else:
        print(f'Received unknown message type {msg_type}: {packet.hex()}')
