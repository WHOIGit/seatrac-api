-- SeaTrac dissector – extended per SeaTrac Interface Rev4.docx
-- This implementation covers the major message types and fields as described in the document.

local proto = Proto("seatrac", "SeaTrac Message")

----------------------------------------
-- Helper function to invert enum tables
----------------------------------------
local function invert_enum(enum_table)
    local result = {}
    for k, v in pairs(enum_table) do
        result[v] = k
    end
    return result
end

----------------------------------------
-- Enum Tables (migrated from seatrac.py)
----------------------------------------
local MessageType                 = {
    STATUS_REQUEST = 7,
    STATUS_REPLY = 8,
    REQUEST = 9,
    REPLY = 10,
    COMMAND = 11
}

local BoardID                     = {
    COM = 1,
    BATTERY1 = 2,
    BATTERY2 = 3,
    BATTERY3 = 4,
    CHARGERS1 = 5,
    CHARGERS2 = 6,
    PMS = 7,
    MOTOR = 8,
    RCREMOTE = 9,
    PC = 10,
    DBSERVER = 11,
    WINCH = 12
}

local SinkID                      = {
    COM_WRITER = 1,
    BATTERY1_WRITER = 2,
    BATTERY2_WRITER = 3,
    BATTERY3_WRITER = 4,
    CHARGERS1_WRITER = 5,
    CHARGERS2_WRITER = 6,
    PMS_WRITER = 7,
    MOTOR_WRITER = 8,
    WINCH_WRITER = 9,
    COM_TIME = 10,
    COM_TRACER = 11,
    COM_CAN_STATUS = 12,
    COM_LOG_REPORTER = 13,
    COM_REPORTER = 14,
    COM_UDPPC_REPORTER = 15, -- Reports just to PC
    COM_CELL1_REPORTER = 16,
    COM_UDPSTARLINK_REPORTER = 17,
    COM_SBD_REPORTER = 18,
    COM_UDPCELL2_REPORTER = 19,
    COM_SWITCHES = 20,
    COM_CELL1_MODEM = 21,
    COM_SBD_MODEM = 23,
    COM_UDPCERTUS_REPORTER = 24,
    COM_SDFAT = 25,
    COM_AIS = 26,
    COM_AIS_TARGET = 27, -- Not used: AISTarget is not a MsgSink
    COM_STD_SENSOR = 28,
    COM_ALERTER = 29,
    BATTERY1_TIME = 30,
    BATTERY1_TRACER = 31,
    BATTERY1_CAN_STATUS = 32,
    BATTERY1_PACK = 33,
    BATTERY2_TIME = 35,
    BATTERY2_TRACER = 36,
    BATTERY2_CAN_STATUS = 37,
    BATTERY2_PACK = 38,
    BATTERY3_TIME = 40,
    BATTERY3_TRACER = 41,
    BATTERY3_CAN_STATUS = 42,
    BATTERY3_PACK = 43,
    CHARGERS1_TIME = 50,
    CHARGERS1_TRACER = 51,
    CHARGERS1_CAN_STATUS = 52,
    CHARGERS1 = 53,
    CHARGERS2_TIME = 55,
    CHARGERS2_TRACER = 56,
    CHARGERS2_CAN_STATUS = 57,
    CHARGERS2 = 58,
    PMS_TIME = 60,
    PMS_TRACER = 61,
    PMS_CAN_STATUS = 62,
    PMS_SWITCHES = 63,
    PMS_BMS = 64,
    MOTOR_TIME = 70,
    MOTOR_TRACER = 71,
    MOTOR_CAN_STATUS = 72,
    MOTOR_SWITCHES = 73,
    MOTOR_PROP_MOTOR = 74,
    MOTOR_STEER_MOTOR = 75,
    MOTOR_ATTITUDE = 76,
    MOTOR_GPS = 77,
    MOTOR_WIND = 78,
    MOTOR_BU_ATTITUDE = 79,
    MOTOR_BU_GPS = 80,
    MOTOR_WATERSPEED = 81,
    MOTOR_PROPULSION = 82,
    MOTOR_NAVIGATOR = 83,
    MOTOR_LOGGER = 84,
    MOTOR_SIMULATOR = 85,
    MOTOR_IMU = 86,
    MOTOR_WATERDEPTH = 87,
    RCREMOTE_WRITER = 90,
    PC_BSDRIVER = 92,
    PC_STARLINK_REPORTER = 93,
    PC_CERTUS_REPORTER = 94,
    PC_CAMERA_MGR = 95,
    PC_CELL2_REPORTER = 97,
    PC_REMOTE_PC = 98,
    PC_SDFAT = 99,
    DBSERVER_ALERTER = 100,
    PC_TIME = 101,
    PC_ADCP = 102,
    PC_WAVE_SENSOR = 103,
    PC_AML_SONDE = 104,
    PC_NMEASTREAM = 105,
    PC_ALERTER = 106,
    WINCH_MOTOR = 110,
    WINCH_MANAGER = 111,
    WINCH_TIME = 112,
    WINCH_TRACER = 113,
    WINCH_LOGGER = 114
}

----------------------------------------
-- Function ID Enums
----------------------------------------

local COM_SWITCHES_FunctionID     = {
    SET = 0
}

local COM_AIS_FunctionID          = {
    POSITION_REPORT = 2,
    TARGET_DETAILS_UPDATE = 4
}

local PMS_SWITCHES_FunctionID     = {
    SET = 0
}

local MOTOR_PROPULSION_FunctionID = {
    RUDDER_ANGLE_RPM = 1,
    HEADING_RPM = 2,
    CHANGE_STATE = 5,
    MOVE_TO_POINT = 7
}

local PC_CAMERA_MGR_FunctionID    = {
    START_ALL = 0,
    START = 2,
    STOP = 3,
    STOP_ALL = 4,
    IMAGE_DATA_START = 5,
    PTZ = 5
}

----------------------------------------
-- Value Mapping Tables
----------------------------------------

local camera_locations            = {
    [0] = "Bow",
    [1] = "Port",
    [2] = "Starboard",
    [3] = "Stern",
    [4] = "PTZ – IR",
    [5] = "PTZ – EO"
}

local send_streams                = {
    [0] = "None",
    [1] = "Cell Link",
    [2] = "Certus Link",
    [4] = "Silvus Link"
}

local resolutions                 = {
    [0] = "Low (360p)",
    [1] = "Medium (720p)",
    [2] = "High (1080p)"
}

local ptz_commands                = {
    [0] = "Pan absolute",
    [1] = "Pan relative",
    [2] = "Pan jogging",
    [3] = "Tilt absolute",
    [4] = "Tilt relative",
    [5] = "Tilt jogging",
    [6] = "Zoom absolute",
    [7] = "Zoom relative",
    [8] = "Zoom jogging / Stabilize" -- Note: 8 is used for zoom jogging and/or stabilization.
}

local propulsion_states           = {
    [0] = "Unknown",
    [1] = "Off",
    [2] = "Power on",
    [3] = "Enabling motors",
    [4] = "Idle",
    [5] = "Homing steering motor",
    [6] = "Ready (Drift)",
    [7] = "Rudder angle & RPM control",
    [8] = "Heading & RPM control",
    [9] = "Track & RPM control"
}

local navigator_states            = {
    [0] = "Unknown",
    [1] = "Off",
    [2] = "Starting",
    [3] = "Drift",
    [4] = "RC manual mode",
    [5] = "RC auto mode",
    [6] = "Hold position",
    [7] = "Mission",
    [8] = "All stop (emergency stop)"
}

----------------------------------------
-- Helper Functions
----------------------------------------

-- Type size mapping
local sizeof                      = {
    [ftypes.UINT8] = 1,
    [ftypes.UINT16] = 2,
    [ftypes.UINT24] = 3,
    [ftypes.UINT32] = 4,
    [ftypes.UINT64] = 8,
    [ftypes.INT8] = 1,
    [ftypes.INT16] = 2,
    [ftypes.INT24] = 3,
    [ftypes.INT32] = 4,
    [ftypes.INT64] = 8,
    [ftypes.FLOAT] = 4,
    [ftypes.DOUBLE] = 8
}

-- Helper function for adding fields and advancing offset
local function tree_add(tree, field, tvb, offset, length)
    length = length or sizeof[field.type]
    local item = tree:add_le(field, tvb(offset, length))
    return offset + length, item
end

-- Helper function to generate board_sink key
local function BS(board_id, sink_id)
    return string.format("%d_%d", board_id, sink_id)
end

-- Helper function to generate board_sink_func key
local function BSF(board_id, sink_id, func_id)
    return string.format("%d_%d_%d", board_id, sink_id, func_id)
end

----------------------------------------
-- Common Fields (Header & Shared)
----------------------------------------

local f = {}

local function PF(field)
    table.insert(proto.fields, field)
    return field
end


-- Header fields
f.sync                  = PF(ProtoField.uint16("seatrac.sync", "Sync Bytes", base.HEX))
f.length                = PF(ProtoField.uint16("seatrac.length", "Data Length", base.DEC))
f.relay                 = PF(ProtoField.uint8("seatrac.relay", "Message Relay", base.DEC))
f.msg_type              = PF(ProtoField.uint8("seatrac.msg_type", "Message Type", base.DEC,
    invert_enum(MessageType)))
f.checksum              = PF(ProtoField.uint16("seatrac.checksum", "Checksum", base.HEX))

-- Common fields for all payloads
f.sink_id               = PF(ProtoField.uint8("seatrac.sink_id", "SinkID", base.DEC, invert_enum(SinkID)))
f.board_id              = PF(ProtoField.uint8("seatrac.board_id", "BoardID", base.DEC, invert_enum(BoardID)))
f.func                  = PF(ProtoField.uint8("seatrac.func", "Function", base.DEC))
f.raw                   = PF(ProtoField.bytes("seatrac.raw", "Raw Data"))

-- Function ID fields for specific combinations
f.com_switches_func     = PF(ProtoField.uint8("seatrac.com_switches_func", "COM_SWITCHES Function", base.DEC,
    invert_enum(COM_SWITCHES_FunctionID)))
f.com_ais_func          = PF(ProtoField.uint8("seatrac.com_ais_func", "COM_AIS Function", base.DEC,
    invert_enum(COM_AIS_FunctionID)))
f.pms_switches_func     = PF(ProtoField.uint8("seatrac.pms_switches_func", "PMS_SWITCHES Function", base.DEC,
    invert_enum(PMS_SWITCHES_FunctionID)))
f.motor_propulsion_func = PF(ProtoField.uint8("seatrac.motor_propulsion_func", "MOTOR_PROPULSION Function",
    base.DEC,
    invert_enum(MOTOR_PROPULSION_FunctionID)))
f.pc_camera_mgr_func    = PF(ProtoField.uint8("seatrac.pc_camera_mgr_func", "PC_CAMERA_MGR Function", base.DEC,
    invert_enum(PC_CAMERA_MGR_FunctionID)))

-- Common fields used across multiple message types
f.new_state             = PF(ProtoField.int32("seatrac.new_state", "New State"))
f.rudder_angle          = PF(ProtoField.float("seatrac.rudder_angle", "Rudder Angle (deg)"))
f.rpm                   = PF(ProtoField.float("seatrac.rpm", "RPM"))
f.heading               = PF(ProtoField.float("seatrac.heading", "Heading"))
f.speed                 = PF(ProtoField.float("seatrac.speed", "Speed (kts)"))

-- DATETIME structure fields (8 bytes as defined in document)
f.datetime              = PF(ProtoField.none("seatrac.datetime", "DATETIME"))
f.datetime_year         = PF(ProtoField.uint16("seatrac.datetime.year", "Year", base.DEC))
f.datetime_month        = PF(ProtoField.uint8("seatrac.datetime.month", "Month", base.DEC))
f.datetime_day          = PF(ProtoField.uint8("seatrac.datetime.day", "Day", base.DEC))
f.datetime_hour         = PF(ProtoField.uint8("seatrac.datetime.hour", "Hour", base.DEC))
f.datetime_minute       = PF(ProtoField.uint8("seatrac.datetime.minute", "Minute", base.DEC))
f.datetime_second       = PF(ProtoField.uint8("seatrac.datetime.second", "Second", base.DEC))
f.datetime_hundredths   = PF(ProtoField.uint8("seatrac.datetime.hundredths", "Hundredths", base.DEC))

-- Parse DATETIME structure (8 bytes, per the interface document)
local function dissect_datetime(tvb, offset, tree, label)
    local dt_tree = tree:add_le(f.datetime, tvb(offset, 8), label or "DATETIME")
    local year = tvb(offset, 2):le_uint() + 1900
    local item
    offset, item = tree_add(dt_tree, f.datetime_year, tvb, offset)
    item:append_text(" (" .. year .. ")")
    offset = tree_add(dt_tree, f.datetime_month, tvb, offset)
    offset = tree_add(dt_tree, f.datetime_day, tvb, offset)
    offset = tree_add(dt_tree, f.datetime_hour, tvb, offset)
    offset = tree_add(dt_tree, f.datetime_minute, tvb, offset)
    offset = tree_add(dt_tree, f.datetime_second, tvb, offset)
    offset = tree_add(dt_tree, f.datetime_hundredths, tvb, offset)
    return offset
end

-- Add function field based on board_id and sink_id
local function add_function_field(tvb, tree, offset, board_id, sink_id)
    if board_id == BoardID.COM and sink_id == SinkID.COM_SWITCHES then
        return tree_add(tree, f.com_switches_func, tvb, offset)
    elseif board_id == BoardID.COM and sink_id == SinkID.COM_AIS then
        return tree_add(tree, f.com_ais_func, tvb, offset)
    elseif board_id == BoardID.PMS and sink_id == SinkID.PMS_SWITCHES then
        return tree_add(tree, f.pms_switches_func, tvb, offset)
    elseif board_id == BoardID.MOTOR and sink_id == SinkID.MOTOR_PROPULSION then
        return tree_add(tree, f.motor_propulsion_func, tvb, offset)
    elseif board_id == BoardID.PC and sink_id == SinkID.PC_CAMERA_MGR then
        return tree_add(tree, f.pc_camera_mgr_func, tvb, offset)
    else
        return tree_add(tree, f.func, tvb, offset)
    end
end

----------------------------------------
-- Status Reply: PMS_BMS (Power Level)
----------------------------------------
-- Fields
f.pms_bms = {
    pack_current = PF(ProtoField.int16("seatrac.pms_bms.pack_current", "Pack Current", base.DEC, nil, 0, "×0.002")),
    load_current = PF(ProtoField.int16("seatrac.pms_bms.load_current", "Load Current", base.DEC, nil, 0, "×0.002")),
    pack_voltage = PF(ProtoField.uint16("seatrac.pms_bms.pack_voltage", "Pack Voltage", base.DEC, nil, 0, "×0.001")),
    soc = PF(ProtoField.uint16("seatrac.pms_bms.soc", "SOC%", base.DEC, nil, 0, "×0.002")),
    unknown = PF(ProtoField.bytes("seatrac.pms_bms.unknown", "Unknown Fields"))
}

-- Dissector
local function dissect_status_pms_bms(tvb, tree, offset)
    tree:append_text(" [Power Level]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    local item
    offset, item = tree_add(tree, f.raw, tvb, offset, 18)
    item:set_text("Unknown Fields (bytes 16–33)")
    offset = tree_add(tree, f.pms_bms.pack_current, tvb, offset)
    offset = tree_add(tree, f.pms_bms.load_current, tvb, offset)
    offset = tree_add(tree, f.pms_bms.pack_voltage, tvb, offset)
    offset = tree_add(tree, f.pms_bms.soc, tvb, offset)
    offset = tree_add(tree, f.pms_bms.unknown, tvb, offset, 4)
    return offset
end

----------------------------------------
-- Status Reply: COM_AIS
----------------------------------------
-- Fields
f.com_ais = {
    state = PF(ProtoField.uint8("seatrac.com_ais.state", "AIS State", base.DEC,
        { [0] = "Off", [1] = "On" })),
    num_targets = PF(ProtoField.uint8("seatrac.com_ais.num_targets", "Number of AIS Targets", base.DEC)),
    closest_target = PF(ProtoField.float("seatrac.com_ais.closest_target", "Closest AIS Target (nm)")),
    mmsi = PF(ProtoField.uint32("seatrac.com_ais.mmsi", "MMSI")),
    speed_over_ground = PF(ProtoField.uint16("seatrac.com_ais.speed_over_ground", "Speed over Ground", base.DEC,
        nil, 0, "×0.002")),
    course_over_ground = PF(ProtoField.uint16("seatrac.com_ais.course_over_ground", "Course over Ground", base.DEC,
        nil, 0, "×0.01")),
    heading = PF(ProtoField.uint16("seatrac.com_ais.heading", "Heading", base.DEC)),
    nav_status = PF(ProtoField.uint8("seatrac.com_ais.nav_status", "Navigation Status", base.DEC))
}

-- Dissector for status reply
local function dissect_status_com_ais(tvb, tree, offset)
    tree:append_text(" [AIS Status Reply]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    offset = tree_add(tree, f.com_ais.state, tvb, offset)
    offset = tree_add(tree, f.com_ais.num_targets, tvb, offset)
    offset = tree_add(tree, f.com_ais.closest_target, tvb, offset)
    return offset
end

-- Dissector for reply
local function dissect_reply_com_ais(tvb, tree, offset, func_id)
    tree:append_text(" [COM/AIS Target Update]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    offset = tree_add(tree, f.gps.latitude, tvb, offset)
    offset = tree_add(tree, f.gps.longitude, tvb, offset)
    offset = tree_add(tree, f.com_ais.mmsi, tvb, offset)
    offset = tree_add(tree, f.com_ais.speed_over_ground, tvb, offset)
    offset = tree_add(tree, f.com_ais.course_over_ground, tvb, offset)
    offset = tree_add(tree, f.com_ais.heading, tvb, offset)
    offset = tree_add(tree, f.com_ais.nav_status, tvb, offset)
    return offset
end

----------------------------------------
-- Status Reply: MOTOR_ATTITUDE
----------------------------------------
-- Fields
f.attitude = {
    pitch = PF(ProtoField.int16("seatrac.attitude.pitch", "Pitch", base.DEC, nil, 0, "×0.01")),
    min_pitch = PF(ProtoField.int16("seatrac.attitude.min_pitch", "Min Pitch", base.DEC, nil, 0, "×0.01")),
    max_pitch = PF(ProtoField.int16("seatrac.attitude.max_pitch", "Max Pitch", base.DEC, nil, 0, "×0.01")),
    roll = PF(ProtoField.int16("seatrac.attitude.roll", "Roll", base.DEC, nil, 0, "×0.01")),
    min_roll = PF(ProtoField.int16("seatrac.attitude.min_roll", "Min Roll", base.DEC, nil, 0, "×0.01")),
    max_roll = PF(ProtoField.int16("seatrac.attitude.max_roll", "Max Roll", base.DEC, nil, 0, "×0.01")),
    heading = PF(ProtoField.int16("seatrac.attitude.heading", "Heading", base.DEC, nil, 0, "×0.01")),
    min_heading = PF(ProtoField.int16("seatrac.attitude.min_heading", "Min Heading", base.DEC, nil, 0, "×0.01")),
    max_heading = PF(ProtoField.int16("seatrac.attitude.max_heading", "Max Heading", base.DEC, nil, 0, "×0.01"))
}

-- Dissector
local function dissect_status_motor_attitude(tvb, tree, offset)
    tree:append_text(" [Attitude]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    local item
    offset, item = tree_add(tree, f.attitude.heading, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.pitch, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.min_pitch, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.max_pitch, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.roll, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.min_roll, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.max_roll, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.min_heading, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.max_heading, tvb, offset)
    item:append_text(" (×0.01)")
    return offset
end

----------------------------------------
-- Status Reply: MOTOR_GPS
----------------------------------------
-- Fields
f.gps = {
    latitude = PF(ProtoField.double("seatrac.gps.latitude", "Latitude", base.NONE, nil, "radians")),
    longitude = PF(ProtoField.double("seatrac.gps.longitude", "Longitude", base.NONE, nil, "radians")),
    kts = PF(ProtoField.uint16("seatrac.gps.kts", "Speed over Ground", base.DEC, nil, 0, "×0.002")),
    heading = PF(ProtoField.uint16("seatrac.gps.heading", "Heading", base.DEC, nil, 0, "×0.01")),
    current_kts = PF(ProtoField.uint16("seatrac.gps.current_kts", "Current Speed", base.DEC, nil, 0, "×0.002")),
    current_heading = PF(ProtoField.uint16("seatrac.gps.current_heading", "Current Heading", base.DEC, nil, 0, "×0.01")),
    wind_kts = PF(ProtoField.uint16("seatrac.gps.wind_kts", "Wind Speed", base.DEC, nil, 0, "×0.002")),
    wind_heading = PF(ProtoField.uint16("seatrac.gps.wind_heading", "Wind Heading", base.DEC, nil, 0, "×0.01"))
}

-- Dissector
local function dissect_status_motor_gps(tvb, tree, offset)
    tree:append_text(" [GPS]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    offset = tree_add(tree, f.gps.latitude, tvb, offset)
    offset = tree_add(tree, f.gps.longitude, tvb, offset)
    offset = tree_add(tree, f.gps.kts, tvb, offset)
    local item
    offset, item = tree_add(tree, f.gps.heading, tvb, offset)
    item:append_text(" (×0.01)")
    offset = tree_add(tree, f.gps.current_kts, tvb, offset)
    offset = tree_add(tree, f.gps.current_heading, tvb, offset)
    offset = tree_add(tree, f.gps.wind_kts, tvb, offset)
    offset = tree_add(tree, f.gps.wind_heading, tvb, offset)
    return offset
end

----------------------------------------
-- Status Reply: MOTOR_IMU
----------------------------------------
-- Fields
f.imu = {
    roll_gyro = PF(ProtoField.int16("seatrac.imu.roll_gyro", "Roll Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")),
    pitch_gyro = PF(ProtoField.int16("seatrac.imu.pitch_gyro", "Pitch Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")),
    heading_gyro = PF(ProtoField.int16("seatrac.imu.heading_gyro", "Heading Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")),
    accel_x = PF(ProtoField.int16("seatrac.imu.accel_x", "Acceleration X", base.DEC, nil, 0, "×0.01 m/s²")),
    accel_y = PF(ProtoField.int16("seatrac.imu.accel_y", "Acceleration Y", base.DEC, nil, 0, "×0.01 m/s²")),
    accel_z = PF(ProtoField.int16("seatrac.imu.accel_z", "Acceleration Z", base.DEC, nil, 0, "×0.01 m/s²")),
    max_accel_x = PF(ProtoField.int16("seatrac.imu.max_accel_x", "Max Accel X", base.DEC, nil, 0, "×0.01 m/s²")),
    max_accel_y = PF(ProtoField.int16("seatrac.imu.max_accel_y", "Max Accel Y", base.DEC, nil, 0, "×0.01 m/s²")),
    max_accel_z = PF(ProtoField.int16("seatrac.imu.max_accel_z", "Max Accel Z", base.DEC, nil, 0, "×0.01 m/s²")),
    heave = PF(ProtoField.int16("seatrac.imu.heave", "Heave", base.DEC, nil, 0, "×0.001 m")),
    min_z = PF(ProtoField.int16("seatrac.imu.min_z", "Min Z", base.DEC, nil, 0, "×0.001 m")),
    max_z = PF(ProtoField.int16("seatrac.imu.max_z", "Max Z", base.DEC, nil, 0, "×0.001 m"))
}

-- Dissector
local function dissect_status_motor_imu(tvb, tree, offset)
    tree:append_text(" [IMU]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    local item
    offset, item = tree_add(tree, f.attitude.roll, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.min_roll, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.max_roll, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.pitch, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.min_pitch, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.max_pitch, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.heading, tvb, offset)
    item:append_text(" (×0.01)")
    offset = tree_add(tree, f.imu.roll_gyro, tvb, offset)
    offset = tree_add(tree, f.imu.pitch_gyro, tvb, offset)
    offset = tree_add(tree, f.imu.heading_gyro, tvb, offset)
    offset = tree_add(tree, f.imu.accel_x, tvb, offset)
    offset = tree_add(tree, f.imu.accel_y, tvb, offset)
    offset = tree_add(tree, f.imu.accel_z, tvb, offset)
    offset = tree_add(tree, f.imu.max_accel_x, tvb, offset)
    offset = tree_add(tree, f.imu.max_accel_y, tvb, offset)
    offset = tree_add(tree, f.imu.max_accel_z, tvb, offset)
    offset = tree_add(tree, f.imu.heave, tvb, offset)
    offset = tree_add(tree, f.imu.min_z, tvb, offset)
    offset = tree_add(tree, f.imu.max_z, tvb, offset)
    offset, item = tree_add(tree, f.attitude.min_heading, tvb, offset)
    item:append_text(" (×0.01)")
    offset, item = tree_add(tree, f.attitude.max_heading, tvb, offset)
    item:append_text(" (×0.01)")
    return offset
end

----------------------------------------
-- Status Reply: MOTOR_WIND
----------------------------------------
-- Fields
f.wind = {
    apparent_speed = PF(ProtoField.uint16("seatrac.wind.apparent_speed", "Apparent Wind Speed", base.DEC, nil, 0,
        "×0.002")),
    apparent_angle = PF(ProtoField.int16("seatrac.wind.apparent_angle", "Apparent Wind Angle", base.DEC, nil, 0, "×0.01")),
    temperature = PF(ProtoField.int16("seatrac.wind.temperature", "Air Temperature", base.DEC, nil, 0, "×0.01 °C")),
    pressure = PF(ProtoField.int16("seatrac.wind.pressure", "Air Pressure", base.DEC, nil, 0, "×0.01 Bar"))
}

-- Dissector
local function dissect_status_motor_wind(tvb, tree, offset)
    tree:append_text(" [Wind Sensor]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    offset = tree_add(tree, f.wind.apparent_speed, tvb, offset)
    offset = tree_add(tree, f.wind.apparent_angle, tvb, offset)
    offset = tree_add(tree, f.wind.temperature, tvb, offset)
    offset = tree_add(tree, f.wind.pressure, tvb, offset)
    return offset
end

----------------------------------------
-- Status Reply: MOTOR_PROPULSION
----------------------------------------
-- Fields
f.prop = {
    state = PF(ProtoField.int16("seatrac.prop.state", "Propulsion State", base.DEC, propulsion_states, 0)),
    target_rpm = PF(ProtoField.int16("seatrac.prop.target_rpm", "Target RPM", base.DEC)),
    target_rudder = PF(ProtoField.int16("seatrac.prop.target_rudder", "Target Rudder Angle", base.DEC, nil, 0,
        "×0.01 deg")),
    target_heading = PF(ProtoField.int16("seatrac.prop.target_heading", "Target Heading", base.DEC, nil, 0, "×0.01 deg")),
    actual_rudder = PF(ProtoField.int16("seatrac.prop.actual_rudder", "Actual Rudder Angle", base.DEC, nil, 0,
        "×0.01 deg")),
    actual_rudder_speed = PF(ProtoField.int16("seatrac.prop.actual_rudder_speed", "Actual Rudder Speed", base.DEC, nil, 0,
        "×0.01 deg/sec"))
}

-- Dissector
local function dissect_status_motor_propulsion(tvb, tree, offset)
    tree:append_text(" [Propulsion]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    offset = tree_add(tree, f.prop.state, tvb, offset)
    offset = tree_add(tree, f.prop.target_rpm, tvb, offset)
    offset = tree_add(tree, f.prop.target_rudder, tvb, offset)
    offset = tree_add(tree, f.prop.target_heading, tvb, offset)
    offset = offset + 6 -- skip reserved bytes (3 shorts)
    offset = tree_add(tree, f.prop.actual_rudder, tvb, offset)
    offset = tree_add(tree, f.prop.actual_rudder_speed, tvb, offset)
    return offset
end

-- MOTOR_PROPULSION command handlers
local function dissect_command_motor_propulsion_rudder_angle_rpm(tvb, tree, offset, func_id)
    tree:append_text(" [MOTOR/Propulsion Rudder Angle and RPM]")
    offset = tree_add(tree, f.rudder_angle, tvb, offset, 4)
    offset = tree_add(tree, f.rpm, tvb, offset, 4)
    return offset
end

local function dissect_command_motor_propulsion_heading_rpm(tvb, tree, offset, func_id)
    tree:append_text(" [MOTOR/Propulsion Heading and RPM]")
    offset = tree_add(tree, f.heading, tvb, offset, 4)
    offset = tree_add(tree, f.rpm, tvb, offset, 4)
    return offset
end

----------------------------------------
-- Status Reply & Command: MOTOR_NAVIGATOR
----------------------------------------
-- Fields
f.nav = {
    mission_state = PF(ProtoField.int16("seatrac.nav.mission_state", "Mission State", base.DEC, navigator_states)),
    requested_speed = PF(ProtoField.uint16("seatrac.nav.requested_speed", "Requested Speed", base.DEC, nil, 0,
        "×0.002 kts")),
    target_rpm = PF(ProtoField.uint16("seatrac.nav.target_rpm", "Target RPM", base.DEC)),
    n_waypoints = PF(ProtoField.uint16("seatrac.nav.n_waypoints", "Number of Waypoints", base.DEC)),
    next_wp_index = PF(ProtoField.uint16("seatrac.nav.next_wp_index", "Next Waypoint Index", base.HEX)),
    leg_start_lat = PF(ProtoField.double("seatrac.nav.leg_start_lat", "Leg Start Latitude", base.NONE, nil, "radians")),
    leg_start_long = PF(ProtoField.double("seatrac.nav.leg_start_long", "Leg Start Longitude", base.NONE, nil, "radians")),
    leg_end_lat = PF(ProtoField.double("seatrac.nav.leg_end_lat", "Leg End Latitude", base.NONE, nil, "radians")),
    leg_end_long = PF(ProtoField.double("seatrac.nav.leg_end_long", "Leg End Longitude", base.NONE, nil, "radians")),
    laps_to_do = PF(ProtoField.uint16("seatrac.nav.laps_to_do", "Laps To Do", base.DEC)),
    laps_done = PF(ProtoField.uint16("seatrac.nav.laps_done", "Laps Done", base.DEC)),
    hold_lat = PF(ProtoField.double("seatrac.nav.hold_lat", "Hold Latitude", base.NONE, nil, "radians")),
    hold_long = PF(ProtoField.double("seatrac.nav.hold_long", "Hold Longitude", base.NONE, nil, "radians")),
    hold_rin = PF(ProtoField.float("seatrac.nav.hold_rin", "Hold Rin (ft)")),
    hold_rout = PF(ProtoField.float("seatrac.nav.hold_rout", "Hold Rout (ft)")),
    hold_kts = PF(ProtoField.float("seatrac.nav.hold_kts", "Hold Speed (kts)")),
    hold_time = PF(ProtoField.uint32("seatrac.nav.hold_time", "Hold Time (secs)")),
    alt_wp_active = PF(ProtoField.uint8("seatrac.nav.alt_wp_active", "Alternate Waypoint Active")),
    alt_wp_lat = PF(ProtoField.double("seatrac.nav.alt_wp_lat", "Alternate Waypoint Latitude", base.NONE, nil, "radians")),
    alt_wp_long = PF(ProtoField.double("seatrac.nav.alt_wp_long", "Alternate Waypoint Longitude", base.NONE, nil,
        "radians")),
    alt_heading_active = PF(ProtoField.uint8("seatrac.nav.alt_heading_active", "Alternate Heading Active")),
    alt_heading = PF(ProtoField.float("seatrac.nav.alt_heading", "Alternate Heading")),
    alt_speed_active = PF(ProtoField.uint8("seatrac.nav.alt_speed_active", "Alternate Speed Active")),
    alt_speed = PF(ProtoField.float("seatrac.nav.alt_speed", "Alternate Speed (kts)"))
}

-- Status Reply Dissector
local function dissect_status_motor_navigator(tvb, tree, offset)
    tree:append_text(" [Navigation]")
    offset = dissect_datetime(tvb, offset, tree, "Timestamp")
    offset = tree_add(tree, f.nav.mission_state, tvb, offset)
    offset = tree_add(tree, f.nav.requested_speed, tvb, offset)
    offset = tree_add(tree, f.nav.target_rpm, tvb, offset)
    offset = tree_add(tree, f.nav.n_waypoints, tvb, offset)
    offset = tree_add(tree, f.nav.next_wp_index, tvb, offset)
    if tvb:len() - offset >= 32 then
        offset = tree_add(tree, f.nav.leg_start_lat, tvb, offset)
        offset = tree_add(tree, f.nav.leg_start_long, tvb, offset)
        offset = tree_add(tree, f.nav.leg_end_lat, tvb, offset)
        offset = tree_add(tree, f.nav.leg_end_long, tvb, offset)
    end
    if tvb:len() - offset >= 4 then
        offset = tree_add(tree, f.nav.laps_to_do, tvb, offset)
        offset = tree_add(tree, f.nav.laps_done, tvb, offset)
    end
    -- Additional fields if holding (mission state == 6) or running a mission (state == 7)
    if tvb:len() - offset >= 8 + 8 + 4 + 4 + 4 then
        offset = tree_add(tree, f.nav.hold_lat, tvb, offset)
        offset = tree_add(tree, f.nav.hold_long, tvb, offset)
        offset = tree_add(tree, f.nav.hold_rin, tvb, offset)
        offset = tree_add(tree, f.nav.hold_rout, tvb, offset)
        offset = tree_add(tree, f.nav.hold_kts, tvb, offset)
    end
    if tvb:len() - offset >= 4 + 4 + 1 + 16 + 1 + 4 + 1 + 4 then
        offset = tree_add(tree, f.nav.hold_time, tvb, offset)
        offset = tree_add(tree, f.nav.hold_rout, tvb, offset)
        offset = tree_add(tree, f.nav.alt_wp_active, tvb, offset)
        offset = tree_add(tree, f.nav.alt_wp_lat, tvb, offset)
        offset = tree_add(tree, f.nav.alt_wp_long, tvb, offset)
        offset = tree_add(tree, f.nav.alt_heading_active, tvb, offset)
        offset = tree_add(tree, f.nav.alt_heading, tvb, offset)
        offset = tree_add(tree, f.nav.alt_speed_active, tvb, offset)
        offset = tree_add(tree, f.nav.alt_speed, tvb, offset)
    end
    return offset
end

-- Command Handlers
local function dissect_command_motor_navigator_change_state(tvb, tree, offset, func_id)
    tree:append_text(" [MOTOR/Navigation Change State]")
    offset = tree_add(tree, f.new_state, tvb, offset, 4)
    return offset
end

local function dissect_command_motor_navigator_move_to_point(tvb, tree, offset, func_id)
    tree:append_text(" [MOTOR/Navigation Move to Point]")
    offset = tree_add(tree, f.gps.latitude, tvb, offset)
    offset = tree_add(tree, f.gps.longitude, tvb, offset)
    offset = tree_add(tree, f.speed, tvb, offset)
    return offset
end

----------------------------------------
-- PC_CAMERA_MGR Commands and Replies
----------------------------------------
-- Fields
f.camera = {
    location = PF(ProtoField.uint8("seatrac.camera.location", "Camera Location", base.DEC, camera_locations)),
    resolution = PF(ProtoField.uint8("seatrac.camera.resolution", "Resolution", base.DEC, resolutions)),
    period = PF(ProtoField.uint16("seatrac.camera.period", "Period", base.DEC)),
    send_stream = PF(ProtoField.uint8("seatrac.camera.send_stream", "Send Stream", base.DEC, send_streams)),
    nchunks = PF(ProtoField.uint16("seatrac.camera.nchunks", "Total Chunks", base.DEC)),
    chunk = PF(ProtoField.uint16("seatrac.camera.chunk", "Chunk Number", base.DEC)),
    setup_frame_size = PF(ProtoField.uint16("seatrac.camera.setup_frame_size", "Setup Frame Size", base.DEC)),
    setup_frame_data = PF(ProtoField.bytes("seatrac.camera.setup_frame_data", "Setup Frame Data")),
    strength = PF(ProtoField.float("seatrac.camera.strength", "Strength")),
    zoom = PF(ProtoField.float("seatrac.camera.zoom", "Zoom")),
    height = PF(ProtoField.float("seatrac.camera.height", "Camera Height (ft)")),
    ptz_azimuth = PF(ProtoField.float("seatrac.camera.ptz_azimuth", "PTZ Azimuth")),
    ptz_elevation = PF(ProtoField.float("seatrac.camera.ptz_elevation", "PTZ Elevation")),
    ptz_view_angle = PF(ProtoField.float("seatrac.camera.ptz_view_angle", "PTZ View Angle")),
    image_rotation = PF(ProtoField.float("seatrac.camera.image_rotation", "Image Rotation")),
    image_height = PF(ProtoField.float("seatrac.camera.image_height", "Image Height")),
    image_data_size = PF(ProtoField.uint16("seatrac.camera.image_data_size", "Image Data Size", base.DEC)),
    image_data = PF(ProtoField.bytes("seatrac.camera.image_data", "Image Data"))
}

-- PTZ fields
f.ptz = {
    command = PF(ProtoField.uint8("seatrac.ptz.command", "PTZ Command", base.DEC, ptz_commands)),
    parameter = PF(ProtoField.int16("seatrac.ptz.parameter", "PTZ Parameter", base.DEC))
}

-- Command Handlers
local function dissect_command_pc_camera_mgr_start(tvb, tree, offset, func_id)
    tree:append_text(" [PC/Camera Start Stream]")
    offset = tree_add(tree, f.camera.location, tvb, offset)
    offset = tree_add(tree, f.camera.resolution, tvb, offset)
    offset = tree_add(tree, f.camera.period, tvb, offset)
    offset = tree_add(tree, f.camera.send_stream, tvb, offset)
    return offset
end

local function dissect_command_pc_camera_mgr_stop(tvb, tree, offset, func_id)
    tree:append_text(" [PC/Camera Stop Stream Specific]")
    offset = tree_add(tree, f.camera.location, tvb, offset)
    offset = tree_add(tree, f.camera.send_stream, tvb, offset)
    return offset
end

local function dissect_command_pc_camera_mgr_stop_all(tvb, tree, offset, func_id)
    tree:append_text(" [PC/Camera Stop All Streams]")
    return offset
end

local function dissect_command_pc_camera_mgr_ptz(tvb, tree, offset, func_id)
    tree:append_text(" [PC/Camera PTZ Command]")
    offset = tree_add(tree, f.ptz.command, tvb, offset)
    offset = tree_add(tree, f.ptz.parameter, tvb, offset)
    return offset
end

-- Reply Handlers
local function dissect_reply_pc_camera_mgr_image_data(tvb, tree, offset, func_id)
    tree:append_text(" [PC/Camera Image Data Start]")
    offset = tree_add(tree, f.camera.location, tvb, offset)
    offset = tree_add(tree, f.camera.send_stream, tvb, offset)
    offset = tree_add(tree, f.camera.nchunks, tvb, offset)
    offset = tree_add(tree, f.camera.chunk, tvb, offset)
    if tvb:len() - offset >= 2 then
        local next_field = tvb(offset, 2):le_uint()
        if next_field > 0 and next_field < 10000 then
            tree:append_text(" [H.264 Setup Frame]")
            offset = tree_add(tree, f.camera.setup_frame_size, tvb, offset)
            local item
            offset, item = tree_add(tree, f.camera.setup_frame_data, tvb, offset, next_field)
        else
            tree:append_text(" [H.264 Image Frame]")
            offset = tree_add(tree, f.camera.image_data_size, tvb, offset)
            local data_len = tvb(offset - 2, 2):le_uint()
            local item
            offset, item = tree_add(tree, f.camera.image_data, tvb, offset, data_len)
        end
    end
    return offset
end

----------------------------------------
-- Main Dispatcher Functions
----------------------------------------

-- Handle unknown message gracefully
local function dissect_unknown_message(tvb, tree, offset)
    tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown message")
    offset = offset or 0
    return tree_add(tree, f.raw, tvb, offset, tvb:len() - offset)
end

-- Dispatch for Status Request (msg type 7)
local function dissect_status_request(tvb, tree)
    tree:append_text(" [Status Request]")
    return dissect_unknown_message(tvb, tree, 0)
end

-- Dispatch for Status Reply (msg type 8)
local function dissect_status_reply(tvb, tree)
    local offset = 0
    local sink_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.sink_id, tvb, offset)

    -- Map sink_id to handler function
    local handler_map = {
        [SinkID.PMS_BMS] = dissect_status_pms_bms,
        [SinkID.COM_AIS] = dissect_status_com_ais,
        [SinkID.MOTOR_ATTITUDE] = dissect_status_motor_attitude,
        [SinkID.MOTOR_BU_ATTITUDE] = dissect_status_motor_attitude,
        [SinkID.MOTOR_GPS] = dissect_status_motor_gps,
        [SinkID.MOTOR_BU_GPS] = dissect_status_motor_gps,
        [SinkID.MOTOR_IMU] = dissect_status_motor_imu,
        [SinkID.MOTOR_WIND] = dissect_status_motor_wind,
        [SinkID.MOTOR_PROPULSION] = dissect_status_motor_propulsion,
        [SinkID.MOTOR_NAVIGATOR] = dissect_status_motor_navigator
    }

    local handler = handler_map[sink_id]
    if handler then
        return handler(tvb, tree, offset)
    end

    tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Status Reply SinkID: " .. sink_id)
end

-- Dispatch for Request (msg type 9)
local function dissect_request(tvb, tree)
    tree:append_text(" [Request]")
    return tree_add(tree, f.raw, tvb, 0, tvb:len())
end

-- Dispatch for Reply (msg type 10)
local function dissect_reply(tvb, tree)
    local offset = 0
    local board_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.board_id, tvb, offset)
    local sink_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.sink_id, tvb, offset)
    local func_id = tvb(offset, 1):uint()
    offset = add_function_field(tvb, tree, offset, board_id, sink_id)

    -- Define a dispatch table for reply handlers
    local handler_key = BSF(board_id, sink_id, func_id)
    local simple_key = BS(board_id, sink_id)

    local reply_handlers = {
        -- COM_AIS handler (handles all functions)
        [BS(BoardID.COM, SinkID.COM_AIS)] = dissect_reply_com_ais,

        -- PC_CAMERA_MGR handlers (function-specific)
        [BSF(BoardID.PC, SinkID.PC_CAMERA_MGR, PC_CAMERA_MGR_FunctionID.IMAGE_DATA_START)] =
            dissect_reply_pc_camera_mgr_image_data
    }

    -- Try exact match (board_id, sink_id, func_id) first, then fallback to (board_id, sink_id)
    local handler = reply_handlers[handler_key] or reply_handlers[simple_key]
    if handler then
        return handler(tvb, tree, offset, func_id)
    end

    return dissect_unknown_message(tvb, tree, offset)
end

-- Dispatch for Command (msg type 11)
local function dissect_command(tvb, tree)
    local offset = 0
    local board_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.board_id, tvb, offset)
    local sink_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.sink_id, tvb, offset)
    local func_id = tvb(offset, 1):uint()
    offset = add_function_field(tvb, tree, offset, board_id, sink_id)

    -- Define a dispatch table for command handlers
    local handler_key = BSF(board_id, sink_id, func_id)

    local command_handlers = {
        -- PC_CAMERA_MGR handlers
        [BSF(BoardID.PC, SinkID.PC_CAMERA_MGR, PC_CAMERA_MGR_FunctionID.START)] =
            dissect_command_pc_camera_mgr_start,
        [BSF(BoardID.PC, SinkID.PC_CAMERA_MGR, PC_CAMERA_MGR_FunctionID.STOP)] =
            dissect_command_pc_camera_mgr_stop,
        [BSF(BoardID.PC, SinkID.PC_CAMERA_MGR, PC_CAMERA_MGR_FunctionID.STOP_ALL)] =
            dissect_command_pc_camera_mgr_stop_all,
        [BSF(BoardID.PC, SinkID.PC_CAMERA_MGR, PC_CAMERA_MGR_FunctionID.PTZ)] =
            dissect_command_pc_camera_mgr_ptz,

        -- MOTOR_NAVIGATOR handlers
        [BSF(BoardID.MOTOR, SinkID.MOTOR_NAVIGATOR, MOTOR_PROPULSION_FunctionID.CHANGE_STATE)] =
            dissect_command_motor_navigator_change_state,
        [BSF(BoardID.MOTOR, SinkID.MOTOR_NAVIGATOR, MOTOR_PROPULSION_FunctionID.MOVE_TO_POINT)] =
            dissect_command_motor_navigator_move_to_point,

        -- MOTOR_PROPULSION handlers
        [BSF(BoardID.MOTOR, SinkID.MOTOR_PROPULSION, MOTOR_PROPULSION_FunctionID.RUDDER_ANGLE_RPM)] =
            dissect_command_motor_propulsion_rudder_angle_rpm,
        [BSF(BoardID.MOTOR, SinkID.MOTOR_PROPULSION, MOTOR_PROPULSION_FunctionID.HEADING_RPM)] =
            dissect_command_motor_propulsion_heading_rpm
    }

    local handler = command_handlers[handler_key]
    if handler then
        return handler(tvb, tree, offset, func_id)
    end

    return dissect_unknown_message(tvb, tree, offset)
end

----------------------------------------
-- Main Dissector Function
----------------------------------------
function proto.dissector(tvb, pinfo, tree)
    pinfo.cols.info = "SeaTrac Message"
    local subtree = tree:add(proto, tvb(), "SeaTrac Protocol Data")
    local offset = 0

    -- Header (per the document: sync bytes, length, relay, message type)
    offset = tree_add(subtree, f.sync, tvb, offset)
    local data_length = tvb(offset, 2):le_uint()
    offset = tree_add(subtree, f.length, tvb, offset)
    offset = tree_add(subtree, f.relay, tvb, offset)
    local mtype = tvb(offset, 1):uint()
    offset = tree_add(subtree, f.msg_type, tvb, offset)

    -- Payload is data_length bytes; dispatch based on Message Type:
    local payload_tvb = tvb(offset, data_length)
    if mtype == MessageType.STATUS_REQUEST then
        offset = offset + dissect_status_request(payload_tvb, subtree)
    elseif mtype == MessageType.STATUS_REPLY then
        offset = offset + dissect_status_reply(payload_tvb, subtree)
    elseif mtype == MessageType.REQUEST then
        offset = offset + dissect_request(payload_tvb, subtree)
    elseif mtype == MessageType.REPLY then
        offset = offset + dissect_reply(payload_tvb, subtree)
    elseif mtype == MessageType.COMMAND then
        offset = offset + dissect_command(payload_tvb, subtree)
    else
        subtree:add_expert_info(PI_UNDECODED, PI_WARN, "Unknown Message Type")
        offset = tree_add(subtree, f.raw, payload_tvb, 0, payload_tvb:len())
    end

    -- Reset the offset to point to the checksum bytes
    local checksum_offset = 6 + data_length
    if checksum_offset ~= offset then
        subtree:add_expert_info(PI_UNDECODED, PI_WARN, "Did not dissect entire packet")
    end
    tree_add(subtree, f.checksum, tvb, checksum_offset)
end

----------------------------------------
-- Register the dissector
----------------------------------------

-- Port 62001 is the default for UDP traffic within the boat
local udp_table = DissectorTable.get("udp.port")
udp_table:add(62001, proto)

-- Enable Decode As for serial captures over RF link
pcall(require, "xbee")
local xbee_table = DissectorTable.get("xbee.rf_data")
if xbee_table then
    xbee_table:add_for_decode_as(proto)
end

-- Enable Decode As for decrypted TLS traffic
local tls_table = DissectorTable.get("tls.port")
if tls_table then
    tls_table:add_for_decode_as(proto)
end
