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
local MessageType = {
    STATUS_REQUEST = 7,
    STATUS_REPLY = 8,
    REQUEST = 9,
    REPLY = 10,
    COMMAND = 11
}

local BoardID = {
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

local SinkID = {
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

local COM_SWITCHES_FunctionID = {
    SET = 0
}

local COM_AIS_FunctionID = {
    POSITION_REPORT = 2,
    TARGET_DETAILS_UPDATE = 4
}

local PMS_SWITCHES_FunctionID = {
    SET = 0
}

local MOTOR_PROPULSION_FunctionID = {
    RUDDER_ANGLE_RPM = 1,
    HEADING_RPM = 2,
    CHANGE_STATE = 5,
    MOVE_TO_POINT = 7
}

local PC_CAMERA_MGR_FunctionID = {
    START_ALL = 0,
    START = 2,
    STOP = 3,
    STOP_ALL = 4,
    IMAGE_DATA_START = 5,
    PTZ = 5
}

local camera_locations = {
    [0] = "Bow",
    [1] = "Port",
    [2] = "Starboard",
    [3] = "Stern",
    [4] = "PTZ – IR",
    [5] = "PTZ – EO"
}

local send_streams = {
    [0] = "None",
    [1] = "Cell Link",
    [2] = "Certus Link",
    [4] = "Silvus Link"
}

local resolutions = {
    [0] = "Low (360p)",
    [1] = "Medium (720p)",
    [2] = "High (1080p)"
}

local ptz_commands = {
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

local propulsion_states = {
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

local navigator_states = {
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

-- We no longer need the mapping tables as we'll use enum constants directly

-- We'll use direct enum comparisons instead of the helper function

----------------------------------------
-- Proto Fields (header & common)
----------------------------------------
local f                 = proto.fields
f.sync1                 = ProtoField.uint8("seatrac.sync1", "Sync Byte 1", base.HEX)
f.sync2                 = ProtoField.uint8("seatrac.sync2", "Sync Byte 2", base.HEX)
f.length                = ProtoField.uint16("seatrac.length", "Data Length", base.DEC)
f.relay                 = ProtoField.uint8("seatrac.relay", "Message Relay", base.DEC)
f.msg_type              = ProtoField.uint8("seatrac.msg_type", "Message Type", base.DEC, invert_enum(MessageType))
f.checksum              = ProtoField.uint16("seatrac.checksum", "Checksum", base.HEX)

-- Common fields used in payloads:
f.sink_id               = ProtoField.uint8("seatrac.sink_id", "SinkID", base.DEC, invert_enum(SinkID))
f.board_id              = ProtoField.uint8("seatrac.board_id", "BoardID", base.DEC, invert_enum(BoardID))
f.func                  = ProtoField.uint8("seatrac.func", "Function", base.DEC)
-- Function ID fields for specific combinations
f.com_switches_func     = ProtoField.uint8("seatrac.com_switches_func", "COM_SWITCHES Function", base.DEC,
    invert_enum(COM_SWITCHES_FunctionID))
f.com_ais_func          = ProtoField.uint8("seatrac.com_ais_func", "COM_AIS Function", base.DEC,
    invert_enum(COM_AIS_FunctionID))
f.pms_switches_func     = ProtoField.uint8("seatrac.pms_switches_func", "PMS_SWITCHES Function", base.DEC,
    invert_enum(PMS_SWITCHES_FunctionID))
f.motor_propulsion_func = ProtoField.uint8("seatrac.motor_propulsion_func", "MOTOR_PROPULSION Function", base.DEC,
    invert_enum(MOTOR_PROPULSION_FunctionID))
f.pc_camera_mgr_func    = ProtoField.uint8("seatrac.pc_camera_mgr_func", "PC_CAMERA_MGR Function", base.DEC,
    invert_enum(PC_CAMERA_MGR_FunctionID))
f.raw                   = ProtoField.bytes("seatrac.raw", "Raw Data")
f.new_state             = ProtoField.int32("seatrac.new_state", "New State")
f.rudder_angle          = ProtoField.float("seatrac.rudder_angle", "Rudder Angle (deg)")
f.rpm                   = ProtoField.float("seatrac.rpm", "RPM")
f.heading               = ProtoField.float("seatrac.heading", "Heading")
f.speed                 = ProtoField.float("seatrac.speed", "Speed (kts)")

-- DATETIME structure fields (8 bytes as defined in document)
f.datetime              = ProtoField.none("seatrac.datetime", "DATETIME")
f.datetime_year         = ProtoField.uint16("seatrac.datetime.year", "Year", base.DEC)
f.datetime_month        = ProtoField.uint8("seatrac.datetime.month", "Month", base.DEC)
f.datetime_day          = ProtoField.uint8("seatrac.datetime.day", "Day", base.DEC)
f.datetime_hour         = ProtoField.uint8("seatrac.datetime.hour", "Hour", base.DEC)
f.datetime_minute       = ProtoField.uint8("seatrac.datetime.minute", "Minute", base.DEC)
f.datetime_second       = ProtoField.uint8("seatrac.datetime.second", "Second", base.DEC)
f.datetime_hundredths   = ProtoField.uint8("seatrac.datetime.hundredths", "Hundredths", base.DEC)

-- Power level fields
f.pack_current          = ProtoField.int16("seatrac.pack_current", "Pack Current", base.DEC, nil, 0, "×0.002")
f.load_current          = ProtoField.int16("seatrac.load_current", "Load Current", base.DEC, nil, 0, "×0.002")
f.pack_voltage          = ProtoField.uint16("seatrac.pack_voltage", "Pack Voltage", base.DEC, nil, 0, "×0.001")
f.soc                   = ProtoField.uint16("seatrac.soc", "SOC%", base.DEC, nil, 0, "×0.002")
f.unknown2              = ProtoField.bytes("seatrac.unknown2", "Unknown Fields")

-- AIS fields
f.ais_state             = ProtoField.uint8("seatrac.ais.state", "AIS State", base.DEC, { [0] = "Off", [1] = "On" })
f.ais_num_targets       = ProtoField.uint8("seatrac.ais.num_targets", "Number of AIS Targets", base.DEC)
f.ais_closest_target    = ProtoField.float("seatrac.ais.closest_target", "Closest AIS Target (nm)")
f.ais_mmsi              = ProtoField.uint32("seatrac.ais.mmsi", "MMSI")
f.ais_speed_over_ground = ProtoField.uint16("seatrac.ais.speed_over_ground", "Speed over Ground", base.DEC, nil, 0, "×0.002")
f.ais_course_over_ground = ProtoField.uint16("seatrac.ais.course_over_ground", "Course over Ground", base.DEC, nil, 0, "×0.01")
f.ais_heading           = ProtoField.uint16("seatrac.ais.heading", "Heading", base.DEC)
f.ais_nav_status        = ProtoField.uint8("seatrac.ais.nav_status", "Navigation Status", base.DEC)

-- Attitude fields (shared by several messages)
f.pitch                 = ProtoField.int16("seatrac.attitude.pitch", "Pitch", base.DEC, nil, 0, "×0.01")
f.min_pitch             = ProtoField.int16("seatrac.attitude.min_pitch", "Min Pitch", base.DEC, nil, 0, "×0.01")
f.max_pitch             = ProtoField.int16("seatrac.attitude.max_pitch", "Max Pitch", base.DEC, nil, 0, "×0.01")
f.roll                  = ProtoField.int16("seatrac.attitude.roll", "Roll", base.DEC, nil, 0, "×0.01")
f.min_roll              = ProtoField.int16("seatrac.attitude.min_roll", "Min Roll", base.DEC, nil, 0, "×0.01")
f.max_roll              = ProtoField.int16("seatrac.attitude.max_roll", "Max Roll", base.DEC, nil, 0, "×0.01")
f.min_heading           = ProtoField.int16("seatrac.attitude.min_heading", "Min Heading", base.DEC, nil, 0, "×0.01")
f.max_heading           = ProtoField.int16("seatrac.attitude.max_heading", "Max Heading", base.DEC, nil, 0, "×0.01")

-- GPS fields
f.latitude              = ProtoField.double("seatrac.gps.latitude", "Latitude", base.NONE, nil, "radians")
f.longitude             = ProtoField.double("seatrac.gps.longitude", "Longitude", base.NONE, nil, "radians")
f.kts                   = ProtoField.uint16("seatrac.gps.kts", "Speed over Ground", base.DEC, nil, 0, "×0.002")
f.current_kts           = ProtoField.uint16("seatrac.gps.current_kts", "Current Speed", base.DEC, nil, 0, "×0.002")
f.current_heading       = ProtoField.uint16("seatrac.gps.current_heading", "Current Heading", base.DEC, nil, 0, "×0.01")
f.wind_kts              = ProtoField.uint16("seatrac.gps.wind_kts", "Wind Speed", base.DEC, nil, 0, "×0.002")
f.wind_heading          = ProtoField.uint16("seatrac.gps.wind_heading", "Wind Heading", base.DEC, nil, 0, "×0.01")

-- IMU fields
f.roll_gyro             = ProtoField.int16("seatrac.imu.roll_gyro", "Roll Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")
f.pitch_gyro            = ProtoField.int16("seatrac.imu.pitch_gyro", "Pitch Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")
f.heading_gyro          = ProtoField.int16("seatrac.imu.heading_gyro", "Heading Gyro Rate", base.DEC, nil, 0,
    "×0.02 deg/sec")
f.accel_x               = ProtoField.int16("seatrac.imu.accel_x", "Acceleration X", base.DEC, nil, 0, "×0.01 m/s²")
f.accel_y               = ProtoField.int16("seatrac.imu.accel_y", "Acceleration Y", base.DEC, nil, 0, "×0.01 m/s²")
f.accel_z               = ProtoField.int16("seatrac.imu.accel_z", "Acceleration Z", base.DEC, nil, 0, "×0.01 m/s²")
f.max_accel_x           = ProtoField.int16("seatrac.imu.max_accel_x", "Max Accel X", base.DEC, nil, 0, "×0.01 m/s²")
f.max_accel_y           = ProtoField.int16("seatrac.imu.max_accel_y", "Max Accel Y", base.DEC, nil, 0, "×0.01 m/s²")
f.max_accel_z           = ProtoField.int16("seatrac.imu.max_accel_z", "Max Accel Z", base.DEC, nil, 0, "×0.01 m/s²")
f.heave                 = ProtoField.int16("seatrac.imu.heave", "Heave", base.DEC, nil, 0, "×0.001 m")
f.min_z                 = ProtoField.int16("seatrac.imu.min_z", "Min Z", base.DEC, nil, 0, "×0.001 m")
f.max_z                 = ProtoField.int16("seatrac.imu.max_z", "Max Z", base.DEC, nil, 0, "×0.001 m")

-- Wind sensor extra fields
f.apparent_speed        = ProtoField.uint16("seatrac.wind.apparent_speed", "Apparent Wind Speed", base.DEC, nil, 0,
    "×0.002")
f.apparent_angle        = ProtoField.int16("seatrac.wind.apparent_angle", "Apparent Wind Angle", base.DEC, nil, 0,
    "×0.01")
f.temperature           = ProtoField.int16("seatrac.wind.temperature", "Air Temperature", base.DEC, nil, 0, "×0.01 °C")
f.pressure              = ProtoField.int16("seatrac.wind.pressure", "Air Pressure", base.DEC, nil, 0, "×0.01 Bar")

-- Propulsion fields
f.prop_state            = ProtoField.int16("seatrac.prop.state", "Propulsion State", base.DEC, propulsion_states, 0)
f.target_rpm            = ProtoField.int16("seatrac.prop.target_rpm", "Target RPM", base.DEC)
f.target_rudder         = ProtoField.int16("seatrac.prop.target_rudder", "Target Rudder Angle", base.DEC, nil, 0,
    "×0.01 deg")
f.actual_rudder         = ProtoField.int16("seatrac.prop.actual_rudder", "Actual Rudder Angle", base.DEC, nil, 0,
    "×0.01 deg")
f.actual_rudder_speed   = ProtoField.int16("seatrac.prop.actual_rudder_speed", "Actual Rudder Speed", base.DEC, nil, 0,
    "×0.01 deg/sec")

-- Navigation fields
f.mission_state         = ProtoField.int16("seatrac.nav.mission_state", "Mission State", base.DEC, navigator_states)
f.requested_speed       = ProtoField.uint16("seatrac.nav.requested_speed", "Requested Speed", base.DEC, nil, 0,
    "×0.002 kts")
f.n_waypoints           = ProtoField.uint16("seatrac.nav.n_waypoints", "Number of Waypoints", base.DEC)
f.next_wp_index         = ProtoField.uint16("seatrac.nav.next_wp_index", "Next Waypoint Index", base.HEX)
f.leg_start_lat         = ProtoField.double("seatrac.nav.leg_start_lat", "Leg Start Latitude", base.NONE, nil, "radians")
f.leg_start_long        = ProtoField.double("seatrac.nav.leg_start_long", "Leg Start Longitude", base.NONE, nil,
    "radians")
f.leg_end_lat           = ProtoField.double("seatrac.nav.leg_end_lat", "Leg End Latitude", base.NONE, nil, "radians")
f.leg_end_long          = ProtoField.double("seatrac.nav.leg_end_long", "Leg End Longitude", base.NONE, nil, "radians")
f.laps_to_do            = ProtoField.uint16("seatrac.nav.laps_to_do", "Laps To Do", base.DEC)
f.laps_done             = ProtoField.uint16("seatrac.nav.laps_done", "Laps Done", base.DEC)
f.hold_lat              = ProtoField.double("seatrac.nav.hold_lat", "Hold Latitude", base.NONE, nil, "radians")
f.hold_long             = ProtoField.double("seatrac.nav.hold_long", "Hold Longitude", base.NONE, nil, "radians")
f.hold_rin              = ProtoField.float("seatrac.nav.hold_rin", "Hold Rin (ft)")
f.hold_rout             = ProtoField.float("seatrac.nav.hold_rout", "Hold Rout (ft)")
f.hold_kts              = ProtoField.float("seatrac.nav.hold_kts", "Hold Speed (kts)")
f.hold_time             = ProtoField.uint32("seatrac.nav.hold_time", "Hold Time (secs)")
f.alt_wp_active         = ProtoField.uint8("seatrac.nav.alt_wp_active", "Alternate Waypoint Active")
f.alt_wp_lat            = ProtoField.double("seatrac.nav.alt_wp_lat", "Alternate Waypoint Latitude", base.NONE, nil,
    "radians")
f.alt_wp_long           = ProtoField.double("seatrac.nav.alt_wp_long", "Alternate Waypoint Longitude", base.NONE, nil,
    "radians")
f.alt_heading_active    = ProtoField.uint8("seatrac.nav.alt_heading_active", "Alternate Heading Active")
f.alt_heading           = ProtoField.float("seatrac.nav.alt_heading", "Alternate Heading")
f.alt_speed_active      = ProtoField.uint8("seatrac.nav.alt_speed_active", "Alternate Speed Active")
f.alt_speed             = ProtoField.float("seatrac.nav.alt_speed", "Alternate Speed (kts)")

-- Video / Camera fields
f.camera_location       = ProtoField.uint8("seatrac.camera.location", "Camera Location", base.DEC, camera_locations)
f.resolution            = ProtoField.uint8("seatrac.camera.resolution", "Resolution", base.DEC, resolutions)
f.period                = ProtoField.uint16("seatrac.camera.period", "Period", base.DEC)
f.send_stream           = ProtoField.uint8("seatrac.camera.send_stream", "Send Stream", base.DEC, send_streams)
f.nchunks               = ProtoField.uint16("seatrac.camera.nchunks", "Total Chunks", base.DEC)
f.chunk                 = ProtoField.uint16("seatrac.camera.chunk", "Chunk Number", base.DEC)
f.setup_frame_size      = ProtoField.uint16("seatrac.camera.setup_frame_size", "Setup Frame Size", base.DEC)
f.setup_frame_data      = ProtoField.bytes("seatrac.camera.setup_frame_data", "Setup Frame Data")
f.strength              = ProtoField.float("seatrac.camera.strength", "Strength")
f.zoom                  = ProtoField.float("seatrac.camera.zoom", "Zoom")
f.camera_height         = ProtoField.float("seatrac.camera.height", "Camera Height (ft)")
f.ptz_azimuth           = ProtoField.float("seatrac.camera.ptz_azimuth", "PTZ Azimuth")
f.ptz_elevation         = ProtoField.float("seatrac.camera.ptz_elevation", "PTZ Elevation")
f.ptz_view_angle        = ProtoField.float("seatrac.camera.ptz_view_angle", "PTZ View Angle")
f.image_rotation        = ProtoField.float("seatrac.camera.image_rotation", "Image Rotation")
f.image_height          = ProtoField.float("seatrac.camera.image_height", "Image Height")
f.image_data_size       = ProtoField.uint16("seatrac.camera.image_data_size", "Image Data Size", base.DEC)
f.image_data            = ProtoField.bytes("seatrac.camera.image_data", "Image Data")

-- PTZ fields
f.ptz_command           = ProtoField.uint8("seatrac.ptz.command", "PTZ Command", base.DEC, ptz_commands)
f.ptz_parameter         = ProtoField.int16("seatrac.ptz.parameter", "PTZ Parameter", base.DEC)

----------------------------------------
-- Helper Functions
----------------------------------------

-- Type size mapping
local sizeof = {
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

-- Parse DATETIME structure (8 bytes, per the interface document)
local function dissect_datetime(tvb, offset, tree, label)
    local dt_tree = tree:add_le(f.datetime, tvb(offset, 8), label or "DATETIME")
    local year = tvb(offset, 2):le_uint() + 1900
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

-- Dispatch for Status Request (msg type 7) – minimal example.
local function dissect_status_request(tvb, tree)
    tree:append_text(" [Status Request]")
    -- (No additional fields defined in the document for Status Request.)
    local offset = 0
    offset, item = tree_add(tree, f.raw, tvb, offset, tvb:len())
    return offset
end

-- Dispatch for Status Reply (msg type 8)
local function dissect_status_reply(tvb, tree)
    local offset = 0
    local sink_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.sink_id, tvb, offset)

    if sink_id == SinkID.PMS_BMS then
        tree:append_text(" [Power Level]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset, item = tree_add(tree, f.raw, tvb, offset, 18)
        item:set_text("Unknown Fields (bytes 16–33)")
        offset = tree_add(tree, f.pack_current, tvb, offset)
        offset = tree_add(tree, f.load_current, tvb, offset)
        offset = tree_add(tree, f.pack_voltage, tvb, offset)
        offset = tree_add(tree, f.soc, tvb, offset)
        offset = tree_add(tree, f.unknown2, tvb, offset, 4)
    elseif sink_id == SinkID.COM_AIS then
        tree:append_text(" [AIS Status Reply]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset = tree_add(tree, f.ais_state, tvb, offset)
        offset = tree_add(tree, f.ais_num_targets, tvb, offset)
        offset = tree_add(tree, f.ais_closest_target, tvb, offset)
    elseif sink_id == SinkID.MOTOR_ATTITUDE or sink_id == SinkID.MOTOR_BU_ATTITUDE then
        tree:append_text(" [Attitude]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset, item = tree_add(tree, f.heading, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.pitch, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.min_pitch, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.max_pitch, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.roll, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.min_roll, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.max_roll, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.min_heading, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.max_heading, tvb, offset)
        item:append_text(" (×0.01)")
    elseif sink_id == SinkID.MOTOR_GPS or sink_id == SinkID.MOTOR_BU_GPS then
        tree:append_text(" [GPS]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset = tree_add(tree, f.latitude, tvb, offset, 8)
        offset = tree_add(tree, f.longitude, tvb, offset, 8)
        offset = tree_add(tree, f.kts, tvb, offset)
        offset, item = tree_add(tree, f.heading, tvb, offset)
        item:append_text(" (×0.01)")
        offset = tree_add(tree, f.current_kts, tvb, offset)
        offset = tree_add(tree, f.current_heading, tvb, offset)
        offset = tree_add(tree, f.wind_kts, tvb, offset)
        offset = tree_add(tree, f.wind_heading, tvb, offset)
    elseif sink_id == SinkID.MOTOR_IMU then
        tree:append_text(" [IMU]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset, item = tree_add(tree, f.roll, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.min_roll, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.max_roll, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.pitch, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.min_pitch, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.max_pitch, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.heading, tvb, offset)
        item:append_text(" (×0.01)")
        offset = tree_add(tree, f.roll_gyro, tvb, offset)
        offset = tree_add(tree, f.pitch_gyro, tvb, offset)
        offset = tree_add(tree, f.heading_gyro, tvb, offset)
        offset = tree_add(tree, f.accel_x, tvb, offset)
        offset = tree_add(tree, f.accel_y, tvb, offset)
        offset = tree_add(tree, f.accel_z, tvb, offset)
        offset = tree_add(tree, f.max_accel_x, tvb, offset)
        offset = tree_add(tree, f.max_accel_y, tvb, offset)
        offset = tree_add(tree, f.max_accel_z, tvb, offset)
        offset = tree_add(tree, f.heave, tvb, offset)
        offset = tree_add(tree, f.min_z, tvb, offset)
        offset = tree_add(tree, f.max_z, tvb, offset)
        offset, item = tree_add(tree, f.min_heading, tvb, offset)
        item:append_text(" (×0.01)")
        offset, item = tree_add(tree, f.max_heading, tvb, offset)
        item:append_text(" (×0.01)")
    elseif sink_id == SinkID.MOTOR_WIND then
        tree:append_text(" [Wind Sensor]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset = tree_add(tree, f.apparent_speed, tvb, offset)
        offset = tree_add(tree, f.apparent_angle, tvb, offset)
        offset = tree_add(tree, f.temperature, tvb, offset)
        offset = tree_add(tree, f.pressure, tvb, offset)
    elseif sink_id == SinkID.MOTOR_PROPULSION then
        tree:append_text(" [Propulsion]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset = tree_add(tree, f.prop_state, tvb, offset)
        offset = tree_add(tree, f.target_rpm, tvb, offset)
        offset = tree_add(tree, f.target_rudder, tvb, offset)
        offset = tree_add(tree, f.target_heading, tvb, offset)
        offset = offset + 6 -- skip reserved bytes (3 shorts)
        offset = tree_add(tree, f.actual_rudder, tvb, offset)
        offset = tree_add(tree, f.actual_rudder_speed, tvb, offset)
    elseif sink_id == SinkID.MOTOR_NAVIGATOR then
        tree:append_text(" [Navigation]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset = tree_add(tree, f.mission_state, tvb, offset)
        offset = tree_add(tree, f.requested_speed, tvb, offset)
        offset = tree_add(tree, f.target_rpm, tvb, offset)
        offset = tree_add(tree, f.n_waypoints, tvb, offset)
        offset = tree_add(tree, f.next_wp_index, tvb, offset)
        if tvb:len() - offset >= 32 then
            offset = tree_add(tree, f.leg_start_lat, tvb, offset, 8)
            offset = tree_add(tree, f.leg_start_long, tvb, offset, 8)
            offset = tree_add(tree, f.leg_end_lat, tvb, offset, 8)
            offset = tree_add(tree, f.leg_end_long, tvb, offset, 8)
        end
        if tvb:len() - offset >= 4 then
            offset = tree_add(tree, f.laps_to_do, tvb, offset)
            offset = tree_add(tree, f.laps_done, tvb, offset)
        end
        -- Additional fields if holding (mission state == 6) or running a mission (state == 7)
        if tvb:len() - offset >= 8 + 8 + 4 + 4 + 4 then
            offset = tree_add(tree, f.hold_lat, tvb, offset, 8)
            offset = tree_add(tree, f.hold_long, tvb, offset, 8)
            offset = tree_add(tree, f.hold_rin, tvb, offset, 4)
            offset = tree_add(tree, f.hold_rout, tvb, offset, 4)
            offset = tree_add(tree, f.hold_kts, tvb, offset, 4)
        end
        if tvb:len() - offset >= 4 + 4 + 1 + 16 + 1 + 4 + 1 + 4 then
            offset = tree_add(tree, f.hold_time, tvb, offset, 4)
            offset = tree_add(tree, f.hold_rout, tvb, offset, 4)
            offset = tree_add(tree, f.alt_wp_active, tvb, offset)
            offset = tree_add(tree, f.alt_wp_lat, tvb, offset, 8)
            offset = tree_add(tree, f.alt_wp_long, tvb, offset, 8)
            offset = tree_add(tree, f.alt_heading_active, tvb, offset)
            offset = tree_add(tree, f.alt_heading, tvb, offset, 4)
            offset = tree_add(tree, f.alt_speed_active, tvb, offset)
            offset = tree_add(tree, f.alt_speed, tvb, offset, 4)
        end
    else
        tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Status Reply SinkID: " .. sink_id)
    end
    return offset
end

-- Dispatch for Request (msg type 9) – for brevity we show raw payload.
local function dissect_request(tvb, tree)
    tree:append_text(" [Request]")
    local offset = 0
    offset, item = tree_add(tree, f.raw, tvb, offset, tvb:len())
    return offset
end

-- Dispatch for Reply (msg type 10)
local function dissect_reply(tvb, tree)
    local offset = 0
    local board_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.board_id, tvb, offset)
    local sink_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.sink_id, tvb, offset)
    local func = tvb(offset, 1):uint()

    -- Use the appropriate function field based on sink_id
    if board_id == BoardID.COM and sink_id == SinkID.COM_SWITCHES then
        offset = tree_add(tree, f.com_switches_func, tvb, offset)
    elseif board_id == BoardID.COM and sink_id == SinkID.COM_AIS then
        offset = tree_add(tree, f.com_ais_func, tvb, offset)
    elseif board_id == BoardID.PMS and sink_id == SinkID.PMS_SWITCHES then
        offset = tree_add(tree, f.pms_switches_func, tvb, offset)
    elseif board_id == BoardID.MOTOR and sink_id == SinkID.MOTOR_PROPULSION then
        offset = tree_add(tree, f.motor_propulsion_func, tvb, offset)
    elseif board_id == BoardID.PC and sink_id == SinkID.PC_CAMERA_MGR then
        offset = tree_add(tree, f.pc_camera_mgr_func, tvb, offset)
    else
        offset = tree_add(tree, f.func, tvb, offset)
    end

    -- Use direct enum comparisons instead of descriptive strings
    if board_id == BoardID.COM and sink_id == SinkID.COM_AIS then
        tree:append_text(" [COM/AIS Target Update]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        offset = tree_add(tree, f.latitude, tvb, offset, 8)
        offset = tree_add(tree, f.longitude, tvb, offset, 8)
        offset = tree_add(tree, f.ais_mmsi, tvb, offset)
        offset = tree_add(tree, f.ais_speed_over_ground, tvb, offset)
        offset = tree_add(tree, f.ais_course_over_ground, tvb, offset)
        offset = tree_add(tree, f.ais_heading, tvb, offset)
        offset = tree_add(tree, f.ais_nav_status, tvb, offset)
    elseif board_id == BoardID.PC and sink_id == SinkID.PC_CAMERA_MGR and func == PC_CAMERA_MGR_FunctionID.IMAGE_DATA_START then
        tree:append_text(" [PC/Camera Image Data Start]")
        offset = tree_add(tree, f.camera_location, tvb, offset)
        offset = tree_add(tree, f.send_stream, tvb, offset)
        offset = tree_add(tree, f.nchunks, tvb, offset)
        offset = tree_add(tree, f.chunk, tvb, offset)
        if tvb:len() - offset >= 2 then
            local next_field = tvb(offset, 2):le_uint()
            if next_field > 0 and next_field < 10000 then
                tree:append_text(" [H.264 Setup Frame]")
                offset = tree_add(tree, f.setup_frame_size, tvb, offset)
                offset, item = tree_add(tree, f.setup_frame_data, tvb, offset, next_field)
            else
                tree:append_text(" [H.264 Image Frame]")
                offset = tree_add(tree, f.image_data_size, tvb, offset)
                local data_len = tvb(offset - 2, 2):le_uint()
                offset, item = tree_add(tree, f.image_data, tvb, offset, data_len)
            end
        end
    else
        local boardName = invert_enum(BoardID)[board_id] or ("Board " .. board_id)
        local sinkName = invert_enum(SinkID)[sink_id] or ("Sink " .. sink_id)
        tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Reply (" .. boardName .. "/" .. sinkName .. ")")
        offset, item = tree_add(tree, f.raw, tvb, offset, tvb:len() - offset)
    end

    return offset
end

-- Dispatch for Command (msg type 11)
local function dissect_command(tvb, tree)
    local offset = 0
    local board_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.board_id, tvb, offset)
    local sink_id = tvb(offset, 1):uint()
    offset = tree_add(tree, f.sink_id, tvb, offset)
    local func = tvb(offset, 1):uint()

    -- Use the appropriate function field based on sink_id
    if board_id == BoardID.COM and sink_id == SinkID.COM_SWITCHES then
        offset = tree_add(tree, f.com_switches_func, tvb, offset)
    elseif board_id == BoardID.COM and sink_id == SinkID.COM_AIS then
        offset = tree_add(tree, f.com_ais_func, tvb, offset)
    elseif board_id == BoardID.PMS and sink_id == SinkID.PMS_SWITCHES then
        offset = tree_add(tree, f.pms_switches_func, tvb, offset)
    elseif board_id == BoardID.MOTOR and sink_id == SinkID.MOTOR_PROPULSION then
        offset = tree_add(tree, f.motor_propulsion_func, tvb, offset)
    elseif board_id == BoardID.PC and sink_id == SinkID.PC_CAMERA_MGR then
        offset = tree_add(tree, f.pc_camera_mgr_func, tvb, offset)
    else
        offset = tree_add(tree, f.func, tvb, offset)
    end

    -- Use direct enum comparisons instead of descriptive strings
    if board_id == BoardID.PC and sink_id == SinkID.PC_CAMERA_MGR then
        if func == PC_CAMERA_MGR_FunctionID.START then
            tree:append_text(" [PC/Camera Start Stream]")
            offset = tree_add(tree, f.camera_location, tvb, offset)
            offset = tree_add(tree, f.resolution, tvb, offset)
            offset = tree_add(tree, f.period, tvb, offset)
            offset = tree_add(tree, f.send_stream, tvb, offset)
        elseif func == PC_CAMERA_MGR_FunctionID.STOP then
            tree:append_text(" [PC/Camera Stop Stream Specific]")
            offset = tree_add(tree, f.camera_location, tvb, offset)
            offset = tree_add(tree, f.send_stream, tvb, offset)
        elseif func == PC_CAMERA_MGR_FunctionID.STOP_ALL then
            tree:append_text(" [PC/Camera Stop All Streams]")
        elseif func == PC_CAMERA_MGR_FunctionID.PTZ then
            tree:append_text(" [PC/Camera PTZ Command]")
            offset = tree_add(tree, f.ptz_command, tvb, offset)
            offset = tree_add(tree, f.ptz_parameter, tvb, offset)
        end
    elseif board_id == BoardID.MOTOR then
        if sink_id == SinkID.MOTOR_NAVIGATOR then
            if func == MOTOR_PROPULSION_FunctionID.CHANGE_STATE then
                tree:append_text(" [MOTOR/Navigation Change State]")
                offset = tree_add(tree, f.new_state, tvb, offset, 4)
            elseif func == MOTOR_PROPULSION_FunctionID.MOVE_TO_POINT then
                tree:append_text(" [MOTOR/Navigation Move to Point]")
                offset = tree_add(tree, f.latitude, tvb, offset, 8)
                offset = tree_add(tree, f.longitude, tvb, offset, 8)
                offset = tree_add(tree, f.speed, tvb, offset, 4)
            end
        elseif sink_id == SinkID.MOTOR_PROPULSION then
            if func == MOTOR_PROPULSION_FunctionID.RUDDER_ANGLE_RPM then
                tree:append_text(" [MOTOR/Propulsion Rudder Angle and RPM]")
                offset = tree_add(tree, f.rudder_angle, tvb, offset, 4)
                offset = tree_add(tree, f.rpm, tvb, offset, 4)
            elseif func == MOTOR_PROPULSION_FunctionID.HEADING_RPM then
                tree:append_text(" [MOTOR/Propulsion Heading and RPM]")
                offset = tree_add(tree, f.heading, tvb, offset, 4)
                offset = tree_add(tree, f.rpm, tvb, offset, 4)
            end
        end
    else
        local boardName = invert_enum(BoardID)[board_id] or ("Board " .. board_id)
        local sinkName = invert_enum(SinkID)[sink_id] or ("Sink " .. sink_id)
        tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Command (" .. boardName .. "/" .. sinkName .. ")")
        offset, item = tree_add(tree, f.raw, tvb, offset, tvb:len() - offset)
    end

    return offset
end

----------------------------------------
-- Main Dissector Function
----------------------------------------
function proto.dissector(tvb, pinfo, tree)
    pinfo.cols.info = "SeaTrac Message"
    local subtree = tree:add(proto, tvb(), "SeaTrac Protocol Data")
    local offset = 0

    -- Header (per the document: sync bytes, length, relay, message type)
    offset = tree_add(subtree, f.sync1, tvb, offset)
    offset = tree_add(subtree, f.sync2, tvb, offset)
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
        local _, item = tree_add(subtree, f.raw, payload_tvb, 0, payload_tvb:len())
    end

    -- Checksum (2 bytes as uint16)
    offset = 6 + data_length -- Reset offset to point to the checksum bytes
    offset = tree_add(subtree, f.checksum, tvb, offset)
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
