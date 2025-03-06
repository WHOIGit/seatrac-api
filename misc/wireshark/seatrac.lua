-- SeaTrac dissector – extended per SeaTrac Interface Rev4.docx
-- This implementation covers the major message types and fields as described in the document.

local proto = Proto("seatrac", "SeaTrac Message")

----------------------------------------
-- Enum Tables (from Interface Rev4.docx)
----------------------------------------
local msg_types = {
    [7]  = "Status Request",
    [8]  = "Status Reply",
    [9]  = "Request",
    [10] = "Reply",
    [11] = "Command"
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

-- New enum tables for Board IDs and Sink IDs (brief descriptions from the document)
local board_ids = {
    [1]  = "AIS",
    [8]  = "CAN Bus",
    [10] = "Video/Camera"
}

local sink_ids = {
    [26] = "AIS",
    [64] = "Power/Battery",
    [76] = "Attitude Main",
    [77] = "GPS Main",
    [78] = "Wind Sensor",
    [79] = "Attitude Backup",
    [80] = "GPS Backup",
    [82] = "Propulsion",
    [83] = "Navigation",
    [86] = "IMU",
    [95] = "Video/Camera"
}

-- Helper function for reverse lookup on Board and Sink IDs.
local function get_board_sink_desc(board_id, sink_id)
    local bdesc = board_ids[board_id] or ("Board " .. board_id)
    local sdesc = sink_ids[sink_id] or ("Sink " .. sink_id)
    return bdesc, sdesc, bdesc .. " / " .. sdesc
end

----------------------------------------
-- Proto Fields (header & common)
----------------------------------------
local f               = proto.fields
f.sync1               = ProtoField.uint8("seatrac.sync1", "Sync Byte 1", base.HEX)
f.sync2               = ProtoField.uint8("seatrac.sync2", "Sync Byte 2", base.HEX)
f.length              = ProtoField.uint16("seatrac.length", "Data Length", base.DEC)
f.relay               = ProtoField.uint8("seatrac.relay", "Message Relay", base.DEC)
f.msg_type            = ProtoField.uint8("seatrac.msg_type", "Message Type", base.DEC, msg_types)
f.check1              = ProtoField.uint8("seatrac.check1", "Checksum Byte 1", base.HEX)
f.check2              = ProtoField.uint8("seatrac.check2", "Checksum Byte 2", base.HEX)

-- Common fields used in payloads:
f.sink_id             = ProtoField.uint8("seatrac.sink_id", "SinkID", base.DEC)
f.board_id            = ProtoField.uint8("seatrac.board_id", "BoardID", base.DEC)
f.func                = ProtoField.uint8("seatrac.func", "Function", base.DEC)
f.raw                 = ProtoField.bytes("seatrac.raw", "Raw Data")
f.new_state           = ProtoField.int32("seatrac.new_state", "New State")
f.rudder_angle        = ProtoField.float("seatrac.rudder_angle", "Rudder Angle (deg)")
f.rpm                 = ProtoField.float("seatrac.rpm", "RPM")
f.heading             = ProtoField.float("seatrac.heading", "Heading")
f.speed               = ProtoField.float("seatrac.speed", "Speed (kts)")

-- DATETIME structure fields (8 bytes as defined in document)
f.datetime            = ProtoField.none("seatrac.datetime", "DATETIME")
f.datetime_year       = ProtoField.uint16("seatrac.datetime.year", "Year", base.DEC)
f.datetime_month      = ProtoField.uint8("seatrac.datetime.month", "Month", base.DEC)
f.datetime_day        = ProtoField.uint8("seatrac.datetime.day", "Day", base.DEC)
f.datetime_hour       = ProtoField.uint8("seatrac.datetime.hour", "Hour", base.DEC)
f.datetime_minute     = ProtoField.uint8("seatrac.datetime.minute", "Minute", base.DEC)
f.datetime_second     = ProtoField.uint8("seatrac.datetime.second", "Second", base.DEC)
f.datetime_hundredths = ProtoField.uint8("seatrac.datetime.hundredths", "Hundredths", base.DEC)

-- Power level fields
f.pack_current        = ProtoField.int16("seatrac.pack_current", "Pack Current", base.DEC, nil, 0, "×0.002")
f.load_current        = ProtoField.int16("seatrac.load_current", "Load Current", base.DEC, nil, 0, "×0.002")
f.pack_voltage        = ProtoField.uint16("seatrac.pack_voltage", "Pack Voltage", base.DEC, nil, 0, "×0.001")
f.soc                 = ProtoField.uint16("seatrac.soc", "SOC%", base.DEC, nil, 0, "×0.002")
f.unknown2            = ProtoField.bytes("seatrac.unknown2", "Unknown Fields")

-- AIS fields
f.ais_state           = ProtoField.uint8("seatrac.ais.state", "AIS State", base.DEC, { [0] = "Off", [1] = "On" })
f.ais_num_targets     = ProtoField.uint8("seatrac.ais.num_targets", "Number of AIS Targets", base.DEC)
f.ais_closest_target  = ProtoField.float("seatrac.ais.closest_target", "Closest AIS Target (nm)")

-- Attitude fields (shared by several messages)
f.pitch               = ProtoField.int16("seatrac.attitude.pitch", "Pitch", base.DEC, nil, 0, "×0.01")
f.min_pitch           = ProtoField.int16("seatrac.attitude.min_pitch", "Min Pitch", base.DEC, nil, 0, "×0.01")
f.max_pitch           = ProtoField.int16("seatrac.attitude.max_pitch", "Max Pitch", base.DEC, nil, 0, "×0.01")
f.roll                = ProtoField.int16("seatrac.attitude.roll", "Roll", base.DEC, nil, 0, "×0.01")
f.min_roll            = ProtoField.int16("seatrac.attitude.min_roll", "Min Roll", base.DEC, nil, 0, "×0.01")
f.max_roll            = ProtoField.int16("seatrac.attitude.max_roll", "Max Roll", base.DEC, nil, 0, "×0.01")
f.min_heading         = ProtoField.int16("seatrac.attitude.min_heading", "Min Heading", base.DEC, nil, 0, "×0.01")
f.max_heading         = ProtoField.int16("seatrac.attitude.max_heading", "Max Heading", base.DEC, nil, 0, "×0.01")

-- GPS fields
f.latitude            = ProtoField.double("seatrac.gps.latitude", "Latitude", base.NONE, nil, "radians")
f.longitude           = ProtoField.double("seatrac.gps.longitude", "Longitude", base.NONE, nil, "radians")
f.kts                 = ProtoField.uint16("seatrac.gps.kts", "Speed over Ground", base.DEC, nil, 0, "×0.002")
f.current_kts         = ProtoField.uint16("seatrac.gps.current_kts", "Current Speed", base.DEC, nil, 0, "×0.002")
f.current_heading     = ProtoField.uint16("seatrac.gps.current_heading", "Current Heading", base.DEC, nil, 0, "×0.01")
f.wind_kts            = ProtoField.uint16("seatrac.gps.wind_kts", "Wind Speed", base.DEC, nil, 0, "×0.002")
f.wind_heading        = ProtoField.uint16("seatrac.gps.wind_heading", "Wind Heading", base.DEC, nil, 0, "×0.01")

-- IMU fields
f.roll_gyro           = ProtoField.int16("seatrac.imu.roll_gyro", "Roll Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")
f.pitch_gyro          = ProtoField.int16("seatrac.imu.pitch_gyro", "Pitch Gyro Rate", base.DEC, nil, 0, "×0.02 deg/sec")
f.heading_gyro        = ProtoField.int16("seatrac.imu.heading_gyro", "Heading Gyro Rate", base.DEC, nil, 0,
    "×0.02 deg/sec")
f.accel_x             = ProtoField.int16("seatrac.imu.accel_x", "Acceleration X", base.DEC, nil, 0, "×0.01 m/s²")
f.accel_y             = ProtoField.int16("seatrac.imu.accel_y", "Acceleration Y", base.DEC, nil, 0, "×0.01 m/s²")
f.accel_z             = ProtoField.int16("seatrac.imu.accel_z", "Acceleration Z", base.DEC, nil, 0, "×0.01 m/s²")
f.max_accel_x         = ProtoField.int16("seatrac.imu.max_accel_x", "Max Accel X", base.DEC, nil, 0, "×0.01 m/s²")
f.max_accel_y         = ProtoField.int16("seatrac.imu.max_accel_y", "Max Accel Y", base.DEC, nil, 0, "×0.01 m/s²")
f.max_accel_z         = ProtoField.int16("seatrac.imu.max_accel_z", "Max Accel Z", base.DEC, nil, 0, "×0.01 m/s²")
f.heave               = ProtoField.int16("seatrac.imu.heave", "Heave", base.DEC, nil, 0, "×0.001 m")
f.min_z               = ProtoField.int16("seatrac.imu.min_z", "Min Z", base.DEC, nil, 0, "×0.001 m")
f.max_z               = ProtoField.int16("seatrac.imu.max_z", "Max Z", base.DEC, nil, 0, "×0.001 m")

-- Wind sensor extra fields
f.apparent_speed      = ProtoField.uint16("seatrac.wind.apparent_speed", "Apparent Wind Speed", base.DEC, nil, 0,
    "×0.002")
f.apparent_angle      = ProtoField.int16("seatrac.wind.apparent_angle", "Apparent Wind Angle", base.DEC, nil, 0, "×0.01")
f.temperature         = ProtoField.int16("seatrac.wind.temperature", "Air Temperature", base.DEC, nil, 0, "×0.01 °C")
f.pressure            = ProtoField.int16("seatrac.wind.pressure", "Air Pressure", base.DEC, nil, 0, "×0.01 Bar")

-- Propulsion fields
f.prop_state          = ProtoField.int16("seatrac.prop.state", "Propulsion State", base.DEC, propulsion_states, 0)
f.target_rpm          = ProtoField.int16("seatrac.prop.target_rpm", "Target RPM", base.DEC)
f.target_rudder       = ProtoField.int16("seatrac.prop.target_rudder", "Target Rudder Angle", base.DEC, nil, 0,
    "×0.01 deg")
f.actual_rudder       = ProtoField.int16("seatrac.prop.actual_rudder", "Actual Rudder Angle", base.DEC, nil, 0,
    "×0.01 deg")
f.actual_rudder_speed = ProtoField.int16("seatrac.prop.actual_rudder_speed", "Actual Rudder Speed", base.DEC, nil, 0,
    "×0.01 deg/sec")

-- Navigation fields
f.mission_state       = ProtoField.int16("seatrac.nav.mission_state", "Mission State", base.DEC, navigator_states)
f.requested_speed     = ProtoField.uint16("seatrac.nav.requested_speed", "Requested Speed", base.DEC, nil, 0,
    "×0.002 kts")
f.n_waypoints         = ProtoField.uint16("seatrac.nav.n_waypoints", "Number of Waypoints", base.DEC)
f.next_wp_index       = ProtoField.uint16("seatrac.nav.next_wp_index", "Next Waypoint Index", base.HEX)
f.leg_start_lat       = ProtoField.double("seatrac.nav.leg_start_lat", "Leg Start Latitude", base.NONE, nil, "radians")
f.leg_start_long      = ProtoField.double("seatrac.nav.leg_start_long", "Leg Start Longitude", base.NONE, nil, "radians")
f.leg_end_lat         = ProtoField.double("seatrac.nav.leg_end_lat", "Leg End Latitude", base.NONE, nil, "radians")
f.leg_end_long        = ProtoField.double("seatrac.nav.leg_end_long", "Leg End Longitude", base.NONE, nil, "radians")
f.laps_to_do          = ProtoField.uint16("seatrac.nav.laps_to_do", "Laps To Do", base.DEC)
f.laps_done           = ProtoField.uint16("seatrac.nav.laps_done", "Laps Done", base.DEC)
f.hold_lat            = ProtoField.double("seatrac.nav.hold_lat", "Hold Latitude", base.NONE, nil, "radians")
f.hold_long           = ProtoField.double("seatrac.nav.hold_long", "Hold Longitude", base.NONE, nil, "radians")
f.hold_rin            = ProtoField.float("seatrac.nav.hold_rin", "Hold Rin (ft)")
f.hold_rout           = ProtoField.float("seatrac.nav.hold_rout", "Hold Rout (ft)")
f.hold_kts            = ProtoField.float("seatrac.nav.hold_kts", "Hold Speed (kts)")
f.hold_time           = ProtoField.uint32("seatrac.nav.hold_time", "Hold Time (secs)")
f.alt_wp_active       = ProtoField.uint8("seatrac.nav.alt_wp_active", "Alternate Waypoint Active")
f.alt_wp_lat          = ProtoField.double("seatrac.nav.alt_wp_lat", "Alternate Waypoint Latitude", base.NONE, nil,
    "radians")
f.alt_wp_long         = ProtoField.double("seatrac.nav.alt_wp_long", "Alternate Waypoint Longitude", base.NONE, nil,
    "radians")
f.alt_heading_active  = ProtoField.uint8("seatrac.nav.alt_heading_active", "Alternate Heading Active")
f.alt_heading         = ProtoField.float("seatrac.nav.alt_heading", "Alternate Heading")
f.alt_speed_active    = ProtoField.uint8("seatrac.nav.alt_speed_active", "Alternate Speed Active")
f.alt_speed           = ProtoField.float("seatrac.nav.alt_speed", "Alternate Speed (kts)")

-- Video / Camera fields
f.camera_location     = ProtoField.uint8("seatrac.camera.location", "Camera Location", base.DEC, camera_locations)
f.resolution          = ProtoField.uint8("seatrac.camera.resolution", "Resolution", base.DEC, resolutions)
f.period              = ProtoField.uint16("seatrac.camera.period", "Period", base.DEC)
f.send_stream         = ProtoField.uint8("seatrac.camera.send_stream", "Send Stream", base.DEC, send_streams)
f.nchunks             = ProtoField.uint16("seatrac.camera.nchunks", "Total Chunks", base.DEC)
f.chunk               = ProtoField.uint16("seatrac.camera.chunk", "Chunk Number", base.DEC)
f.setup_frame_size    = ProtoField.uint16("seatrac.camera.setup_frame_size", "Setup Frame Size", base.DEC)
f.setup_frame_data    = ProtoField.bytes("seatrac.camera.setup_frame_data", "Setup Frame Data")
f.strength            = ProtoField.float("seatrac.camera.strength", "Strength")
f.zoom                = ProtoField.float("seatrac.camera.zoom", "Zoom")
f.camera_height       = ProtoField.float("seatrac.camera.height", "Camera Height (ft)")
f.ptz_azimuth         = ProtoField.float("seatrac.camera.ptz_azimuth", "PTZ Azimuth")
f.ptz_elevation       = ProtoField.float("seatrac.camera.ptz_elevation", "PTZ Elevation")
f.ptz_view_angle      = ProtoField.float("seatrac.camera.ptz_view_angle", "PTZ View Angle")
f.image_rotation      = ProtoField.float("seatrac.camera.image_rotation", "Image Rotation")
f.image_height        = ProtoField.float("seatrac.camera.image_height", "Image Height")
f.image_data_size     = ProtoField.uint16("seatrac.camera.image_data_size", "Image Data Size", base.DEC)
f.image_data          = ProtoField.bytes("seatrac.camera.image_data", "Image Data")

----------------------------------------
-- Helper Functions
----------------------------------------

-- Parse DATETIME structure (8 bytes, per the interface document)
local function dissect_datetime(tvb, offset, tree, label)
    local dt_tree = tree:add(f.datetime, tvb(offset, 8), label or "DATETIME")
    local year = tvb(offset, 2):le_uint() + 1900
    dt_tree:add(f.datetime_year, tvb(offset, 2)):append_text(" (" .. year .. ")")
    offset = offset + 2
    dt_tree:add(f.datetime_month, tvb(offset, 1))
    offset = offset + 1
    dt_tree:add(f.datetime_day, tvb(offset, 1))
    offset = offset + 1
    dt_tree:add(f.datetime_hour, tvb(offset, 1))
    offset = offset + 1
    dt_tree:add(f.datetime_minute, tvb(offset, 1))
    offset = offset + 1
    dt_tree:add(f.datetime_second, tvb(offset, 1))
    offset = offset + 1
    dt_tree:add(f.datetime_hundredths, tvb(offset, 1))
    offset = offset + 1
    return offset
end

-- Dispatch for Status Request (msg type 7) – minimal example.
local function dissect_status_request(tvb, tree)
    tree:append_text(" [Status Request]")
    -- (No additional fields defined in the document for Status Request.)
    tree:add(f.raw, tvb())
    return tvb:len()
end

-- Dispatch for Status Reply (msg type 8)
local function dissect_status_reply(tvb, tree)
    local offset = 0
    local sink_id = tvb(offset, 1):uint()
    tree:add(f.sink_id, tvb(offset, 1))
    offset = offset + 1

    if sink_id == 64 then
        tree:append_text(" [Power Level]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.raw, tvb(offset, 18)):set_text("Unknown Fields (bytes 16–33)")
        offset = offset + 18
        tree:add(f.pack_current, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.load_current, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.pack_voltage, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.soc, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.unknown2, tvb(offset, 4))
        offset = offset + 4
    elseif sink_id == 26 then
        tree:append_text(" [AIS Status Reply]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.ais_state, tvb(offset, 1))
        offset = offset + 1
        tree:add(f.ais_num_targets, tvb(offset, 1))
        offset = offset + 1
        tree:add(f.ais_closest_target, tvb(offset, 4))
        offset = offset + 4
    elseif sink_id == 76 or sink_id == 79 then
        tree:append_text(" [Attitude]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.pitch, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.min_pitch, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.max_pitch, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.roll, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.min_roll, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.max_roll, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.min_heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.max_heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
    elseif sink_id == 77 or sink_id == 80 then
        tree:append_text(" [GPS]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.latitude, tvb(offset, 8))
        offset = offset + 8
        tree:add(f.longitude, tvb(offset, 8))
        offset = offset + 8
        tree:add(f.kts, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.current_kts, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.current_heading, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.wind_kts, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.wind_heading, tvb(offset, 2))
        offset = offset + 2
    elseif sink_id == 86 then
        tree:append_text(" [IMU]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.roll, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.min_roll, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.max_roll, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.pitch, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.min_pitch, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.max_pitch, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.roll_gyro, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.pitch_gyro, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.heading_gyro, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.accel_x, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.accel_y, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.accel_z, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.max_accel_x, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.max_accel_y, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.max_accel_z, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.heave, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.min_z, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.max_z, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.min_heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
        tree:add(f.max_heading, tvb(offset, 2)):append_text(" (×0.01)")
        offset = offset + 2
    elseif sink_id == 78 then
        tree:append_text(" [Wind Sensor]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.apparent_speed, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.apparent_angle, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.temperature, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.pressure, tvb(offset, 2))
        offset = offset + 2
    elseif sink_id == 82 then
        tree:append_text(" [Propulsion]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.prop_state, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.target_rpm, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.target_rudder, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.target_heading, tvb(offset, 2))
        offset = offset + 2
        offset = offset + 6 -- skip reserved bytes (3 shorts)
        tree:add(f.actual_rudder, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.actual_rudder_speed, tvb(offset, 2))
        offset = offset + 2
    elseif sink_id == 83 then
        tree:append_text(" [Navigation]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.mission_state, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.requested_speed, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.target_rpm, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.n_waypoints, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.next_wp_index, tvb(offset, 2))
        offset = offset + 2
        if tvb:len() - offset >= 32 then
            tree:add(f.leg_start_lat, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.leg_start_long, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.leg_end_lat, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.leg_end_long, tvb(offset, 8))
            offset = offset + 8
        end
        if tvb:len() - offset >= 4 then
            tree:add(f.laps_to_do, tvb(offset, 2))
            offset = offset + 2
            tree:add(f.laps_done, tvb(offset, 2))
            offset = offset + 2
        end
        -- Additional fields if holding (mission state == 6) or running a mission (state == 7)
        if tvb:len() - offset >= 8 + 8 + 4 + 4 + 4 then
            tree:add(f.hold_lat, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.hold_long, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.hold_rin, tvb(offset, 4))
            offset = offset + 4
            tree:add(f.hold_rout, tvb(offset, 4))
            offset = offset + 4
            tree:add(f.hold_kts, tvb(offset, 4))
            offset = offset + 4
        end
        if tvb:len() - offset >= 4 + 4 + 1 + 16 + 1 + 4 + 1 + 4 then
            tree:add(f.hold_time, tvb(offset, 4))
            offset = offset + 4
            tree:add(f.hold_rout, tvb(offset, 4))
            offset = offset + 4
            tree:add(f.alt_wp_active, tvb(offset, 1))
            offset = offset + 1
            tree:add(f.alt_wp_lat, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.alt_wp_long, tvb(offset, 8))
            offset = offset + 8
            tree:add(f.alt_heading_active, tvb(offset, 1))
            offset = offset + 1
            tree:add(f.alt_heading, tvb(offset, 4))
            offset = offset + 4
            tree:add(f.alt_speed_active, tvb(offset, 1))
            offset = offset + 1
            tree:add(f.alt_speed, tvb(offset, 4))
            offset = offset + 4
        end
    else
        tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Status Reply SinkID: " .. sink_id)
    end
    return offset
end

-- Dispatch for Request (msg type 9) – for brevity we show raw payload.
local function dissect_request(tvb, tree)
    tree:append_text(" [Request]")
    tree:add(f.raw, tvb())
    return tvb:len()
end

-- Dispatch for Reply (msg type 10)
local function dissect_reply(tvb, tree)
    local offset = 0
    local board_id = tvb(offset, 1):uint()
    tree:add(f.board_id, tvb(offset, 1))
    offset = offset + 1
    local sink_id = tvb(offset, 1):uint()
    tree:add(f.sink_id, tvb(offset, 1))
    offset = offset + 1
    local func = tvb(offset, 1):uint()
    tree:add(f.func, tvb(offset, 1))
    offset = offset + 1

    local board_desc, sink_desc, bs_desc = get_board_sink_desc(board_id, sink_id)
    if board_desc == "AIS" and sink_desc == "AIS" then
        tree:append_text(" [" .. bs_desc .. " AIS Target Update]")
        offset = dissect_datetime(tvb, offset, tree, "Timestamp")
        tree:add(f.latitude, tvb(offset, 8))
        offset = offset + 8
        tree:add(f.longitude, tvb(offset, 8))
        offset = offset + 8
        tree:add(ProtoField.uint32("seatrac.ais.mmsi", "MMSI"), tvb(offset, 4))
        offset = offset + 4
        tree:add(ProtoField.uint16("seatrac.ais.speed_over_ground", "Speed over Ground", base.DEC, nil, 0, "×0.002"),
            tvb(offset, 2))
        offset = offset + 2
        tree:add(ProtoField.uint16("seatrac.ais.course_over_ground", "Course over Ground", base.DEC, nil, 0, "×0.01"),
            tvb(offset, 2))
        offset = offset + 2
        tree:add(ProtoField.uint16("seatrac.ais.heading", "Heading", base.DEC), tvb(offset, 2))
        offset = offset + 2
        tree:add(ProtoField.uint8("seatrac.ais.nav_status", "Navigation Status", base.DEC), tvb(offset, 1))
        offset = offset + 1
    elseif board_desc == "Video/Camera" and sink_desc == "Video/Camera" and func == 5 then
        tree:append_text(" [" .. bs_desc .. " Image Data Start]")
        tree:add(f.camera_location, tvb(offset, 1), camera_locations)
        offset = offset + 1
        tree:add(f.send_stream, tvb(offset, 1), send_streams)
        offset = offset + 1
        tree:add(f.nchunks, tvb(offset, 2))
        offset = offset + 2
        tree:add(f.chunk, tvb(offset, 2))
        offset = offset + 2
        if tvb:len() - offset >= 2 then
            local next_field = tvb(offset, 2):le_uint()
            if next_field > 0 and next_field < 10000 then
                tree:append_text(" [H.264 Setup Frame]")
                tree:add(f.setup_frame_size, tvb(offset, 2))
                offset = offset + 2
                tree:add(f.setup_frame_data, tvb(offset, next_field))
                offset = offset + next_field
            else
                tree:append_text(" [H.264 Image Frame]")
                tree:add(f.image_data_size, tvb(offset, 2))
                offset = offset + 2
                local data_len = tvb(offset - 2, 2):le_uint()
                tree:add(f.image_data, tvb(offset, data_len))
                offset = offset + data_len
            end
        end
    else
        tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Reply (" .. bs_desc .. ")")
        tree:add(f.raw, tvb(offset))
    end

    return offset
end

-- Dispatch for Command (msg type 11)
local function dissect_command(tvb, tree)
    local offset = 0
    local board_id = tvb(offset, 1):uint()
    tree:add(f.board_id, tvb(offset, 1))
    offset = offset + 1
    local sink_id = tvb(offset, 1):uint()
    tree:add(f.sink_id, tvb(offset, 1))
    offset = offset + 1
    local func = tvb(offset, 1):uint()
    tree:add(f.func, tvb(offset, 1))
    offset = offset + 1

    local board_desc, sink_desc, bs_desc = get_board_sink_desc(board_id, sink_id)
    if board_desc == "Video/Camera" and sink_desc == "Video/Camera" then
        if func == 2 then
            tree:append_text(" [" .. bs_desc .. " Start Stream]")
            tree:add(f.camera_location, tvb(offset, 1), camera_locations)
            offset = offset + 1
            tree:add(f.resolution, tvb(offset, 1), resolutions)
            offset = offset + 1
            tree:add(f.period, tvb(offset, 2))
            offset = offset + 2
            tree:add(f.send_stream, tvb(offset, 1), send_streams)
            offset = offset + 1
        elseif func == 3 then
            tree:append_text(" [" .. bs_desc .. " Stop Stream Specific]")
            tree:add(f.camera_location, tvb(offset, 1), camera_locations)
            offset = offset + 1
            tree:add(f.send_stream, tvb(offset, 1), send_streams)
            offset = offset + 1
        elseif func == 4 then
            tree:append_text(" [" .. bs_desc .. " Stop All Streams]")
        elseif func == 5 then
            tree:append_text(" [" .. bs_desc .. " PTZ Command]")
            tree:add(ProtoField.uint8("seatrac.ptz.command", "PTZ Command", base.DEC, ptz_commands), tvb(offset, 1))
            offset = offset + 1
            tree:add(ProtoField.int16("seatrac.ptz.parameter", "PTZ Parameter", base.DEC), tvb(offset, 2))
            offset = offset + 2
        end
    elseif board_desc == "CAN Bus" then
        if sink_desc == "Navigation" then
            if func == 5 then
                tree:append_text(" [" .. bs_desc .. " Change State]")
                tree:add(f.new_state, tvb(offset, 4))
                offset = offset + 4
            elseif func == 7 then
                tree:append_text(" [" .. bs_desc .. " Move to Point]")
                tree:add(f.latitude, tvb(offset, 8))
                offset = offset + 8
                tree:add(f.longitude, tvb(offset, 8))
                offset = offset + 8
                tree:add(f.speed, tvb(offset, 4))
                offset = offset + 4
            end
        elseif sink_desc == "Propulsion" then
            if func == 1 then
                tree:append_text(" [" .. bs_desc .. " Rudder Angle and RPM]")
                tree:add(f.rudder_angle, tvb(offset, 4))
                offset = offset + 4
                tree:add(f.rpm, tvb(offset, 4))
                offset = offset + 4
            elseif func == 2 then
                tree:append_text(" [" .. bs_desc .. " Heading and RPM]")
                tree:add(f.heading, tvb(offset, 4))
                offset = offset + 4
                tree:add(f.rpm, tvb(offset, 4))
                offset = offset + 4
            end
        end
    else
        tree:add_expert_info(PI_UNDECODED, PI_NOTE, "Unknown Command (" .. bs_desc .. ")")
        tree:add(f.raw, tvb(offset))
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
    subtree:add(f.sync1, tvb(offset, 1)); offset = offset + 1
    subtree:add(f.sync2, tvb(offset, 1)); offset = offset + 1
    local data_length = tvb(offset, 2):le_uint()
    subtree:add_le(f.length, tvb(offset, 2)); offset = offset + 2
    subtree:add(f.relay, tvb(offset, 1)); offset = offset + 1
    local mtype = tvb(offset, 1):uint()
    subtree:add(f.msg_type, tvb(offset, 1)); offset = offset + 1

    -- Payload is data_length bytes; dispatch based on Message Type:
    local payload_tvb = tvb(offset, data_length)
    if mtype == 7 then
        offset = offset + dissect_status_request(payload_tvb, subtree)
    elseif mtype == 8 then
        offset = offset + dissect_status_reply(payload_tvb, subtree)
    elseif mtype == 9 then
        offset = offset + dissect_request(payload_tvb, subtree)
    elseif mtype == 10 then
        offset = offset + dissect_reply(payload_tvb, subtree)
    elseif mtype == 11 then
        offset = offset + dissect_command(payload_tvb, subtree)
    else
        subtree:add_expert_info(PI_UNDECODED, PI_WARN, "Unknown Message Type")
        subtree:add(f.raw, payload_tvb)
    end

    -- Checksum bytes (Check1 and Check2)
    subtree:add(f.check1, tvb(6 + data_length, 1))
    subtree:add(f.check2, tvb(6 + data_length + 1, 1))
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
