#!/usr/bin/env python3
import datetime
import functools

import rospy

from typing import Optional

import seatrac.protocol as proto

from ds_core_msgs.msg import RawData
from std_msgs.msg import Bool
from seatrac.msg import PowerLevel, OutletStatus


def populate_timestamps(msg, timestamp: Optional[datetime.datetime]) -> None:
    # See HEADERS.md in ds_core_msgs for details on the timestamps.
    #     header.stamp: timestamp reported by the boat
    #     ds_header.io_time: timestamp when we received it
    msg.header.stamp = msg.ds_header.io_time = rospy.Time.now()
    if timestamp is not None:
        msg.header.stamp = rospy.Time.from_sec(timestamp.timestamp())


def main() -> None:
    rospy.init_node('seatrac')

    outlets = {
        o['name']: o['outlet']
        for o in rospy.get_param('~outlets', [])
        if 'name' in o and 'outlet' in o
    }

    # Sanity check the outlet numbers
    for name, number in outlets.items():
        if not 1 <= number <= proto.PMSSwitchStatusMessage.NUM_SWITCHES:
            rospy.logerr(f'Outlet {name} has out of range number {number}')
            exit(1)

    if len(outlets) != len(set(outlets.values())):
        rospy.logerr('Duplicate outlet numbers found in configuration')
        exit(1)

    power_pub = rospy.Publisher('~power', PowerLevel, queue_size=10)
    out_pub = rospy.Publisher('~out', RawData, queue_size=10)
    status_pubs = {
        name: rospy.Publisher(f'~outlet/{name}/status', OutletStatus,
                              queue_size=10)
        for name in outlets
    }

    buffer = bytearray()

    def handle_power_level(msg: proto.PowerLevelMessage,
                           timestamp: Optional[datetime.datetime]) -> None:
        ros_msg = PowerLevel()
        populate_timestamps(ros_msg, timestamp)
        ros_msg.pack_current = msg.pack_current
        ros_msg.load_current = msg.load_current
        ros_msg.pack_voltage = msg.pack_voltage
        ros_msg.soc_percentage = msg.soc_percentage
        power_pub.publish(ros_msg)

    def handle_switch_status(msg: proto.PMSSwitchStatusMessage,
                             timestamp: Optional[datetime.datetime]) -> None:
        for name, number in outlets.items():
            ros_msg = OutletStatus()
            populate_timestamps(ros_msg, timestamp)
            ros_msg.is_active = msg.states[number - 1]
            status_pubs[name].publish(ros_msg)

    def handle_seatrac_message(msg: proto.SeaTracMessage) -> None:
        if isinstance(msg.payload, proto.PowerLevelMessage):
            handle_power_level(msg.payload, msg.timestamp)
        elif isinstance(msg.payload, proto.PMSSwitchStatusMessage):
            handle_switch_status(msg.payload, msg.timestamp)

    def handle_raw_packet(in_msg: RawData) -> None:
        buffer.extend(in_msg.data)
        while True:
            try:
                if not (ready := proto.SeaTracMessage.peek_length(buffer)):
                    break
            except ValueError:
                buffer.clear()
                break

            packet = buffer[:ready]
            del buffer[:ready]

            parsed = proto.SeaTracMessage.from_bytes(packet)
            if not parsed.is_checksum_valid:
                rospy.logwarn('Dropped packet with invalid checksum')
                continue  # discard

            handle_seatrac_message(parsed)

    rospy.Subscriber('~in', RawData, handle_raw_packet)


    def handle_outlet_control(outlet: int, msg: Bool) -> None:
        out = RawData()
        out.data_direction = RawData.DATA_OUT
        out.data = bytes(proto.SeaTracMessage(
            relay=0,  # FIXME? Should we use a specific relay?
            msg_type=proto.MessageType.COMMAND,
            board_id=proto.BoardID.PMS,
            sink_id=proto.SinkID.PMS_SWITCHES,
            function_id=proto.PMS_SWITCHES_FunctionID.SET,
            payload=proto.SwitchSetCommand(outlet, msg.data),
        ))
        out.header.stamp = out.ds_header.io_time = rospy.Time.now()
        out_pub.publish(out)

    for name, number in outlets.items():
        rospy.Subscriber(f'~outlet/{name}/control', Bool,
                         functools.partial(handle_outlet_control, number))


    rospy.spin()


if __name__ == '__main__':
    main()
