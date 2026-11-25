import datetime
import time
import uuid

import pytest
import rclpy

import seatrac.protocol as proto

from rclpy.node import Node
from rclpy.parameter import Parameter
from seatrac.msg import OutletStatus, PowerLevel
from seatrac_node import configure_node
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData


@pytest.fixture
def node():
    rclpy.init(args=[])
    driver = Node('seatrac', namespace='test_' + uuid.uuid4().hex,
                  parameter_overrides=[Parameter('outlets.starlink', value=15)],
                  automatically_declare_parameters_from_overrides=True)
    try:
        configure_node(driver)
        yield driver
    finally:
        driver.destroy_node()
        rclpy.shutdown()


def spin_until(node, predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        assert time.monotonic() < deadline, 'Timed out waiting for ROS topics'
        rclpy.spin_once(node, timeout_sec=0.05)


def status_packet(sink, payload, timestamp=None):
    return bytes(proto.SeaTracMessage(
        outbound_relay=proto.Relay.CAN_BUS,
        return_relay=proto.Relay.DEFAULT,
        msg_type=proto.MessageType.STATUS_REPLY,
        sink_id=sink,
        timestamp=timestamp,
        payload=payload,
    ))


def test_driver_topics(node):
    power, status, outgoing = [], [], []
    node.create_subscription(PowerLevel, '~/power', power.append, 10)
    node.create_subscription(OutletStatus, '~/outlet/starlink/status', status.append, 10)
    node.create_subscription(RawData, '~/out', outgoing.append, 10)
    incoming = node.create_publisher(RawData, '~/in', 10)
    control = node.create_publisher(Bool, '~/outlet/starlink/control', 10)
    spin_until(node, lambda: all(p.get_subscription_count() for p in node.publishers
                                 if p.topic_name.startswith(node.get_namespace() + '/')))

    timestamp = datetime.datetime(2025, 3, 7, 21, 23, 20, 250000,
                                  tzinfo=datetime.timezone.utc)
    power_packet = status_packet(proto.SinkID.PMS_BMS, proto.PowerLevelMessage(
        pack_current=-0.5, load_current=1.0, pack_voltage=40.0, soc_percentage=85.5
    ), timestamp)
    switches = proto.PMSSwitchStatusMessage()
    switches.states[15 - 1] = True
    switch_packet = status_packet(proto.SinkID.PMS_SWITCHES, switches)

    # The bridge can split one packet and deliver the next in the same read.
    incoming.publish(RawData(data=list(power_packet[:5])))
    incoming.publish(RawData(data=list(power_packet[5:] + switch_packet)))
    control.publish(Bool(data=True))
    spin_until(node, lambda: power and status and outgoing)

    battery = power[0]
    assert (battery.pack_current, battery.load_current,
            battery.pack_voltage, battery.soc_percentage) == (-0.5, 1.0, 40.0, 85.5)
    assert (battery.header.stamp.sec, battery.header.stamp.nanosec) == (1741382600, 250000000)
    assert battery.ds_header.io_time.sec > 0
    assert status[0].is_active
    assert status[0].header.stamp == status[0].ds_header.io_time
    assert outgoing[0].data_direction == RawData.DIRECTION_DATA_OUT
    assert outgoing[0].io_stamp.sec > 0
    expected = proto.SeaTracMessage(
        outbound_relay=proto.Relay.CAN_BUS,
        return_relay=proto.Relay.DEFAULT,
        msg_type=proto.MessageType.COMMAND,
        board_id=proto.BoardID.PMS,
        sink_id=proto.SinkID.PMS_SWITCHES,
        function_id=proto.PMS_SWITCHES_FunctionID.SET,
        payload=proto.SwitchSetCommand(switch=15, state=True),
    )
    assert bytes(outgoing[0].data) == bytes(expected)
