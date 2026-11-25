import time
import uuid

import pytest
import rclpy
from rclpy.parameter import Parameter
from seatrac.msg import OutletStatus, PowerLevel
from seatrac_node import SeaTracNode
from std_msgs.msg import Bool
from wr2_msgs.msg import RawData


# A timestamped PMS battery report and an undated switch report with outlet 15 on.
POWER = bytes.fromhex(
    '00ff2900080840e907030715171419' + '00' * 20 + '06fff401409cfea600000000feb4')
SWITCHES = bytes.fromhex('00ff1e0008083f' + '00' * 26 + '0040006ee3')


@pytest.fixture
def node():
    rclpy.init(args=[])
    driver = SeaTracNode(namespace='test_' + uuid.uuid4().hex,
                         parameter_overrides=[Parameter('outlets.starlink', value=15)])
    try:
        yield driver
    finally:
        driver.destroy_node()
        rclpy.shutdown()


def spin_until(node, predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        assert time.monotonic() < deadline, 'Timed out waiting for ROS topics'
        rclpy.spin_once(node, timeout_sec=0.05)


def test_driver_topics(node):
    power, status, outgoing = [], [], []
    node.create_subscription(PowerLevel, '~/power', power.append, 10)
    node.create_subscription(OutletStatus, '~/outlet/starlink/status', status.append, 10)
    node.create_subscription(RawData, '~/out', outgoing.append, 10)
    incoming = node.create_publisher(RawData, '~/in', 10)
    control = node.create_publisher(Bool, '~/outlet/starlink/control', 10)
    spin_until(node, lambda: all(p.get_subscription_count() for p in node.publishers
                                 if p.topic_name.startswith(node.get_namespace() + '/')))

    # The bridge can split one packet and deliver the next in the same read.
    incoming.publish(RawData(data=list(POWER[:5])))
    incoming.publish(RawData(data=list(POWER[5:] + SWITCHES)))
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
    assert bytes(outgoing[0].data) == bytes.fromhex('00ff0500080b073f000f01aae6')
