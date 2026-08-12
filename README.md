# SeaTrac API for Python

This repository contains the `seatrac` Python package for interfacing with a [SeaTrac Systems SP-48 autonomous surface vessel](https://www.seatrac.com), along with a [ROS 1](https://ros.org) node for integration with ROS-based systems.

Documentation of the protocol is out of scope for this repository. Please contact SeaTrac Systems with questions.


## Usage

The `seatrac.protocol` module can parse the headers and validate checksums of all SeaTrac messages. The message payload is usually returned as a `bytes` object, though a small subset of messages can be fully or partially parsed, including:

  - `PowerLevelMessage`: status update related to current draw and state of charge
  - `PMSSwitchStatusMessage`: status update related to power switches
  - `SwitchSetCommand`: command for controlling a power switch

A message object can also be serialized calling `bytes()`.

```python3
from seatrac.protocol import SeaTracMessage
packet = bytes.fromhex('00ff0500080b073f000f01aae6')
msg = SeaTracMessage.from_bytes(packet)

# SeaTracMessage(
#     outbound_relay=<Relay.CAN_BUS: 8>,
#     return_relay=<Relay.DEFAULT: 0>,
#     msg_type=<MessageType.COMMAND: 11>,
#     board_id=<BoardID.PMS: 7>,
#     sink_id=<SinkID.PMS_SWITCHES: 63>,
#     function_id=<PMS_SWITCHES_FunctionID.SET: 0>,
#     timestamp=None,
#     is_checksum_valid=True,
#     payload=SwitchSetCommand(switch=15, state=True)
# )

assert bytes(msg) == packet
```


## Installation

```
python3 -m pip install git+https://github.com/WHOIGit/seatrac-api.git
```


## Command Line

A minimal command line interface is provided for testing purposes.

The command line client can listen to local UDP traffic:

```
python3 -m seatrac --listen --port 62001
```

Or connect to the SeaTrac server using a client certificate. Review [docs/ports.md](docs/ports.md) for the various port assignments.

```
python3 -m seatrac --connect --port 45107 --auth cert.pem key.pem
```

In `--connect` mode, it may be useful to use `--relay host port` which forwards the decrypted traffic as UDP packets to the specified destination.


## Video

The `seatrac.video` module starts a camera stream and reassembles the chunked
H.264 data into an Annex B elementary stream.

```
python3 -m seatrac.video --auth cert.pem key.pem --boat 5 --camera bow | ffplay -
```

Video does not use the link ports listed in [docs/ports.md](docs/ports.md). It
travels over the PC board port belonging to the link, so `--link starlink
--boat 5` connects to port 48104. Use `--resolution` to pick 360p, 720p or
1080p, and `--period` to choose a continuous stream, a frame every so many
seconds, or a single still image.

To hand the stream to a media server, `--publish` runs `ffmpeg` for you:

```
python3 -m seatrac.video --auth cert.pem key.pem --boat 5 --camera bow \
    --publish rtsp://127.0.0.1:8554/sn5/bow
```

The camera is stopped on the boat when the process exits, including on SIGINT
and SIGTERM, so a stream is not left running on a metered link.

Like the Dashboard, `seatrac.video` acknowledges each frame it reassembles. This
message is not in the interface document; see [docs/errata.md](docs/errata.md)
for it and for the other places the document and the boat disagree.

Changing resolution changes the picture size mid-stream, which a downstream RTSP
session cannot follow. Output is held back until a picture matching
`--resolution` arrives, and if the size later changes the process exits with
status 75 so a supervisor can start it again and renegotiate.

The Send Stream bitfield that selects which link carries the video is only
partly documented, and Certus has never been observed. `--probe` requests one
low resolution still image per candidate bit and reports which link delivered
it.

`seatrac.camera` exposes the pieces directly: `start_stream()`, `stop_stream()`,
`stop_all_streams()` and `image_ack()` build the messages, and `H264Reassembler`
turns received messages into whole frames along with the vessel attitude and
capture timestamp that accompany each one. `seatrac.h264` reads the picture size
out of a parameter set.


## ROS Node

The `seatrac` ROS node interfaces between the ROS graph and the SeaTrac vessel. It supports [ROS Noetic Ninjemys](https://wiki.ros.org/noetic). ROS 2 is not yet supported.


### Network communication

The node does not communicate directly with the boat; instead, the `~in` and `~out` topics carry [ds_core_msgs/RawData](https://bitbucket.org/whoidsl/ds_msgs/src/master/ds_core_msgs/msg/RawData.msg) messages.

An easy way to connect UDP traffic with these topics is to use the bridge_node from [ds_util_nodes](https://bitbucket.org/whoidsl/ds_base/src/master/ds_util_nodes/). An example is given below.

The UDP communications within the boat network can be configured using the SeaTrac Dashboard app. This is outside the scope of this documentation.


### Installation

Clone this repository into the `src/` directory of a ROS workspace. ROS will automatically discover the `seatrac` package in the directory hierarchy.


### Launch configuration

```xml
<launch>
    <rosparam command="load" file="seatrac.yaml" />

    <node name="seatrac" pkg="seatrac" type="seatrac_node.py">
        <!-- remap i/o to the bridge_node topics -->
        <remap from="~in" to="~comms/in" />
        <remap from="~out" to="~comms/out" />
    </node>

    <node ns="seatrac" name="comms" pkg="ds_util_nodes" type="bridge_node"/>
</launch>
```

``` yaml
seatrac:
  outlets:
    - name: "starlink"
      outlet: 15

  # Configuration for the bridge_node
  comms:
    connection:
      type: "UDP"
      udp_rx: 62001
      udp_tx: 62001
      udp_address: "10.1.20.88"
```


### Parameters

- `~outlets`: A list of objects with `name` and `outlet` (switch index, 1-based)

The `comm` parameters in the above example belong to bridge_node.


### Subscribed Topics

- `~in` (ds_core_msgs/RawData): Incoming SeaTrac packets.
- `~outlet/<name>/control` (std_msgs/Bool): Control the named outlet.


### Published Topics

- `~out` (ds_core_msgs/RawData): Outgoing SeaTrac packets.
- `~outlet/<name>/status` (seatrac/OutletStatus): On/off status of the named outlet.
- `~power` (seatrac/PowerLevel): Battery pack state of charge and current draw.
