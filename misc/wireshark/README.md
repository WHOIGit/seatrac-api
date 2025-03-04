# SeaTrac Dissector for Wireshark

This repository contains a Wireshark dissector for the SeaTrac protocol.

To install the dissector, copy the seatrac.lua script into the Wireshark plugins directory. The location of the plugins directory can be found by opening Wireshark and navigating to About Wireshark → Folders. Load the plugin using Analyze → Reload Lua Plugins.


### Plaintext UDP

Wireshark can capture and dissect plaintext UDP packets transmitted within the boat network. The default port number is 62001.


### Encrypted UDP

The Dashboard uses an TLS connection to the SeaTrac server which relays commands to the boat. Wireshark can be [configured to decrypt TLS](https://wiki.wireshark.org/TLS#tls-decryption) if the client private key is known.


### RF Link

The RF Link is based on an XBee radio module. The Dashboard app communicates with the module over serial. Captured the serial traffic with another tool and bring it into Wireshark (see [rgov/serial2pcap](https://github.com/rgov/serial2pcap)). Use the [rgov/xbee-api-dissector](https://github.com/rgov/xbee-api-dissector) dissector plugin to parse the XBee traffic, then Decode As to decode the XBee payload as the SeaTrac protocol.
