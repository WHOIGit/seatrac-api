# SeaTrac Dissector for Wireshark

This repository contains a Wireshark dissector for the SeaTrac protocol.

To install the dissector, copy the seatrac.lua script into the Wireshark plugins directory. The location of the plugins directory can be found by opening Wireshark and navigating to About Wireshark → Folders. Load the plugin using Analyze → Reload Lua Plugins.


## Plaintext UDP

Wireshark can capture and dissect plaintext UDP packets transmitted within the boat network. The default port number is 62001.


## RF Link

The RF Link is based on an XBee radio module. The Dashboard app communicates with the module over serial. Captured the serial traffic with another tool and bring it into Wireshark (see [rgov/serial2pcap](https://github.com/rgov/serial2pcap)). Use the [rgov/xbee-api-dissector](https://github.com/rgov/xbee-api-dissector) dissector plugin to parse the XBee traffic, then Decode As to decode the XBee payload as the SeaTrac protocol.


## Encrypted Dashboard Traffic

The Dashboard uses an TLS connection to the SeaTrac server which relays commands to the boat. Wireshark can be decrypt the TLS if we capture the encryption secrets from the Dashboard app. Unfortunately, the process is a little involved, because

1. The connection is secured by a Diffie-Hellman key exchange, so the client private key alone is insufficient to decrypt the traffic.

2. The TLS library used by the Dashboard at the time of writing does not support the `SSLKEYLOGFILE` environment variable to export the encryption secrets.

First, we need Python; if your system does not have it, install with the following command under an administrator command prompt:

    winget install python3 --architecture x64 --scope machine

If you are on Windows for ARM, you must also delete `C:\Program Files\Python3xx\vcruntime_140.dll` files and separately install the x86_64 Visual C++ runtime:

    winget install Microsoft.VCRedist.2015+.x64

 Close the command prompt window and open a new one. Proceed to installing [Frida](https://frida.re):

    python -m ensurepip
    python -m pip install frida-tools

Open Wireshark. Under Edit → Preferences → Protocols → TLS, set the (Pre)-Master-Secret log filename to `C:\dashboard\sslkeys.log`. Now, when the Dashboard app is run with Frida, Wireshark can use the logged secrets to decrypt the TLS traffic.

> [!TIP]
> The `tls/example.pcap` and `tls/example-sslkeys.log` sample files can be used to test the decryption process.

After starting a capture in Wireshark, run the `snoop.bat` script to start the Dashboard app with encryption secrets logged to `C:\dashboard\sslkeys.log`.

Successful decryption is recognizable by a "Data (xx bytes)" subtree appearing in the lower left Packet Details pane of an "Application Data" packet. Decode this as a SeaTrac message by right-clicking the subtree and selecting Decode As and choosing the SeaTrac dissector.

To save a capture along with key material necessary to decrypt it, use Edit → Inject TLS Secrets, then save as a `.pcapng` file.
