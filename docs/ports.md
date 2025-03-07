The SeaTrac server is at 3.213.3.223.

The server relays messages to/from the boat and Dashboard application. The boat connects outbound to the server.

There are many ports used depending on the link type and whether the connection is read-only ("listen") or read-write ("pilot"). The base port numbers are below; the boat number (SN8 = 7, etc.) is added to the base port number.

| Link       | Read-Only | Read-Write |
|------------|-----------|------------|
| Cell1      | 44000     | 41000      |
| SBD        | 45000     | 42000      |
| Cell2      | 45100     | 42100      |
| Certus     | 45200     | 42200      |
| Starlink   | 45300     | 42300      |
| PCCell2    | 46000     | 43000      |
| PCCertus   | 49000     | 48000      |
| PCStarlink | 49100     | 48100      |
