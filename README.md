ha
==

Yet Another home automation project
Written for myself with no promises to anybody
Message format, protocol etc. are the subject to change

Message format for serial interface (mostly for RS485) was inspired by BSC and some other projects.
==
Start byte of every frame is STX (0x02)
Stop byte of every frame is ETX (0x03)
Every other byte (between STX and ETX) is splitted to two bytes - higher nibble goes to the first byte and lower nibble goes to the second.
After this the inverted values of the nibbles are placed near them.
0x12 -> 0x1_ 0x_2 -> 0x1E 0xD2
0x45 -> 0x4_ 0x_5 -> 0x4B 0xA5
0xFE -> 0xF_ 0x_E -> 0xF0 0x1E


Header Format:
==
To address (byte). 0xFF is a broadcast. This is the first field to let the receiver decide wheither it want to get the rest of packet or not.
From address (byte).
Message type (byte)
The rest of packet depends on the type.

Message types
==
0x00 - ping reply. When broadcasted - "Hello, World! I'm here!" May be used as an ACK (need some flags for this?), can send some health status (power voltage, uptime, etc.)
0x01 - ping request. May be used to get the node status (voltages, uptime, setup values, etc.).
0x02 - setup the node: GPIO masks for analog and digital inputs, outputs, "onchange" event(s?), counters, automatic measure and send intervals, etc.
0x03 - node information (sensors list, etc.)

0x10 - Request: start any long-counting measures (DS18B20, DHT22, etc.)
0x11 - Reply: a long-counting measures (DS18B20, DHT22, etc.) started (if the request was addressed, not broadcast).
0x12 - Request of the measured values. Should send the max measure age in seconds. If more time elapsed since the last measure, do it again in blocking mode. Zero means infinite? Or not?
0x13 - scan for 1-wire, DHT22 and other detectable sensors. If configured, check inputs, connected to a "common sensors output" (strobe).

0x15 - Data from sensors (in reply to 0x12 or by timer), respective to sensors list sent in message 0x03

0x20 - set outputs. Masks, IO port values.
0x30 - program some automation. In -> Out dependencies.

0x54 - test request - for debug

0xFF - NAK or any Error in received packet (CRC, unknown command, etc.)

TODO:
- Check if channel is free before transmit
- CRC
- Message number (magic)
- Strobe output pin - to go HIGH and LOW during some input tests.
- Flags field? Flags for "send self status" request/mark, link status (for radio - signal level, SNR, etc.)
