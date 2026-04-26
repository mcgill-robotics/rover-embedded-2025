# About ROSJam2

## Goals

The protocol is designed to be a reliable and relatively fast on embedded platorms while remaining flexible to be easily updated and extended. Other goals include being human partially human legible to help with debugging. 

Some applications of the protocol are to provide a common diagnostic interface to custom boards over USB, enabling multiplexing data through single physical links and providing some basic addressing of subsystems on embedded targets through a message format.

## Protocol Description

All data is COBS (Consistent Overhead Byte Stuffing) encoded to ensure reliable framing of messages while allowing arbitrary binary data to be sent as part of a message. `'\0'` is used as a delimiter that is only emitted at the end of a message.

> To note: Because the delimiter is only sent at the end of a message, if a delimiter is not sent before the first message, the message will always be dropped as there is no way to reliably detect the start and decode the COBS encoded data.

### Message format

ROSJam messages are based on MessagePack (a binary JSON like format).

Each message consists of a MessagePack map of the following form

```
+-----------------------------------------+
|                 Message                 |
+-----------------------------------------+
| "topic" | topic_name | "data" | payload |
+-----------------------------------------+
```

`"Topic"` is the key for `topic_name` and `"data"` is the key for `custom_data`

Keyvalue pairs do not have to be ordered so this is also valid
```
+-----------------------------------------+
|                 Message                 |
+-----------------------------------------+
| "data" | payload | "topic" | topic_name |
+-----------------------------------------+
```


### Message field description

**Keys:**

- `"topic"` is a human readable key associated to the value representing the topic. This key is encoded as a MessagePack String.
- `"data"` is a human readable key associated to the value associated to the topic depended custom data. This key is also encoded as MessagePack String.

**Values:**

- `"topic_name"` is a MessagePack String representing a topic. A topic represents some function/system to which the data is to be routed to.
- `"payload"` MessagePack binary data representing a payload for the subsystem/application associated with the message's topic. More details in the next section.

### Payload Format

The payload as explained previously is encoded as MessagePack binary data. The contents however must be another MessagePack object. This was chosen for ease of implementation with the mpack library on embedded targets as it would allow decoupling the payload serialization/deserialization from the one for the full message. 

Other than being some MessagePack object wrapped within a MessagePack binary data field, the payload format is defined by the user of a specific topic.

## Implementation details

This implementation is written in C and has been tested on an STM32G4 series microcontroller. 

Currently, the implementation is focused on supporting communication through USB CDC Virtual COM port. In theory this could be extended to other serial style communication interfaces like UART.

Other microcontrollers could also be supported as the USB code is built on top of [tinyusb](). The only STM32 specific code is a function to provide a serial number as part of one of the USB descriptors.

MessagePack serialization and deserialization uses [mpack]() as it is a fast C implementation that is well suited for many platforms including embedded platforms. For our usage, it is used exclusively with statically allocated buffers.

The COBS implementation is a custom implementation that can be also found in the McGill Robotics [rover-embedded-2025]() repository.