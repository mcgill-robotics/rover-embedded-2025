# ROSJam V2

A communication protocol between microcontrollers and on-board computers for McGill Robotics' rover.

## Table of Contents

- [ROSJam V2](#rosjam-v2)
	- [Table of Contents](#table-of-contents)
	- [Usage](#usage)
		- [Adding to your build](#adding-to-your-build)
		- [Using it in your code](#using-it-in-your-code)
	- [Project Structure](#project-structure)
	- [Features](#features)
	- [To-do](#to-do)
	- [Tooling](#tooling)
	- [Protocol and implementation details](#protocol-and-implementation-details)

## Usage

### Adding to your build
Ddd `src` as a subdirectory in a CMake project using `add_subdirectory` and add `rosjam2` as a library with `target_link_libraries`.

If you are manually adding this library to some other build system, make sure all the files in `src`, `mpack.c` and `mpack.h` from [mpack]() and all the required files from [tinyusb]() are available.

### Using it in your code

[See the embedded API documentation](./docs/embedded.md)


## Project Structure

This folder contains the embedded implementation of ROSJamV2, tooling to test to the protocol implementation, tooling to debug communication from custom boards and bridges to relay data from embedded targets to the on-board computers.

The project uses CMake to build the library and example program.

- `src`: Contains the ROSJamV2 source code for the library.
- `python-bench`: Python scripts to connect to a Virtual COM port and test the library
- `rust-bench`: Rust based tools to perform the same tasks as the Python versions.
- `libs`: Contains third-party libraries used by the library. Uses git submodules when possible. (Libraries written by McGill Robotics are linked using relative paths as they live in the same repo)
- `setup-deps.bat`: Script to manually fetch dependencies on Windows when not using Cmake
- `setup-deps.sh`: Script to manually fetch dependencies on Linux/MacOS (MacOS is not tested but should work) when not using CMake
- `Core`, `Drivers`: Code for the example/test program loaded on a STM32G4
- `cmake`: Contains mostly cmake files for stm32cubemx but also contains additional `CMakeLists.txt` for third party libraries that do not use CMake
  
## Features
Optional features are features that not all clients or endpoints will need to implement

**Protocol features:**
- [x] Basic COBS + messagepack data transmission
- [ ] Remove MessagePack from message format keep only for payload
  - [ ] Implement VarInt based TLV with ASCII tags based format

- [ ] High priority messages?
- [ ] Diagnostic messages
  - [ ] Ping (with timestamp passthrough for latency measurements)
  - [ ] Packet ID on diagnostic packets
  - [ ] (Optional) Heartbeat
  - [ ] Board Info (HW/FW version)
  - [ ] Protocol version
  - [ ] 
**Embedded Implementation**

- [ ] Implement a stream compatible COBS API to reduce copying
- [ ] Implement a zero-copy parser for the new custom message format (copy only needed for the message pack payload)
- [ ] (Optional) Add per endpoint receive callbacks

**Other**

- [ ] Bridge to software control

## To-do
- [ ] On deserialize be more lenient to allow extra fields to extend protocol in the future.

## Tooling
- [ ] Bridge script/program between embedded targets and software division's code
- [x] CLI Latency tool (python and rust versions)
- [x] CLI receiver (python and rust versions)
- [x] CLI flood sender (python and rust versions)
- [ ] Web based dashboard? 
- [ ] TUI based dashboard?

## Protocol and implementation details

[See the protocol docs](./docs/protocol.md)

