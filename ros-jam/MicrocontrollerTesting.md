# Implementation & Testing Guide for ROS-JAM Microcontroller

## Overview
This document provides step-by-step instructions for implementing and testing the `ros-jam` microcontroller package. The repository follows a structured layout to separate implementation, documentation, and testing. Follow the guidelines below to ensure smooth development and validation.

## Repository Structure
```
├── README.md                         # General project documentation
├── doc
│   └── Requirements.md               # System requirements and specifications
├── ros-jam-microcontroller           # Main microcontroller implementation
│   ├── include
│   │   └── README                    # Placeholder for header files
│   ├── lib
│   │   ├── README                    # Library documentation
│   │   └── ros_jam                    
│   │       ├── ros_jam.cpp           # Core implementation file (TO BE IMPLEMENTED)
│   │       └── ros_jam.h             # Header file (ALREADY DEFINED)
│   ├── platformio.ini                 # PlatformIO configuration file
│   ├── src
│   │   └── main.cpp                   # Microcontroller firmware entry point (READY)
│   └── test
│       └── README                     # Microcontroller test documentation
└── test
    ├── microcontroller_tester.py      # Python script for automated testing
    └── microcontroller_tests.md       # List of test cases
```

## Step 1: Implement `ros_jam.cpp`
### Location: `ros-jam-microcontroller/lib/ros_jam/ros_jam.cpp`
- The header file (`ros_jam.h`) is already defined.
- Implement each function according to its declaration.
- Follow the message format `<topic>:<message>\n` for communication.
- Ensure `begin()`, `publish()`, `subscribe()`, `getMessage()`, and `spin()` work as expected.

## Step 2: Compile & Upload to Microcontroller
### Using PlatformIO
1. Navigate to the `ros-jam-microcontroller/` directory.
2. Ensure PlatformIO is installed (`pip install platformio`).
3. Run the following command to compile and upload:
   ```sh
   pio run --target upload
   ```
4. Open the serial monitor to check logs:
   ```sh
   pio device monitor
   ```

## Step 3: Run Automated Tests
### Using `microcontroller_tester.py`
1. Ensure the microcontroller is connected via USB.
2. Navigate to the `test/` directory.
3. Run the test script:
   ```sh
   python microcontroller_tester.py
   ```
4. Observe test results. Tests will validate:
   - Serial communication startup (`READY` message)
   - Publishing messages
   - Subscribing & responding to updates
   - Ignoring unsubscribed topics
   - Message formatting & parsing
   - Edge cases (e.g., empty or malformed messages)

## Step 4: Debugging & Fixing Issues
- If tests fail, check the serial monitor for logs.
- Review the implementation against `microcontroller_tests.md`.
- Debug in `ros_jam.cpp`, recompile, and retest.

## Notes
- `main.cpp` is already structured to interact with `ros_jam.h`.
- The testing framework ensures correctness before ROS2 integration.
- Follow the provided test cases to validate each implementation step.

## Final Steps
1. Implement `ros_jam.cpp` functionally.
2. Run tests to verify expected behavior.
3. Ensure all tests pass before proceeding to ROS2 integration.
4. Document any changes made during implementation.

For further assistance, refer to `microcontroller_tests.md` or contact the repository maintainer.

