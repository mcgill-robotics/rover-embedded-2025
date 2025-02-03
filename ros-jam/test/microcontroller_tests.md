# ROS-JAM Testing Suite

## Overview
This document outlines the test suite for verifying the functionality of the `ros-jam` package on a microcontroller. The microcontroller runs `main.cpp`, subscribes to specific topics, and responds accordingly. A Python script on a connected laptop sends messages and verifies expected responses.

To optimize testing, the number of unique topic names has been minimized while maintaining test coverage. The following topics are used throughout the suite:

- **`test_topic`** – Used for general publishing, parsing, and edge cases.
- **`control`** – Used for subscription-based responses.
- **`status`** – Used for error handling and ignored messages.

## 1. Basic Initialization
### Test 1.1: Serial Communication Starts Properly
- **Microcontroller Action**: Send `"READY\n"` upon startup.
- **Python Test**: Verify that `"READY\n"` is received.

## 2. Publishing Messages
### Test 2.1: Publishing a Simple Message
### Test 2.2: Publishing Numeric Data
### Test 2.3: Publishing JSON Data
- **Microcontroller Action**:
  ```cpp
  RosJam.publish("test_topic", "OK");
  RosJam.publish("test_topic", "23.5");
  RosJam.publish("test_topic", "{\"x\":1.0,\"y\":0.0}");
  ```
- **Python Test**:
  - Expect:
    ```
    test_topic:OK\n
    test_topic:23.5\n
    test_topic:{"x":1.0,"y":0.0}\n
    ```

## 3. Subscribing & Receiving Messages
### Test 3.1: Microcontroller Responds to a Subscribed Topic
### Test 3.2: Multiple Subscribed Topics
- **Microcontroller Setup**:
  ```cpp
  RosJam.subscribe("control");
  ```
- **Microcontroller Behavior**:
  - If it receives `"control:ON"`, it responds with `"test_topic:ACTIVATED"`.
  - If it receives `"control:HIGH"`, it responds with `"test_topic:RUNNING"`.
  - If it receives `"control:LOW"`, it responds with `"test_topic:IDLE"`.
- **Python Test**:
  - Send `"control:ON\n"`, expect `"test_topic:ACTIVATED\n"`.
  - Send `"control:HIGH\n"`, expect `"test_topic:RUNNING\n"`.
  - Send `"control:LOW\n"`, expect `"test_topic:IDLE\n"`.

## 4. Ignoring Unsubscribed Topics
### Test 4.1: Microcontroller Ignores Unsubscribed Topics
- **Microcontroller Setup**:
  ```cpp
  RosJam.subscribe("control"); // Does NOT subscribe to "status"
  ```
- **Python Test**:
  - Send `"status:DATA\n"` and verify **no response**.

## 5. Message Formatting & Parsing
### Test 5.1: Correct Parsing of Topic and Message
### Test 5.2: Handling Messages Without Newline (`\n`)
### Test 5.3: Handling Extra Spaces in Messages
### Test 5.4: Handling Messages Without Topic Separator (`:`)
- **Microcontroller Setup**:
  ```cpp
  RosJam.subscribe("test_topic");
  ```
- **Python Test**:
  - Send `"test_topic:25.6\n"`, verify `getMessage("test_topic") == "25.6"`.
  - Send `"test_topic:MOVE"` (no newline), expect **no update**.
  - Send `" test_topic : OK "` (extra spaces), expect `"OK"`.
  - Send `"MALFORMED_MESSAGE\n"`, expect **no response**.

## 6. Handling Edge Cases
### Test 6.1: Empty Messages
### Test 6.2: Long Messages
### Test 6.3: High-Frequency Messages
- **Microcontroller Setup**:
  ```cpp
  RosJam.subscribe("test_topic");
  ```
- **Python Test**:
  - Send `"\n"` (empty), expect **no response**.
  - Send `"test_topic:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n"` (long message), ensure **no crash**.
  - Send multiple messages in rapid succession:
    ```
    test_topic:20\n
    test_topic:21\n
    test_topic:22\n
    ```
    - Ensure **all messages are processed correctly**.

## Summary of Optimization
- Reduced unique topics from multiple individual names to **three** (`test_topic`, `control`, `status`).
- Maintained **all original test cases**.
- Ensured the **entire suite runs in one execution cycle**.

This document serves as a reference for verifying `ros-jam`'s functionality on a microcontroller before integrating with ROS2.

