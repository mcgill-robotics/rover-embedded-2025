# **ROS-JAM Requirements**

## **1. Functional Requirements**
### **1.1 Microcontroller Communication**
- The microcontroller must communicate with the ROS2 script using USB serial communication.
- The microcontroller must send and receive structured messages through the serial interface.
- The message format must be defined to ensure consistent parsing on both ends.

### **1.2 ROS2 Python Script**
- The script must establish a serial connection with the microcontroller upon startup.
- The script must parse incoming serial messages and convert them into ROS2 messages.
- The script must subscribe to relevant ROS2 topics and relay messages to the microcontroller via serial communication.
- The script must handle message queuing to prevent loss of communication due to high-frequency updates.

### **1.3 Error Handling and Recovery**
- The system must handle serial connection failures and attempt automatic reconnection.
- The script must detect and log malformed messages, ignoring or requesting retransmission if necessary.
- The system must implement timeouts to avoid blocking operations on serial communication failures.

## **2. Non-Functional Requirements**
### **2.1 Performance**
- The communication latency between the microcontroller and the ROS2 script should remain below a defined threshold (e.g., 10ms for real-time applications).
- The system must be able to handle a sustained message rate without significant degradation in performance.

### **2.2 Reliability**
- The system should recover gracefully from transient errors without requiring a restart.
- Messages should be delivered with minimal packet loss.

### **2.3 Portability**
- The ROS2 script must be containerized using Docker to ensure deployment consistency.
- The script must support running on multiple hardware architectures (e.g., x86, ARM).
