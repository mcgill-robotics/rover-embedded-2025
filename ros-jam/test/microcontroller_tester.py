import serial
import time

class RosJamTester:
    def __init__(self, port, baud_rate=9600, timeout=2):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        time.sleep(2)  # Allow time for the serial connection to initialize

    def send_message(self, message):
        """Sends a message to the microcontroller."""
        self.ser.write((message + '\n').encode('utf-8'))

    def read_response(self):
        """Reads a response from the microcontroller."""
        return self.ser.readline().decode('utf-8').strip()

    # Test 1.1: Basic Initialization
    def test_basic_initialization(self):
        response = self.read_response()
        return response == "READY"

    # Test 2: Publishing Messages
    def test_publish_receive(self, topic, message):
        self.send_message(f"{topic}:{message}")
        response = self.read_response()
        return response == f"{topic}:{message}"

    # Test 3: Subscribing & Receiving Messages
    def test_subscribe_response(self, topic, input_message, expected_response):
        self.send_message(f"{topic}:{input_message}")
        response = self.read_response()
        return response == expected_response
    
    # Test 4: Ignoring Unsubscribed Topics
    def test_ignore_unsubscribed(self, topic, message):
        self.send_message(f"{topic}:{message}")
        response = self.read_response()
        return response == ""

    # Test 5: Message Formatting & Parsing
    def test_message_formatting(self, input_message, expected_response):
        self.send_message(input_message)
        response = self.read_response()
        return response == expected_response

    # Test 6: Handling Edge Cases
    def test_edge_cases(self, input_message, expected_response):
        self.send_message(input_message)
        response = self.read_response()
        return response == expected_response

    def close(self):
        self.ser.close()

if __name__ == "__main__":
    tester = RosJamTester(port="COM3")  # Adjust port as needed
    
    # Run Test 1.1: Basic Initialization
    print("Test 1.1 - Initialization Test:", "Passed" if tester.test_basic_initialization() else "Failed")
    
    # Run Test 2: Publishing Messages
    print("Test 2.1 - Publish Simple Message:", "Passed" if tester.test_publish_receive("test_topic", "OK") else "Failed")
    print("Test 2.2 - Publish Numeric Message:", "Passed" if tester.test_publish_receive("test_topic", "23.5") else "Failed")
    print("Test 2.3 - Publish JSON Message:", "Passed" if tester.test_publish_receive("test_topic", "{\"x\":1.0,\"y\":0.0}") else "Failed")
    
    # Run Test 3: Subscribing & Receiving Messages
    print("Test 3.1 - Subscribe Response ON:", "Passed" if tester.test_subscribe_response("control", "ON", "test_topic:ACTIVATED") else "Failed")
    print("Test 3.2 - Subscribe Response HIGH:", "Passed" if tester.test_subscribe_response("control", "HIGH", "test_topic:RUNNING") else "Failed")
    print("Test 3.2 - Subscribe Response LOW:", "Passed" if tester.test_subscribe_response("control", "LOW", "test_topic:IDLE") else "Failed")
    
    # Run Test 4: Ignoring Unsubscribed Topics
    print("Test 4.1 - Ignore Unsubscribed Topic:", "Passed" if tester.test_ignore_unsubscribed("status", "DATA") else "Failed")
    
    # Run Test 5: Message Formatting & Parsing
    print("Test 5.1 - Message Parsing:", "Passed" if tester.test_message_formatting("test_topic:25.6", "test_topic:25.6") else "Failed")
    print("Test 5.2 - Message Without Newline:", "Passed" if tester.test_message_formatting("test_topic:MOVE", "") else "Failed")
    print("Test 5.3 - Message With Extra Spaces:", "Passed" if tester.test_message_formatting(" test_topic : OK ", "test_topic:OK") else "Failed")
    print("Test 5.4 - Malformed Message:", "Passed" if tester.test_message_formatting("MALFORMED_MESSAGE", "") else "Failed")
    
    # Run Test 6: Handling Edge Cases
    print("Test 6.1 - Empty Message:", "Passed" if tester.test_edge_cases("", "") else "Failed")
    print("Test 6.2 - Long Message:", "Passed" if tester.test_edge_cases("test_topic:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx", "test_topic:xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx") else "Failed")
    print("Test 6.3 - High-Frequency Message:", "Passed" if tester.test_edge_cases("test_topic:20", "test_topic:20") and \
          tester.test_edge_cases("test_topic:21", "test_topic:21") and \
          tester.test_edge_cases("test_topic:22", "test_topic:22") else "Failed")
    
    tester.close()
