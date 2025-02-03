#include <Arduino.h>
#include "ros_jam.h"

void setup() {
    Serial.begin(9600);
    RosJam::begin();
    
    // Subscribe to necessary topics (Test 3.1, Test 3.2, Test 4.1, Test 5.1, Test 5.2, Test 5.3, Test 5.4, Test 6.1, Test 6.2, Test 6.3)
    RosJam::subscribe("test_topic");
    RosJam::subscribe("control");
    
    // Indicate that the system is ready (Test 1.1)
    Serial.println("READY");
    
    // Publish initial test messages (Test 2.1, Test 2.2, Test 2.3)
    RosJam::publish("test_topic", "OK");
    RosJam::publish("test_topic", "23.5");
    RosJam::publish("test_topic", "{\"x\":1.0,\"y\":0.0}");
}

void loop() {
    // Process incoming messages (Supports all tests involving message reception)
    RosJam::spin();

    // Check for updates on "test_topic" (Test 5.1, Test 5.3, Test 6.1, Test 6.2, Test 6.3)
    String testMessage = RosJam::getMessage("test_topic");
    if (testMessage != "") {
        Serial.println("test_topic:" + testMessage);
    }
    
    // Handle control messages and respond accordingly (Test 3.1, Test 3.2)
    String controlMessage = RosJam::getMessage("control");
    if (controlMessage == "ON") {
        RosJam::publish("test_topic", "ACTIVATED"); // Test 3.1
    } else if (controlMessage == "HIGH") {
        RosJam::publish("test_topic", "RUNNING"); // Test 3.2
    } else if (controlMessage == "LOW") {
        RosJam::publish("test_topic", "IDLE"); // Test 3.2
    }
}