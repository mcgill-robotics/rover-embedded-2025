#include <Arduino.h>
#include "ros_jam.h"

// Initialize serial communication
void RosJam::begin(unsigned long baud_rate) {
    // To be implemented
}

// Publish a message to a ROS2 topic
bool RosJam::publish(String topic, String message) {
    
    //Check for non-empty inputs
    if (topic.length() == 0 || message.length() == 0) {
        return false; 
    }

    //Match message format according to "<topic>:<message>\n"
    String formattedMessage = topic + ":" + message + "\n";

    //If the Serial isn't available return false
    if (!Serial) {
        return false; 
    }

    //Serial.print() will return the number of bytes successfully written
    size_t bytesSent = Serial.print(formattedMessage);

    //Compare bytesSent with the original message
    return (bytesSent == formattedMessage.length());


    return false;
}

// Subscribe to a ROS2 topic
void RosJam::subscribe(String topic) {
    // To be implemented
}

// Retrieve the latest message from a subscribed topic
String RosJam::getMessage(String topic) {
    // To be implemented
    return "";
}

// Process incoming serial messages
void RosJam::spin() {
    
    //Check the num of bytes waiting in the serial buffer (if data waiting)
    if (Serial.available() > 0) {

        //Read until newline
        String receivedMessage = Serial.readStringUntil('\n');
        receivedMessage.trim();

        int separatorIndex = receivedMessage.indexOf(':');

        //If ':' was not found then format is wrong
        if (separatorIndex == -1) {
            Serial.println("ERROR: Invalid message format");
            return;
        }

        //Everything before the separator index
        String topic = receivedMessage.substring(0, separatorIndex);

        //Everything after the separator index
        String message = receivedMessage.substring(separatorIndex + 1);

        topic.trim();
        message.trim();

        // Directly handle different topics here
        if (topic == "test_topic") {
    Serial.println("test_topic:" + message);  // Echo the message back
    } else if (topic == "control") {
    if (message == "ON") {
        Serial.println("test_topic:ACTIVATED");
    } else if (message == "HIGH") {
        Serial.println("test_topic:RUNNING");
    } else if (message == "LOW") {
        Serial.println("test_topic:IDLE");
    } else {
        Serial.println("ERROR: Unknown control command");
    }
    } else {
    Serial.println("ERROR: Unknown topic");
    }
    }
}
