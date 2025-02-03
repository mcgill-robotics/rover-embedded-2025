#ifndef ROS_JAM_H
#define ROS_JAM_H

#include <Arduino.h>

class RosJam {
public:
    /**
     * @brief Initializes the serial communication for ROS-JAM.
     * @param baud_rate The baud rate for serial communication (default: 9600).
     */
    static void begin(unsigned long baud_rate = 9600);

    /**
     * @brief Publishes a message to a ROS2 topic.
     * @param topic The name of the ROS2 topic.
     * @param message The message to be sent.
     * @return True if the message was successfully sent, false otherwise.
     * 
     * Message format: "<topic>:<message>\n"
     * Example: "sensor_data:25.3\n"
     */
    static bool publish(String topic, String message);

    /**
     * @brief Subscribes to a ROS2 topic.
     * @param topic The name of the topic to subscribe to.
     */
    static void subscribe(String topic);

    /**
     * @brief Checks if a subscribed topic has received a new message.
     * @param topic The name of the topic.
     * @return The latest message if updated, or an empty string if no update is available.
     */
    static String getMessage(String topic);

    /**
     * @brief Handles incoming serial messages.
     * This function should be called regularly inside the `loop()` function.
     * 
     * Expected message format from ROS2: "<topic>:<message>\n"
     * Example: "cmd_vel:{"x":1.0,"y":0.0}\n"
     */
    static void spin();
};

#endif // ROS_JAM_H
