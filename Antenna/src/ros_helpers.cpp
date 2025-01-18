
#include <ros.h>
#include <std_msgs/String.h>
#include "ros_helpers.h"

extern ros::NodeHandle nh;

std_msgs::String debug_msg;
ros::Publisher debug_pub("debug_topic", &debug_msg);
char message_buffer[256];

// ros_printf function
void ros_printf(const char *format, ...)
{
    // Initialize the variable argument list
    va_list args;
    va_start(args, format);

    // Format the message and store it in message_buffer
    vsnprintf(message_buffer, sizeof(message_buffer), format, args);

    // End the variable argument list
    va_end(args);

    // Assign the formatted message to debug_msg.data
    debug_msg.data = message_buffer;

    // Publish the debug message
    debug_pub.publish(&debug_msg);

    // Spin once to handle callbacks
    nh.spinOnce();
}
