#include <ros.h>
#include <std_msgs/String.h>

extern std_msgs::String debug_msg;
extern ros::Publisher debug_pub;
extern char message_buffer[256];

void ros_printf(const char *format, ...);