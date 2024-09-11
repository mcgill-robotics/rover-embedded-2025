#include "common.h"
#if USE_ROS_FIRMWARE == 1
#include <Arduino.h>
#include <hardware_pins.h>
#include <driver_motor.h>
#include <model_encoder.h>
#include <model_sensor.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

#include <memory>

// CONFIGURATION
#define PID_PERIOD_MS 100
#define PID_PERIOD_US PID_PERIOD_MS * 1000
#define HWSERIAL Serial4
#define DEBUG_PRINT 1

#define OUTA_Pin ENCPIN1_1
#define OUTB_Pin ENCPIN1_2

#define LED_PIN 13

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// ROS
rcl_publisher_t arm_brushed_fb_pub;
rcl_subscription_t arm_brushed_cmd_sub;
rcl_publisher_t debug_pub;
std_msgs__msg__Float32MultiArray arm_brushed_fb_msg;
std_msgs__msg__Float32MultiArray input_msg;
std_msgs__msg__String debug_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float arm_brushed_setpoint_ps[3] = {0, 0, 0};
float arm_brushed_angle_ps[3] = {0, 0, 0};
void brushed_board_ros_loop();

void arm_brushed_cmd_cb(const void *msgin);

char message_buffer[256];

// DUMB POINTERS
driver_motor mot1;
driver_motor mot2;
driver_motor mot3;

// TESTER VARIABLES
int stage = 0;
int pos = 90;
int tolerance = 0.5;
volatile boolean is_homed = false;

// FUNCTION DECLARATIONS
void brushed_board_homing();
void brushed_board_ros_loop();

void lim1ISR();
void lim2ISR();
void lim3ISR();
void lim4ISR();
void lim5ISR();
void lim6ISR();

// Define the size of the moving average window
const uint32_t MOVING_AVERAGE_SIZE = 10;
float cur1_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
float cur2_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
float cur3_voltage_buffer[MOVING_AVERAGE_SIZE] = {0};
int cur1_voltage_buffer_idx = 0;
int cur2_voltage_buffer_idx = 0;
int cur3_voltage_buffer_idx = 0;

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

float moving_average(float new_reading, float *buffer, int buffer_size, int *buffer_idx)
{
  float sum = 0;
  float oldest_reading = buffer[*buffer_idx];

  // Add the new reading to the buffer.
  buffer[*buffer_idx] = new_reading;
  *buffer_idx = (*buffer_idx + 1) % buffer_size; // Circular buffer

  // Sum all readings.
  for (int i = 0; i < buffer_size; i++)
  {
    sum += buffer[i];
  }
  sum -= oldest_reading;

  return sum / buffer_size;
}

// Buffer for formatted messages

// ros_printf function
static void ros_printf(const char *format, ...)
{
  // Initialize the variable argument list
  va_list args;
  va_start(args, format);

  // Format the message and store it in message_buffer
  vsnprintf(message_buffer, sizeof(message_buffer), format, args);

  // End the variable argument list
  va_end(args);

  // Assign the formatted message to debug_msg.data using rosidl_runtime_c__String__assign
  if (!rosidl_runtime_c__String__assign(&debug_msg.data, message_buffer))
  {
    // Handle assignment error if needed
    return;
  }

  // Publish the debug message
  RCSOFTCHECK(rcl_publish(&debug_pub, &debug_msg, NULL));

  // Spin once to handle callbacks
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// Ensure counter is initialized
int counter = 0;

// LIMIT SWITCHES
IntervalTimer lim_watchdog_timer;
const unsigned long DEBOUNCE_DELAY_MS = 50;

volatile bool lim1_state = false;
volatile bool lim2_state = false;
volatile bool lim3_state = false;
volatile bool lim4_state = false;
volatile bool lim5_state = false;
volatile bool lim6_state = false;

uint32_t loop_last_time = 0;

void setup()
{
  HWSERIAL.begin(115200);
  while (!HWSERIAL)
    ;
  HWSERIAL.println("Serial port initialized");

  // Setup Microros
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &arm_brushed_fb_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/armBrushedFb"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &debug_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "debug_topic_brushed"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &arm_brushed_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/armBrushedCmd"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_brushed_cmd_sub, &input_msg, &arm_brushed_cmd_cb, ON_NEW_DATA));

  std::unique_ptr<model_encoder> enc1 = std::make_unique<model_encoder>();
  std::unique_ptr<model_encoder> enc2 = std::make_unique<model_encoder>();
  std::unique_ptr<model_encoder> enc3 = std::make_unique<model_encoder>();

  std::unique_ptr<model_sensor> cur1 = std::make_unique<model_sensor>();
  std::unique_ptr<model_sensor> cur2 = std::make_unique<model_sensor>();
  std::unique_ptr<model_sensor> cur3 = std::make_unique<model_sensor>();

  // Initialize encoders
  // 43000 clicks for wrist pitch, 32580 for wrist roll
  // Only using 1 & 2 becase 1 & 3 conflicts
  // Could be 32768 since it's a power of 2
  // enc1->initialize_encoder(0, 0, 32580, 1);
  // new small servo estimate for resolution
  enc1->initialize_encoder(0, 0, 43000, 1);
  enc2->initialize_encoder(0, 0, 32580, 2);
  // enc3->initialize_encoder(0, 0, 43000, 3);

  // Initialize current sensors
  cur1->initialize_sensor(CURRENT_SENSE_A);
  cur3->initialize_sensor(CURRENT_SENSE_C);
  cur2->initialize_sensor(CURRENT_SENSE_B);

  // Initialize motors
  mot1.attach_encoder(std::move(enc1));
  mot2.attach_encoder(std::move(enc2));
  // mot3.attach_encoder(std::move(enc3));

  mot1.attach_current_sensor(std::move(cur1));
  mot2.attach_current_sensor(std::move(cur2));
  mot3.attach_current_sensor(std::move(cur3));

  // Motor init, forward logic is 1
  mot1.initialize_motor(1, PWMPIN1, DIRPIN1, nSLEEP1, 5.0, 0.0);
  mot2.initialize_motor(1, PWMPIN2, DIRPIN2, nSLEEP2, 5.0, 0.0);
  mot3.initialize_motor(-1, PWMPIN3, DIRPIN3, nSLEEP3, 5.0, 0.0);

  // Set motor configuration after initialization
  // Gear ratio needs to be set before angle limits so limits are scaled
  mot1.set_gear_ratio(2.0);
  mot1.set_angle_limit_ps(wrist_pitch_max_angle, wrist_pitch_min_angle);
  mot1._is_circular_joint = false;
  mot2.set_gear_ratio(2.0);
  mot2._is_circular_joint = true;

  // Limit Switches
  pinMode(LIM_1, INPUT);
  pinMode(LIM_2, INPUT);
  pinMode(LIM_3, INPUT);
  pinMode(LIM_4, INPUT);
  pinMode(LIM_5, INPUT);
  pinMode(LIM_6, INPUT);
  attachInterrupt(digitalPinToInterrupt(LIM_1), lim1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIM_2), lim2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIM_3), lim3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIM_4), lim4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIM_5), lim5ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIM_6), lim6ISR, CHANGE);

  // Watchdog timer to prevent bouncing or stuck limit switches
  lim_watchdog_timer.begin([]()
                           {
        lim1_state = digitalRead(LIM_1);
        lim2_state = digitalRead(LIM_2);
        lim3_state = digitalRead(LIM_3);
        lim4_state = digitalRead(LIM_4);
        lim5_state = digitalRead(LIM_5);
        lim6_state = digitalRead(LIM_6); },
                           DEBOUNCE_DELAY_MS * 1000);

  // Initialize Motor
  // nSLEEP is actually hardwired to 5V so not really necessary
  digitalWrite(nSLEEP1, HIGH);
  digitalWrite(nSLEEP2, HIGH);
  digitalWrite(nSLEEP3, HIGH);

  // HOMING, only linear joints will be homed
  // brushed_board_homing();
  // while (!is_homed)
  //     ;

  loop_last_time = micros();
}

void loop()
{
  brushed_board_ros_loop();
}

// WRIST PITCH MAX LIMIT SWITCH
void lim1ISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
  {
    is_homed = true;
    mot1._encoder->set_current_angle_es(wrist_pitch_max_angle * mot1._gear_ratio);
    last_interrupt_time = interrupt_time;
    lim1_state = digitalRead(LIM_1);
  }
}

// WRIST PITCH MIN LIMIT SWITCH
void lim2ISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
  {
    mot1._encoder->set_current_angle_es(wrist_pitch_min_angle * mot1._gear_ratio);
    last_interrupt_time = interrupt_time;
    lim2_state = digitalRead(LIM_2);
  }
}

void lim3ISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
  {
    last_interrupt_time = interrupt_time;
    lim3_state = digitalRead(LIM_3);
  }
}

void lim4ISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
  {
    last_interrupt_time = interrupt_time;
    lim4_state = digitalRead(LIM_4);
  }
}

void lim5ISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
  {
    last_interrupt_time = interrupt_time;
    lim5_state = digitalRead(LIM_5);
  }
}

void lim6ISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY_MS)
  {
    last_interrupt_time = interrupt_time;
    lim6_state = digitalRead(LIM_6);
  }
}

// HOMING SEQUENCE, go forward until limit switch is triggered
void brushed_board_homing()
{
  // Only do homing for linear joints
  if (!mot1._is_circular_joint)
  {
    mot1.set_direction(mot1._forward_dir);
    mot1._pwm_write_duty(40);
  }
}

void process_serial_cmd()
{
  static String inputString = "";             // A String to hold incoming data
  static boolean inputStringComplete = false; // Whether the string is complete

  while (HWSERIAL.available())
  {
    char inChar = (char)HWSERIAL.read(); // Read each character
    if (inChar == '\n')
    {
      inputStringComplete = true; // If newline, input is complete
    }
    else
    {
      inputString += inChar; // Add character to input
    }
  }

  if (inputStringComplete)
  {
    HWSERIAL.print("Received: ");
    HWSERIAL.println(inputString); // Echo the input for debugging

    // Process the completed command
    if (inputString.startsWith("i "))
    {
      // Increment command
      float incrementValue = inputString.substring(2).toFloat(); // Extract number
      float pos = mot1.get_target_angle_ps();                    // Get current position
      mot1.set_target_angle_ps(pos + incrementValue);            // Increment and set new position
      HWSERIAL.print("Incremented position by: ");
      HWSERIAL.println(incrementValue);
    }
    else if (inputString.startsWith("s "))
    {
      // Set command
      pos = inputString.substring(2).toFloat(); // Extract and set new position
      mot1.set_target_angle_ps(pos);
      HWSERIAL.print("Set position to: ");
      HWSERIAL.println(pos);
    }
    else
    {
      HWSERIAL.println("Unknown command");
    }

    // Clear the string for the next command
    inputString = "";
    inputStringComplete = false;
  }
}

// PRINT ENCODER INFO
void print_encoder_info()
{
  // TEST LIMIT SWITCHES --------------------------------------------------------------------
  // printf("LIM_1: %d, LIM_2: %d, LIM_3: %d, LIM_4: %d, LIM_5: %d, LIM_6: %d\n",
  //        lim1_state, lim2_state, lim3_state, lim4_state, lim5_state, lim6_state);

  // TEST CURRENT SENSOR --------------------------------------------------------------------
  mot1._current_sensor->read_sensor_value();
  mot2._current_sensor->read_sensor_value();
  mot3._current_sensor->read_sensor_value();

  // TODO check implementation of get_current()
  // float cur1_current = mot1._current_sensor->get_current();
  // float cur2_current = mot2._current_sensor->get_current();
  // float cur3_current = mot3._current_sensor->get_current();

  // Middle value is 1.5V
  // float cur1_voltage = mot1._current_sensor->get_raw_voltage() - 1.5;
  // float cur2_voltage = mot2._current_sensor->get_raw_voltage() - 1.5;
  // float cur3_voltage = mot3._current_sensor->get_raw_voltage() - 1.5;
  // float smoothed_cur1_voltage = moving_average(cur1_voltage, cur1_voltage_buffer, MOVING_AVERAGE_SIZE, &cur1_voltage_buffer_idx);
  // float smoothed_cur2_voltage = moving_average(cur2_voltage, cur2_voltage_buffer, MOVING_AVERAGE_SIZE, &cur2_voltage_buffer_idx);
  // float smoothed_cur3_voltage = moving_average(cur3_voltage, cur3_voltage_buffer, MOVING_AVERAGE_SIZE, &cur3_voltage_buffer_idx);

  // HWSERIAL.printf("cur1_current: %8.4f, cur2_current: %8.4f, cur3_current: %8.4f ",
  //                  cur1_current, cur2_current, cur3_current);
  // HWSERIAL.printf("cur1_voltage: %8.4f, cur2_voltage: %8.4f, cur3_voltage: %8.4f, ",
  //                 smoothed_cur1_voltage, smoothed_cur2_voltage, smoothed_cur3_voltage);

  // TEST ENCODER ------------------------------------------------------------------------------

  // Channel 3 is conflicted, unused.
  mot1._encoder->poll_encoder_angle();
  mot2._encoder->poll_encoder_angle();
  // mot3._encoder->poll_encoder_angle();

  // Read encoder values.
  float enc1_quad_enc_pos = mot1._encoder->_encoder->read();
  float enc1_angle_single = mot1._encoder->get_angle_single();
  float enc1_angle_es = mot1.get_current_angle_es();
  float enc1_angle_ps = mot1.get_current_angle_ps();
  float enc1_setpoint = mot1.get_target_angle_ps();

  float enc2_quad_enc_pos = mot2._encoder->_encoder->read();
  float enc2_angle_single = mot2._encoder->get_angle_single();
  float enc2_angle_es = mot2.get_current_angle_es();
  float enc2_angle_ps = mot2.get_current_angle_ps();
  float enc2_setpoint = mot2.get_target_angle_ps();

  // float enc2_quad_enc_pos = mot2._encoder->_encoder->read();
  // float enc2_angle_single = mot2._encoder->get_angle_single();
  // float enc2_angle_multi = mot2._encoder->get_angle_multi();
  // float enc3_quad_enc_pos = mot3._encoder->_encoder->read();
  // float enc3_angle_single = mot3._encoder->get_angle_single();
  // float enc3_angle_multi = mot3._encoder->get_angle_multi();

  HWSERIAL.printf("enc1_quad_enc_pos: %8.4f, enc1_angle_single: %8.4f, enc1_angle_es: %8.4f, enc1_angle_ps: %8.4f, enc1_setpoint: %8.4f, ",
                  enc1_quad_enc_pos, enc1_angle_single, enc1_angle_es, enc1_angle_ps, enc1_setpoint);
  HWSERIAL.println();
  HWSERIAL.printf("enc2_quad_enc_pos: %8.4f, enc2_angle_single: %8.4f, enc2_angle_es: %8.4f, enc2_angle_ps: %8.4f, enc2_setpoint: %8.4f, ",
                  enc2_quad_enc_pos, enc2_angle_single, enc2_angle_es, enc2_angle_ps, enc2_setpoint);

  ros_printf("enc1_quad_enc_pos: %8.4f, enc1_angle_single: %8.4f, enc1_angle_es: %8.4f, enc1_angle_ps: %8.4f, enc1_setpoint: %8.4f, ",
             enc1_quad_enc_pos, enc1_angle_single, enc1_angle_es, enc1_angle_ps, enc1_setpoint);
  ros_printf("enc2_quad_enc_pos: %8.4f, enc2_angle_single: %8.4f, enc2_angle_es: %8.4f, enc2_angle_ps: %8.4f, enc2_setpoint: %8.4f, ",
             enc2_quad_enc_pos, enc2_angle_single, enc2_angle_es, enc2_angle_ps, enc2_setpoint);

  // float enc1_rev = mot1._encoder->_encoder->getRevolution();
  // float enc1_hold_rev = mot1._encoder->_encoder->getHoldRevolution();
  // HWSERIAL.printf("enc1_rev: %8.4f, enc1_hold_rev: %8.4f, ", enc1_rev, enc1_hold_rev);
  HWSERIAL.println();
}

void brushed_board_ros_loop()
{
#if DEBUG_PRINT == 1
  process_serial_cmd();
#endif
  // delay(PID_PERIOD_MS);

  uint32_t current_time = micros();
  if (current_time - loop_last_time > PID_PERIOD_US)
  {
    counter = current_time;
    mot1.closed_loop_control_tick();
    // mot2.closed_loop_control_tick();
    // mot3.closed_loop_control_tick();
    loop_last_time = current_time;
#if DEBUG_PRINT == 1
    print_encoder_info();
    ros_printf("EE: %8.4f, WR: %8.4f, WP: %8.4f, \n",
               arm_brushed_setpoint_ps[0], arm_brushed_setpoint_ps[1], arm_brushed_setpoint_ps[2]);
    ros_printf("EE dir_pin: %d, WR dir_pin: %d, WP dir_pin: %d, \n",
               mot3._motor_dir_pin, mot2._motor_dir_pin, mot1._motor_dir_pin);
#endif
  }
  // Feedback
  arm_brushed_angle_ps[0] = 0.0;
  // arm_brushed_angle_ps[0] = mot3.get_current_angle_ps();
  arm_brushed_angle_ps[1] = mot2.get_current_angle_ps();
  arm_brushed_angle_ps[2] = mot1.get_current_angle_ps();

  arm_brushed_fb_msg.data.size = 3;
  arm_brushed_fb_msg.data.capacity = 3;
  arm_brushed_fb_msg.data.data = arm_brushed_angle_ps;
  RCSOFTCHECK(rcl_publish(&arm_brushed_fb_pub, &arm_brushed_fb_msg, NULL));

  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
  delay(2);
}

void arm_brushed_cmd_cb(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  // 0 is EE, 1 is WR, 2 is WP
  arm_brushed_setpoint_ps[0] = msg->data.data[0];
  arm_brushed_setpoint_ps[1] = msg->data.data[1];
  arm_brushed_setpoint_ps[2] = msg->data.data[2];

  // Motor 3 is controlled like a forklift, only up and down, range -1 to 1, because encoder is not working
  mot1.set_target_angle_ps(arm_brushed_setpoint_ps[2]);
  mot2.move_manual(((float)arm_brushed_setpoint_ps[0]) / 100.0f);
  mot3.move_manual(((float)arm_brushed_setpoint_ps[1]) / 100.0f);

  HWSERIAL.printf("EE: %8.4f, WR: %8.4f, WP: %8.4f, \n", arm_brushed_setpoint_ps[0], arm_brushed_setpoint_ps[1], arm_brushed_setpoint_ps[2]);
  ros_printf("EE: %8.4f, WR: %8.4f, WP: %8.4f, \n", arm_brushed_setpoint_ps[0], arm_brushed_setpoint_ps[1], arm_brushed_setpoint_ps[2]);
}
#endif // USE_ROS_FIRMWARE == 1
