#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <irobot_create_msgs.h>


// MicroRos arguments
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_timer_t timer;

// Create node object
rcl_node_t node;
const char *node_name_ = "roomba";

// Node namespace (Can remain empty "")
const char *namespace_ = "diff_cont";

// Setting publishers
rcl_publisher_t battery;
create_msgs__msg__Battery battery_msg;
std_msgs__msg__Header header;

#define LED_PIN 2

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect and ESP32 Dev module
#endif

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

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // filling battery_msg
    battery_msg.header = header;
    battery_msg.voltage = data[7];
    battery_msg.current = data[8];
    battery_msg.temperature = data[9];
    battery_msg.charge = data[10];
    battery_msg.capacity = data[11];

    RCSOFTCHECK(rcl_publish(&battery, &battery_msg, NULL));
  }
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  pinMode(15, INPUT);
  pinMode(5,INPUT);
  pinMode(4, INPUT);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, node_name_, namespace_, &support));

  // create publishers
  RCCHECK(rclc_publisher_init_default(
      &battery,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(create_msgs, msg, Battery),
      "battery"));
  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}