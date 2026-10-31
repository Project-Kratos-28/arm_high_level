#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <SCServo.h>
#include <math.h>

// ============================================================================
// PIN DEFINITIONS (1 PWM Pin + 1 Direction Pin for each of the 3 Motors)
// ============================================================================

// Motor 1: Base
#define MOTOR_BASE_PWM_PIN       5    // PWM Pin (Quad 3)
#define MOTOR_BASE_DIR_PIN       15   // Direction Pin (Quad 3)

// Motor 2: Shoulder
#define MOTOR_SHOULDER_PWM_PIN   0    // PWM Pin (Quad 1)
#define MOTOR_SHOULDER_DIR_PIN   4    // Direction Pin (Quad 1)

// Motor 3: Elbow
#define MOTOR_ELBOW_PWM_PIN      32   // PWM Pin (Quad 1)
#define MOTOR_ELBOW_DIR_PIN      33   // Direction Pin (Quad 1)

// Motor 4: Wrist 1 (Differential Bevel System)
#define MOTOR_WRIST1_PWM_PIN     26   // PWM Pin
#define MOTOR_WRIST1_DIR_PIN     25   // Direction Pin

// Motor 5: Wrist 2 (Differential Bevel System - Quad 4)
#define MOTOR_WRIST2_PWM_PIN     12   // PWM Pin
#define MOTOR_WRIST2_DIR_PIN     14   // Direction Pin

// ST3215 serial bus gripper via Waveshare Bus Servo Adapter (A)
#define GRIPPER_SERVO_RX_PIN     16   // ESP32 RX is GPIO 16 (connected to Waveshare TX)
#define GRIPPER_SERVO_TX_PIN     17   // ESP32 TX is GPIO 17 (connected to Waveshare RX)
#define GRIPPER_SERVO_ID         1
#define GRIPPER_SERVO_BAUD       1000000
#define GRIPPER_RAW_OPEN         1830  // Open position (0 deg)
#define GRIPPER_RAW_CLOSED       800   // Closed position (90 deg)
#define GRIPPER_STEP_INTERVAL_MS 40
#define GRIPPER_MOVE_SPEED       500
#define GRIPPER_MOVE_ACC         50

// Status LED (GPIO 2 is standard onboard LED for ESP32-WROOM)
#define LED_PIN                  2

// Direction polarity inversion flags
#define INVERT_BASE_DIR          false
#define INVERT_SHOULDER_DIR      true
#define INVERT_ELBOW_DIR         false
#define INVERT_WRIST1_DIR        false
#define INVERT_WRIST2_DIR        false

// ============================================================================
// PWM CONFIGURATION (ESP32 LEDC Peripheral)
// ============================================================================
#define PWM_FREQ_HZ              20000  // 20 kHz
#define PWM_RESOLUTION_BITS      8      // 8-bit resolution: 0 to 255

// LEDC channels for the 5 motor PWM pins
#define CH_BASE_PWM              0
#define CH_SHOULDER_PWM          1
#define CH_ELBOW_PWM             2
#define CH_WRIST1_PWM            3
#define CH_WRIST2_PWM            4

// ============================================================================
// MICRO-ROS HANDLES & RECONNECTION STATE
// ============================================================================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Subscriber for incoming commands
rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;

// Feedback publisher echoing back actual hardware pin states:
// [base_pwm, base_dir, shoulder_pwm, shoulder_dir, elbow_pwm, elbow_dir,
//  wrist1_pwm, wrist1_dir, wrist2_pwm, wrist2_dir]
rcl_publisher_t feedback_pub;
std_msgs__msg__Int16MultiArray feedback_msg;

// Static message memory buffers
static int16_t msg_data_buffer[16];
static std_msgs__msg__MultiArrayDimension dim_buffer[1];
static char dim_label_buffer[16];

static int16_t feedback_data_buffer[16];
static std_msgs__msg__MultiArrayDimension feedback_dim_buffer[1];
static char feedback_dim_label_buffer[16];

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ST3215 gripper state. Fractional raw counts preserve the 1-degree step size.
SMS_STS gripper;
bool gripper_initialized = false;
int8_t gripper_command = 0;       // +1=open, 0=hold, -1=close
float gripper_raw_target = GRIPPER_RAW_CLOSED;
int16_t gripper_actual_pos = -1;
int16_t gripper_voltage = -1;
uint16_t gripper_tx_packets = 0;
unsigned long last_gripper_step_ms = 0;

// ============================================================================
// ST3215 GRIPPER CONTROL
// ============================================================================
bool initialize_gripper() {
  Serial1.end();
  delay(10);
  Serial1.begin(GRIPPER_SERVO_BAUD, SERIAL_8N1,
                GRIPPER_SERVO_RX_PIN, GRIPPER_SERVO_TX_PIN);
  gripper.pSerial = &Serial1;
  gripper.IOTimeOut = 10; // 10ms timeout
  gripper.Level = 0;      // Non-blocking writes (no ACK wait for WritePosEx)

  // Read current position directly
  int pos = gripper.ReadPos(GRIPPER_SERVO_ID);
  if (pos >= 0) {
    gripper_actual_pos = (int16_t)pos;
    gripper_raw_target = (float)pos;
  } else {
    gripper_actual_pos = -1;
    gripper_raw_target = (float)GRIPPER_RAW_CLOSED;
  }

  int volt = gripper.ReadVoltage(GRIPPER_SERVO_ID);
  if (volt >= 0) {
    gripper_voltage = (int16_t)volt;
  } else {
    gripper_voltage = -1;
  }

  // Keep the measured position, but constrain future targets to the calibrated range.
  float min_pos = min((float)GRIPPER_RAW_OPEN, (float)GRIPPER_RAW_CLOSED);
  float max_pos = max((float)GRIPPER_RAW_OPEN, (float)GRIPPER_RAW_CLOSED);

  if (gripper_raw_target < min_pos) {
    gripper_raw_target = min_pos;
  }
  if (gripper_raw_target > max_pos) {
    gripper_raw_target = max_pos;
  }

  gripper.EnableTorque(GRIPPER_SERVO_ID, 1);
  gripper_command = 0;
  last_gripper_step_ms = millis();
  gripper_initialized = true; // Allow writes to proceed even if initial read was noisy
  return true;
}

void read_gripper_telemetry() {
  if (!gripper_initialized) {
    initialize_gripper();
    return;
  }
  int pos = gripper.ReadPos(GRIPPER_SERVO_ID);
  if (pos >= 0) {
    gripper_actual_pos = (int16_t)pos;
  }
  int volt = gripper.ReadVoltage(GRIPPER_SERVO_ID);
  if (volt >= 0) {
    gripper_voltage = (int16_t)volt;
  }
}

void update_gripper() {
  if (!gripper_initialized || gripper_command == 0) {
    return;
  }

  unsigned long now = millis();
  if (now - last_gripper_step_ms < GRIPPER_STEP_INTERVAL_MS) {
    return;
  }
  last_gripper_step_ms = now;

  // Degrees span between closed and open
  const float raw_counts_per_degree =
      (float)(GRIPPER_RAW_OPEN - GRIPPER_RAW_CLOSED) / 90.0f;

  // gripper_command: +1 = open (moves towards GRIPPER_RAW_OPEN), -1 = close (moves towards GRIPPER_RAW_CLOSED)
  gripper_raw_target +=
      (float)gripper_command * raw_counts_per_degree;

  float min_pos = min((float)GRIPPER_RAW_OPEN, (float)GRIPPER_RAW_CLOSED);
  float max_pos = max((float)GRIPPER_RAW_OPEN, (float)GRIPPER_RAW_CLOSED);

  if (gripper_raw_target < min_pos) {
    gripper_raw_target = min_pos;
  }
  if (gripper_raw_target > max_pos) {
    gripper_raw_target = max_pos;
  }

  gripper.WritePosEx(GRIPPER_SERVO_ID,
                     (int)lroundf(gripper_raw_target),
                     GRIPPER_MOVE_SPEED,
                     GRIPPER_MOVE_ACC);
  gripper_tx_packets++;

  if (gripper_raw_target <= min_pos || gripper_raw_target >= max_pos) {
    gripper_command = 0;
  }
}

// ============================================================================
// MOTOR CONTROL HELPER FUNCTIONS (PWM + DIR ARCHITECTURE)
// ============================================================================

void set_motor_speed(uint8_t pwm_ch, uint8_t dir_pin, int32_t speed, bool invert_dir) {
  // Clamp speed to [-255, 255]
  if (speed > 255)  speed = 255;
  if (speed < -255) speed = -255;

  if (speed > 0) {
    digitalWrite(dir_pin, invert_dir ? LOW : HIGH);
    ledcWrite(pwm_ch, (uint32_t)speed);
  } else if (speed < 0) {
    digitalWrite(dir_pin, invert_dir ? HIGH : LOW);
    ledcWrite(pwm_ch, (uint32_t)(-speed));
  } else {
    ledcWrite(pwm_ch, 0);
    digitalWrite(dir_pin, LOW);
  }
}

void stop_all_motors() {
  ledcWrite(CH_BASE_PWM, 0);
  ledcWrite(CH_SHOULDER_PWM, 0);
  ledcWrite(CH_ELBOW_PWM, 0);
  ledcWrite(CH_WRIST1_PWM, 0);
  ledcWrite(CH_WRIST2_PWM, 0);

  digitalWrite(MOTOR_BASE_DIR_PIN, LOW);
  digitalWrite(MOTOR_SHOULDER_DIR_PIN, LOW);
  digitalWrite(MOTOR_ELBOW_DIR_PIN, LOW);
  digitalWrite(MOTOR_WRIST1_DIR_PIN, LOW);
  digitalWrite(MOTOR_WRIST2_DIR_PIN, LOW);
}

void publish_feedback() {
  if (state != AGENT_CONNECTED || feedback_pub.impl == NULL) {
    return;
  }

  // Hardware Readback: read directly from ESP32 timer registers & pin input buffers
  // Array layout: [base_pwm, base_dir, shoulder_pwm, shoulder_dir, elbow_pwm, elbow_dir,
  //                wrist1_pwm, wrist1_dir, wrist2_pwm, wrist2_dir]
  feedback_data_buffer[0] = (int16_t)ledcRead(CH_BASE_PWM);
  feedback_data_buffer[1] = (int16_t)digitalRead(MOTOR_BASE_DIR_PIN);
  feedback_data_buffer[2] = (int16_t)ledcRead(CH_SHOULDER_PWM);
  feedback_data_buffer[3] = (int16_t)digitalRead(MOTOR_SHOULDER_DIR_PIN);
  feedback_data_buffer[4] = (int16_t)ledcRead(CH_ELBOW_PWM);
  feedback_data_buffer[5] = (int16_t)digitalRead(MOTOR_ELBOW_DIR_PIN);
  feedback_data_buffer[6] = (int16_t)ledcRead(CH_WRIST1_PWM);
  feedback_data_buffer[7] = (int16_t)digitalRead(MOTOR_WRIST1_DIR_PIN);
  feedback_data_buffer[8] = (int16_t)ledcRead(CH_WRIST2_PWM);
  feedback_data_buffer[9] = (int16_t)digitalRead(MOTOR_WRIST2_DIR_PIN);

  // ST3215 Gripper Live Telemetry (Indices 10..15)
  feedback_data_buffer[10] = (int16_t)gripper_command;
  feedback_data_buffer[11] = (int16_t)lroundf(gripper_raw_target);
  feedback_data_buffer[12] = (int16_t)gripper_actual_pos;
  feedback_data_buffer[13] = (int16_t)gripper_voltage;
  feedback_data_buffer[14] = (int16_t)gripper_tx_packets;
  feedback_data_buffer[15] = (int16_t)(gripper_initialized ? 1 : 0);

  feedback_msg.data.size = 16;
  rcl_publish(&feedback_pub, &feedback_msg, NULL);
}

// ============================================================================
// ROS 2 SUBSCRIBER CALLBACK
// ============================================================================
void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int16MultiArray * array = (const std_msgs__msg__Int16MultiArray *)msgin;

  if (array != NULL && array->data.data != NULL) {
    // Index 0: Base Motor (-255 to +255 scaled down by 2, limited to ±125)
    if (array->data.size > 0) {
      int32_t base_cmd = ((int32_t)array->data.data[0]) / 2;
      if (base_cmd > 125)  base_cmd = 125;
      if (base_cmd < -125) base_cmd = -125;
      set_motor_speed(CH_BASE_PWM, MOTOR_BASE_DIR_PIN, base_cmd, INVERT_BASE_DIR);
    }

    // Index 1: Shoulder Motor (-255 to +255)
    if (array->data.size > 1) {
      set_motor_speed(CH_SHOULDER_PWM, MOTOR_SHOULDER_DIR_PIN, array->data.data[1], INVERT_SHOULDER_DIR);
    }

    // Index 2: Elbow Motor (-255 to +255)
    if (array->data.size > 2) {
      set_motor_speed(CH_ELBOW_PWM, MOTOR_ELBOW_DIR_PIN, array->data.data[2], INVERT_ELBOW_DIR);
    }

    // Index 3 & 4: Differential Wrist Bevel System
    // array->data.data[3] = Wrist Pitch (-255 to +255)
    // array->data.data[4] = Wrist Roll  (-255 to +255)
    // Rotate together -> pitch, rotate opposite -> roll
    int32_t pitch = (array->data.size > 3) ? (int32_t)array->data.data[3] : 0;
    int32_t roll  = (array->data.size > 4) ? (int32_t)array->data.data[4] : 0;

    int32_t w1 = pitch + roll;
    int32_t w2 = pitch - roll;

    // Proportional normalization if combined magnitude exceeds 255
    int32_t max_mag = max(abs(w1), abs(w2));
    if (max_mag > 255) {
      w1 = (w1 * 255) / max_mag;
      w2 = (w2 * 255) / max_mag;
    }

    set_motor_speed(CH_WRIST1_PWM, MOTOR_WRIST1_DIR_PIN, w1, INVERT_WRIST1_DIR);
    set_motor_speed(CH_WRIST2_PWM, MOTOR_WRIST2_DIR_PIN, w2, INVERT_WRIST2_DIR);

    // Index 5: Gripper command: +1=open, 0=hold, -1=close.
    if (array->data.size > 5) {
      int16_t command = array->data.data[5];
      gripper_command = (command > 0) ? 1 : ((command < 0) ? -1 : 0);
    }

    // Publish feedback immediately on every command tick (1:1 sync)
    publish_feedback();
  }
}

// ============================================================================
// ENTITY MANAGEMENT (Create & Destroy for reconnects)
// ============================================================================
void init_message_memory()
{
  msg.data.data = msg_data_buffer;
  msg.data.size = 0;
  msg.data.capacity = 16;

  dim_buffer[0].label.data = dim_label_buffer;
  dim_buffer[0].label.size = 0;
  dim_buffer[0].label.capacity = sizeof(dim_label_buffer);
  dim_buffer[0].size = 0;
  dim_buffer[0].stride = 0;

  msg.layout.dim.data = dim_buffer;
  msg.layout.dim.size = 0;
  msg.layout.dim.capacity = 1;
  msg.layout.data_offset = 0;

  feedback_msg.data.data = feedback_data_buffer;
  feedback_msg.data.size = 16;
  feedback_msg.data.capacity = 16;

  feedback_dim_buffer[0].label.data = feedback_dim_label_buffer;
  feedback_dim_buffer[0].label.size = 0;
  feedback_dim_buffer[0].label.capacity = sizeof(feedback_dim_label_buffer);
  feedback_dim_buffer[0].size = 16;
  feedback_dim_buffer[0].stride = 16;

  feedback_msg.layout.dim.data = feedback_dim_buffer;
  feedback_msg.layout.dim.size = 0;
  feedback_msg.layout.dim.capacity = 1;
  feedback_msg.layout.data_offset = 0;
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // Zero-initialize handles
  subscriber = rcl_get_zero_initialized_subscription();
  feedback_pub = rcl_get_zero_initialized_publisher();
  executor = rclc_executor_get_zero_initialized_executor();
  node = rcl_get_zero_initialized_node();
  memset(&support, 0, sizeof(rclc_support_t));

  // Initialize options & context
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));

  // Set up message memory statically
  init_message_memory();

  // Create subscriber for "arm_cmd"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "arm_cmd"));

  // Create feedback publisher for "arm_feedback"
  RCCHECK(rclc_publisher_init_default(
    &feedback_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "arm_feedback"));

  // Create executor with 1 subscriber handle
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  stop_all_motors();
  gripper_command = 0;  // Hold the gripper at its last commanded position.

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  if (subscriber.impl != NULL) {
    rcl_subscription_fini(&subscriber, &node);
  }
  if (feedback_pub.impl != NULL) {
    rcl_publisher_fini(&feedback_pub, &node);
  }
  if (executor.context != NULL) {
    rclc_executor_fini(&executor);
  }
  if (node.impl != NULL) {
    rcl_node_fini(&node);
  }
  if (support.context.impl != NULL) {
    rclc_support_fini(&support);
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================
void setup() {
  // Initialize Serial micro-ROS transport
  set_microros_transports();
  initialize_gripper();

  // Status LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Setup Direction Pins as standard digital outputs
  pinMode(MOTOR_BASE_DIR_PIN, OUTPUT);
  pinMode(MOTOR_SHOULDER_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ELBOW_DIR_PIN, OUTPUT);
  pinMode(MOTOR_WRIST1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_WRIST2_DIR_PIN, OUTPUT);

  digitalWrite(MOTOR_BASE_DIR_PIN, LOW);
  digitalWrite(MOTOR_SHOULDER_DIR_PIN, LOW);
  digitalWrite(MOTOR_ELBOW_DIR_PIN, LOW);
  digitalWrite(MOTOR_WRIST1_DIR_PIN, LOW);
  digitalWrite(MOTOR_WRIST2_DIR_PIN, LOW);

  // Setup LEDC PWM Channels for the 5 motor PWM pins (20 kHz, 8-bit)
  ledcSetup(CH_BASE_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_BASE_PWM_PIN, CH_BASE_PWM);

  ledcSetup(CH_SHOULDER_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_SHOULDER_PWM_PIN, CH_SHOULDER_PWM);

  ledcSetup(CH_ELBOW_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_ELBOW_PWM_PIN, CH_ELBOW_PWM);

  ledcSetup(CH_WRIST1_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_WRIST1_PWM_PIN, CH_WRIST1_PWM);

  ledcSetup(CH_WRIST2_PWM, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(MOTOR_WRIST2_PWM_PIN, CH_WRIST2_PWM);

  // ST3215 gripper is initialized on UART1 (GPIO 16 RX, GPIO 17 TX).

  // Initial safety: motors off
  stop_all_motors();

  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      // Ping micro-ROS agent every 500 ms
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      // Heartbeat blink while waiting
      EXECUTE_EVERY_N_MS(300, digitalWrite(LED_PIN, !digitalRead(LED_PIN)););
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      } else {
        digitalWrite(LED_PIN, HIGH);
      }
      break;

    case AGENT_CONNECTED:
      // Ping agent every 1000 ms to monitor connection health
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        update_gripper();
        EXECUTE_EVERY_N_MS(250, read_gripper_telemetry(););
        // Idle heartbeat feedback at 1 Hz when no commands are active
        EXECUTE_EVERY_N_MS(1000, publish_feedback(););
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      digitalWrite(LED_PIN, LOW);
      state = WAITING_AGENT;
      break;

    default:
      break;
  }
}
