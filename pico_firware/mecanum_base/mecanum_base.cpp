/**
 * mecanum_base.cpp — micro-ROS mecanum wheel base firmware for RP2040 (Pico)
 *
 * Transport:  USB CDC  →  micro_ros_agent on RDK X5 (/dev/ttyACM0)
 *
 * Topics
 *   Subscribe: /cmd_vel        geometry_msgs/Twist
 *   Publish:   /odom           nav_msgs/Odometry          @ 20 Hz
 *              /wheel_speeds   std_msgs/Float32MultiArray  @ 20 Hz  [FL,RL,RR,FR rad/s]
 *
 * LED GP25:  fast blink = waiting for agent  |  solid = connected
 *
 * Pin map — AS WIRED (verified against motor_test):
 *   FL(A): PWM=GP2,  IN1=GP3,  IN2=GP4,  ENC_A=GP15, ENC_B=GP16
 *   RL(B): PWM=GP5,  IN1=GP7,  IN2=GP6,  ENC_A=GP17, ENC_B=GP18
 *   RR(C): PWM=GP8,  IN1=GP10, IN2=GP9,  ENC_A=GP20, ENC_B=GP19
 *   FR(D): PWM=GP11, IN1=GP12, IN2=GP13, ENC_A=GP22, ENC_B=GP21
 *   STBY  → GP14  (HIGH enables both TB6612 chips)
 *
 * Tune before running:
 *   WHEEL_R  — wheel radius in metres
 *   LX       — half wheelbase (centre ↔ front/rear axle) in metres
 *   LY       — half track width (centre ↔ left/right wheel) in metres
 *   ENC_CPR  — encoder counts/revolution (4× quadrature × mechanical CPR)
 *   MAX_W    — maximum wheel angular speed in rad/s (sets PWM 100% duty)
 */

#include <cstring>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/time.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/float32_multi_array.h>

// USB serial transport provided by libmicroros (micro_ros_raspberrypi_pico_sdk)
extern "C" {
bool   pico_serial_transport_open (struct uxrCustomTransport *);
bool   pico_serial_transport_close(struct uxrCustomTransport *);
size_t pico_serial_transport_write(struct uxrCustomTransport *, const uint8_t *, size_t, uint8_t *);
size_t pico_serial_transport_read (struct uxrCustomTransport *, uint8_t *, size_t, int, uint8_t *);
}

// ─── Macros ───────────────────────────────────────────────────────────────────
// Returns false on any RCL error — used only inside bool functions.
#define RCCHECK(fn) do { if ((fn) != RCL_RET_OK) return false; } while (0)
// Silently discards errors for non-critical publish calls.
#define RCSOFTCHECK(fn) do { (void)(fn); } while (0)

// ─── Hardware constants ───────────────────────────────────────────────────────
static constexpr uint32_t PWM_WRAP = 999u;   // 125 MHz / 1000 = 125 kHz
static constexpr uint     PIN_STBY = 14u;
static constexpr uint     PIN_LED  = 25u;

struct MotorPins { uint pwm, in1, in2, enc_a, enc_b; };

// Array order: FL=0, RL=1, RR=2, FR=3  (TB6612 channels A, B, C, D)
static const MotorPins MOTORS[4] = {
    {  2,  3,  4, 15, 16 },   // FL (channel A)
    {  5,  7,  6, 17, 18 },   // RL (channel B) — in1/in2 deliberately swapped vs silkscreen
    {  8, 10,  9, 20, 19 },   // RR (channel C) — in1/in2 deliberately swapped vs silkscreen
    { 11, 12, 13, 22, 21 },   // FR (channel D)
};

// ─── Robot geometry (measure from actual chassis before tuning) ───────────────
static constexpr float WHEEL_R   = 0.04f;   // wheel radius [m]
static constexpr float LX        = 0.125f;   // half wheelbase (front-rear) [m]
static constexpr float LY        = 0.200f;   // half track width (left-right) [m]
static constexpr float ENC_CPR   = 1560.0f;  // 13 pulses/rev × 4× quadrature × 30:1 gear
static constexpr float MAX_W     = 20.0f;    // max wheel angular speed [rad/s]
static constexpr float WD_SECS   = 0.5f;     // cmd_vel watchdog timeout [s]

// ─── Encoder state (written by ISR) ──────────────────────────────────────────
static volatile int32_t enc_count[4] = {};
static volatile uint8_t enc_state[4] = {};

// Quadrature decode table: index = (prev_AB << 2) | curr_AB
static const int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

static void encoder_isr(uint gpio, uint32_t /*events*/)
{
    for (int m = 0; m < 4; ++m) {
        const MotorPins &mp = MOTORS[m];
        if (gpio != mp.enc_a && gpio != mp.enc_b) continue;
        uint8_t curr = (uint8_t)((gpio_get(mp.enc_a) << 1) | gpio_get(mp.enc_b));
        uint8_t prev = enc_state[m];
        enc_state[m] = curr;
        enc_count[m] += QEM[(prev << 2) | curr];
        break;
    }
}

// ─── PWM / motor helpers ──────────────────────────────────────────────────────
static void pwm_init_pin(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint sl = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(sl, PWM_WRAP);
    pwm_set_clkdiv(sl, 1.0f);
    pwm_set_enabled(sl, true);
}

// Open-loop feedforward: maps desired wheel angular velocity → PWM + direction.
static void motor_drive(int m, float rad_s)
{
    const MotorPins &mp = MOTORS[m];
    float pct  = fabsf(rad_s) / MAX_W;
    if (pct > 1.0f) pct = 1.0f;
    uint16_t duty = (uint16_t)(pct * PWM_WRAP);

    if (rad_s > 0.0f) {
        gpio_put(mp.in1, 1); gpio_put(mp.in2, 0);
    } else if (rad_s < 0.0f) {
        gpio_put(mp.in1, 0); gpio_put(mp.in2, 1);
    } else {
        gpio_put(mp.in1, 1); gpio_put(mp.in2, 1);  // active brake
        duty = 0;
    }
    pwm_set_gpio_level(mp.pwm, duty);
}

static void all_brake()
{
    for (int m = 0; m < 4; ++m) motor_drive(m, 0.0f);
}

// ─── Mecanum kinematics ───────────────────────────────────────────────────────
// IK: body velocity (vx=fwd, vy=left, wz=CCW) → individual wheel angular speeds
static void ik(float vx, float vy, float wz, float out[4])
{
    const float k = LX + LY;
    out[0] = (vx - vy - k * wz) / WHEEL_R;   // FL
    out[1] = (vx + vy - k * wz) / WHEEL_R;   // RL
    out[2] = (vx - vy + k * wz) / WHEEL_R;   // RR
    out[3] = (vx + vy + k * wz) / WHEEL_R;   // FR
}

// FK: measured wheel angular speeds → body velocity (for odometry)
static void fk(const float w[4], float &vx, float &vy, float &wz)
{
    vx = WHEEL_R * 0.25f * ( w[0] + w[1] + w[2] + w[3]);
    vy = WHEEL_R * 0.25f * (-w[0] + w[1] - w[2] + w[3]);
    wz = WHEEL_R / (4.0f * (LX + LY)) * (-w[0] - w[1] + w[2] + w[3]);
}

// ─── Hardware init ────────────────────────────────────────────────────────────
static void hw_init()
{
    gpio_init(PIN_STBY); gpio_set_dir(PIN_STBY, GPIO_OUT); gpio_put(PIN_STBY, 1);
    gpio_init(PIN_LED);  gpio_set_dir(PIN_LED,  GPIO_OUT); gpio_put(PIN_LED,  0);

    for (int m = 0; m < 4; ++m) {
        const MotorPins &mp = MOTORS[m];

        gpio_init(mp.in1); gpio_set_dir(mp.in1, GPIO_OUT); gpio_put(mp.in1, 0);
        gpio_init(mp.in2); gpio_set_dir(mp.in2, GPIO_OUT); gpio_put(mp.in2, 0);
        pwm_init_pin(mp.pwm);
        pwm_set_gpio_level(mp.pwm, 0);

        gpio_init(mp.enc_a); gpio_set_dir(mp.enc_a, GPIO_IN); gpio_pull_up(mp.enc_a);
        gpio_init(mp.enc_b); gpio_set_dir(mp.enc_b, GPIO_IN); gpio_pull_up(mp.enc_b);

        enc_state[m] = (uint8_t)((gpio_get(mp.enc_a) << 1) | gpio_get(mp.enc_b));
        enc_count[m] = 0;

        gpio_set_irq_enabled_with_callback(
            mp.enc_a, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, encoder_isr);
        gpio_set_irq_enabled(
            mp.enc_b, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }
}

// ─── micro-ROS objects ────────────────────────────────────────────────────────
static rcl_node_t         node;
static rcl_publisher_t    odom_pub;
static rcl_publisher_t    ws_pub;
static rcl_subscription_t cmd_vel_sub;
static rcl_timer_t        odom_timer;
static rclc_executor_t    executor;
static rclc_support_t     support;
static rcl_allocator_t    allocator;

// ─── Messages ─────────────────────────────────────────────────────────────────
static geometry_msgs__msg__Twist        cmd_vel_msg;
static nav_msgs__msg__Odometry          odom_msg;
static std_msgs__msg__Float32MultiArray ws_msg;
static float ws_data[4] = {};

// Static frame ID strings avoid heap allocation on the Pico.
static char frame_odom[]  = "odom";
static char frame_base[]  = "base_link";

// ─── Odometry state ───────────────────────────────────────────────────────────
static double  odom_x     = 0.0;
static double  odom_y     = 0.0;
static double  odom_theta = 0.0;
static int32_t enc_prev[4] = {};
static int64_t last_cmd_us  = 0;
static int64_t last_odom_us = 0;

// ─── Callbacks ────────────────────────────────────────────────────────────────
static void cmd_vel_callback(const void *msg_in)
{
    const auto *twist = static_cast<const geometry_msgs__msg__Twist *>(msg_in);
    last_cmd_us = time_us_64();

    float wheel_w[4];
    ik((float)twist->linear.x, (float)twist->linear.y, (float)twist->angular.z, wheel_w);
    for (int m = 0; m < 4; ++m) motor_drive(m, wheel_w[m]);
}

static void odom_timer_callback(rcl_timer_t * /*timer*/, int64_t /*last_call_ns*/)
{
    int64_t now_us = time_us_64();
    float dt = (last_odom_us > 0) ? (float)(now_us - last_odom_us) * 1e-6f : 0.05f;
    last_odom_us = now_us;

    // Watchdog: coast to a stop if cmd_vel goes silent
    if (last_cmd_us > 0 && (now_us - last_cmd_us) > (int64_t)(WD_SECS * 1e6f)) {
        all_brake();
        last_cmd_us = 0;
    }

    // Encoder deltas → wheel angular speeds [rad/s]
    float wheel_w[4];
    for (int m = 0; m < 4; ++m) {
        int32_t cnt   = enc_count[m];
        int32_t delta = cnt - enc_prev[m];
        enc_prev[m]   = cnt;
        wheel_w[m]    = ((float)delta / ENC_CPR) * (2.0f * (float)M_PI) / dt;
        ws_data[m]    = wheel_w[m];
    }

    // Forward kinematics → body velocity
    float vx, vy, wz;
    fk(wheel_w, vx, vy, wz);

    // Integrate pose in world frame
    double ct = cos(odom_theta), st = sin(odom_theta);
    odom_x     += (ct * (double)vx - st * (double)vy) * dt;
    odom_y     += (st * (double)vx + ct * (double)vy) * dt;
    odom_theta += (double)wz * dt;

    // Timestamp (epoch-synchronised with agent via rmw_uros_sync_session)
    int64_t epoch_ms = rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec     = (int32_t)(epoch_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((epoch_ms % 1000) * 1000000LL);

    // Pose
    odom_msg.pose.pose.position.x    = odom_x;
    odom_msg.pose.pose.position.y    = odom_y;
    odom_msg.pose.pose.position.z    = 0.0;
    odom_msg.pose.pose.orientation.z = sin(odom_theta * 0.5);
    odom_msg.pose.pose.orientation.w = cos(odom_theta * 0.5);
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;

    // Velocity (expressed in the base_link frame)
    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.linear.y  = vy;
    odom_msg.twist.twist.angular.z = wz;

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&ws_pub,   &ws_msg,   NULL));
}

// ─── Entity lifecycle ─────────────────────────────────────────────────────────
static bool create_entities()
{
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "mecanum_base", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    RCCHECK(rclc_publisher_init_default(
        &ws_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "wheel_speeds"));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    RCCHECK(rclc_timer_init_default(
        &odom_timer, &support, RCL_MS_TO_NS(50), odom_timer_callback));   // 20 Hz

    // 2 handles: 1 subscriber + 1 timer
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));

    rmw_uros_sync_session(1000);   // sync Pico clock with agent epoch

    // Reset odometry on each (re)connect
    odom_x = odom_y = odom_theta = 0.0;
    for (int m = 0; m < 4; ++m) { enc_prev[m] = enc_count[m]; }
    last_odom_us = last_cmd_us = 0;

    return true;
}

static void destroy_entities()
{
    all_brake();
    rclc_executor_fini(&executor);
    rcl_timer_fini(&odom_timer);
    rcl_publisher_fini(&odom_pub, &node);
    rcl_publisher_fini(&ws_pub,   &node);
    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ─── main ─────────────────────────────────────────────────────────────────────
int main()
{
    hw_init();

    // Register USB serial transport (provided by libmicroros)
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    // Init odometry message — use static char arrays for frame IDs
    nav_msgs__msg__Odometry__init(&odom_msg);
    odom_msg.header.frame_id.data     = frame_odom;
    odom_msg.header.frame_id.size     = sizeof(frame_odom) - 1;
    odom_msg.header.frame_id.capacity = sizeof(frame_odom);
    odom_msg.child_frame_id.data      = frame_base;
    odom_msg.child_frame_id.size      = sizeof(frame_base) - 1;
    odom_msg.child_frame_id.capacity  = sizeof(frame_base);

    // Diagonal covariance — wheel odometry only; EKF on host fuses IMU
    memset(odom_msg.pose.covariance,  0, sizeof(odom_msg.pose.covariance));
    memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
    odom_msg.pose.covariance[0]   = 1e-3;   // x
    odom_msg.pose.covariance[7]   = 1e-3;   // y
    odom_msg.pose.covariance[35]  = 1e-2;   // yaw
    odom_msg.twist.covariance[0]  = 1e-3;   // vx
    odom_msg.twist.covariance[7]  = 1e-3;   // vy
    odom_msg.twist.covariance[35] = 1e-2;   // wz

    // Init wheel-speed message (no dim metadata needed for a simple float array)
    ws_msg.data.data     = ws_data;
    ws_msg.data.size     = 4;
    ws_msg.data.capacity = 4;

    // ── Connection state machine ──────────────────────────────────────────────
    enum class State { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };
    State    state    = State::WAITING;
    uint32_t blink_t  = 0;
    int      ping_cnt = 0;

    while (true) {
        switch (state) {

            case State::WAITING:
                // Fast blink (100 ms) while waiting for the micro_ros_agent
                if (to_ms_since_boot(get_absolute_time()) - blink_t > 100u) {
                    blink_t = to_ms_since_boot(get_absolute_time());
                    gpio_xor_mask(1u << PIN_LED);
                }
                if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                    state = State::AVAILABLE;
                break;

            case State::AVAILABLE:
                if (create_entities()) {
                    gpio_put(PIN_LED, 1);   // solid = connected
                    ping_cnt = 0;
                    state = State::CONNECTED;
                } else {
                    state = State::DISCONNECTED;
                }
                break;

            case State::CONNECTED:
                // Spin executor at ~100 Hz; ping the agent roughly every second
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
                if (++ping_cnt >= 100) {
                    ping_cnt = 0;
                    if (RMW_RET_OK != rmw_uros_ping_agent(200, 1))
                        state = State::DISCONNECTED;
                }
                break;

            case State::DISCONNECTED:
                destroy_entities();
                gpio_put(PIN_LED, 0);
                state = State::WAITING;
                break;
        }
    }
}
