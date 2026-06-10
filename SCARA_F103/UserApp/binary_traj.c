#include "binary_traj.h"

#include "app_config.h"
#include "motion_planner.h"
#include "scara_kinematics.h"
#include "serial_dma.h"
#include "stepper_driver.h"
#include "stm32f1xx_hal.h"

#include <string.h>
#include <stdlib.h>

#define BT_SOF0 0xA5u
#define BT_SOF1 0x5Au

#define BT_TYPE_HELLO 0x01u
#define BT_TYPE_BEGIN 0x10u
#define BT_TYPE_CHUNK 0x11u
#define BT_TYPE_VALIDATE 0x12u
#define BT_TYPE_RUN 0x13u
#define BT_TYPE_ABORT 0x14u
#define BT_TYPE_STATUS 0x15u

#define BT_TYPE_ACK 0x80u
#define BT_TYPE_NACK 0x81u
#define BT_TYPE_STATUS_RSP 0x82u

#define BT_ERR_OK 0u
#define BT_ERR_BAD_VERSION 1u
#define BT_ERR_BAD_LEN 2u
#define BT_ERR_BAD_CRC 3u
#define BT_ERR_BAD_STATE 4u
#define BT_ERR_NO_SPACE 5u
#define BT_ERR_MOTION 6u

#define BT_POINT_FLAG_EXACT_STOP 0x0001u
#define BT_POINT_FLAG_CARTESIAN_LINE 0x0002u
#define BT_POINT_FLAG_HOST_TIMED 0x0004u

typedef struct {
    int32_t p1_abs;
    int32_t p2_abs;
    uint16_t v_dom_pps;
    uint16_t flags;
} BinaryTrajPoint;

typedef enum {
    PARSER_SOF0 = 0,
    PARSER_SOF1,
    PARSER_HEADER,
    PARSER_PAYLOAD,
    PARSER_CRC0,
    PARSER_CRC1
} ParserState;

static BinaryTrajPoint s_points[APP_BINARY_TRAJ_POINTS];
static uint16_t s_head;
static uint16_t s_tail;
static uint16_t s_count;
static uint32_t s_total_expected;
static uint32_t s_accepted_count;
static uint32_t s_executed_count;
static volatile BinaryTrajState s_state;
static volatile bool s_run_requested;
static uint32_t s_max_dispatch_gap_ticks;
static uint32_t s_stream_underrun_ticks;
static uint32_t s_stream_underrun_active_ticks;
static uint32_t s_stream_underrun_count;
static bool s_stream_underrun_active;
static uint16_t s_min_buffer_count;

typedef struct {
    bool active;
    int32_t start_x_um;
    int32_t start_y_um;
    int32_t end_x_um;
    int32_t end_y_um;
    int32_t total_um;
    int32_t target_um;
    uint16_t feed_mm_min;
} BinaryCartLine;

static BinaryCartLine s_cart_line;

static ParserState s_parser_state;
static uint8_t s_header[6];
static uint8_t s_header_index;
static uint8_t s_payload[APP_BINARY_TRAJ_MAX_PAYLOAD];
static uint16_t s_payload_index;
static uint16_t s_payload_len;
static uint16_t s_rx_crc;
static uint16_t s_calc_crc;
static uint8_t s_frame_ver;
static uint8_t s_frame_type;
static uint16_t s_frame_seq;

static uint16_t next_index(uint16_t index)
{
    index++;
    if (index >= APP_BINARY_TRAJ_POINTS) {
        index = 0;
    }
    return index;
}

static uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_restore(uint32_t primask)
{
    if (primask == 0u) {
        __enable_irq();
    }
}

static uint16_t crc16_update(uint16_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8u; ++i) {
        if ((crc & 1u) != 0u) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

static uint16_t crc16_buf(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; ++i) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)rd_u32(p);
}

static void wr_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)(value >> 8);
}

static void wr_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFu);
    p[1] = (uint8_t)((value >> 8) & 0xFFu);
    p[2] = (uint8_t)((value >> 16) & 0xFFu);
    p[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint16_t abs_i32_to_u16(int32_t value)
{
    int32_t out = value < 0 ? -value : value;
    if (out > 65535) {
        return 65535u;
    }
    return (uint16_t)out;
}

static int64_t i64_abs_local(int64_t value)
{
    return value < 0 ? -value : value;
}

static int32_t i32_isqrt_i64(int64_t value)
{
    if (value <= 0) {
        return 0;
    }
    uint64_t op = (uint64_t)value;
    uint64_t res = 0;
    uint64_t one = 1ull << 62;
    while (one > op) {
        one >>= 2;
    }
    while (one != 0) {
        if (op >= res + one) {
            op -= res + one;
            res = (res >> 1) + one;
        } else {
            res >>= 1;
        }
        one >>= 2;
    }
    return res > 2147483647ull ? 2147483647L : (int32_t)res;
}

static int32_t clamp_pps_i32(int32_t value)
{
    if (value < APP_MIN_EFFECTIVE_PPS) {
        return APP_MIN_EFFECTIVE_PPS;
    }
    if (value > APP_MAX_PPS_DEFAULT) {
        return APP_MAX_PPS_DEFAULT;
    }
    return value;
}

static uint16_t buffer_free(void)
{
    return (uint16_t)(APP_BINARY_TRAJ_POINTS - s_count);
}

static void clear_queue(void)
{
    uint32_t primask = irq_save();
    s_head = 0;
    s_tail = 0;
    s_count = 0;
    s_cart_line.active = false;
    irq_restore(primask);
}

static void reset_runtime_stats(void)
{
    s_max_dispatch_gap_ticks = 0;
    s_stream_underrun_ticks = 0;
    s_stream_underrun_active_ticks = 0;
    s_stream_underrun_count = 0;
    s_stream_underrun_active = false;
    s_min_buffer_count = APP_BINARY_TRAJ_POINTS;
}

static void parser_reset(void)
{
    s_parser_state = PARSER_SOF0;
    s_header_index = 0;
    s_payload_index = 0;
    s_payload_len = 0;
    s_rx_crc = 0;
    s_calc_crc = 0xFFFFu;
}

static void send_frame(uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[APP_SERIAL_TX_SIZE];
    uint16_t pos = 0;

    if (len > (uint16_t)(APP_SERIAL_TX_SIZE - 10u)) {
        len = (uint16_t)(APP_SERIAL_TX_SIZE - 10u);
    }

    frame[pos++] = BT_SOF0;
    frame[pos++] = BT_SOF1;
    frame[pos++] = APP_BINARY_TRAJ_VERSION;
    frame[pos++] = type;
    wr_u16(&frame[pos], seq);
    pos += 2u;
    wr_u16(&frame[pos], len);
    pos += 2u;
    if (len > 0u && payload != 0) {
        memcpy(&frame[pos], payload, len);
        pos += len;
    }
    uint16_t crc = crc16_buf(&frame[2], (uint16_t)(pos - 2u));
    wr_u16(&frame[pos], crc);
    pos += 2u;
    (void)SerialDma_SendBytes(frame, pos);
}

static void send_status_frame(uint16_t seq, uint8_t type)
{
    BinaryTrajSnapshot snapshot;
    uint8_t payload[32];
    BinaryTraj_GetSnapshot(&snapshot);
    payload[0] = type;
    payload[1] = BT_ERR_OK;
    wr_u16(&payload[2], snapshot.buffer_count);
    wr_u16(&payload[4], snapshot.buffer_free);
    wr_u32(&payload[6], snapshot.accepted_count);
    wr_u32(&payload[10], snapshot.executed_count);
    wr_u32(&payload[14], snapshot.total_expected);
    payload[18] = (uint8_t)snapshot.state;
    payload[19] = APP_CONTROL_HZ == 10000u ? 10u : 0u;
    wr_u32(&payload[20], snapshot.stream_underrun_ticks);
    wr_u32(&payload[24], snapshot.max_dispatch_gap_ticks);
    wr_u16(&payload[28], snapshot.min_buffer_count);
    wr_u16(&payload[30], snapshot.stream_underrun_count > 65535u ? 65535u : (uint16_t)snapshot.stream_underrun_count);
    send_frame(BT_TYPE_STATUS_RSP, seq, payload, sizeof(payload));
}

static void send_ack(uint16_t seq, uint8_t type, uint8_t err)
{
    uint8_t payload[12];
    payload[0] = type;
    payload[1] = err;
    wr_u16(&payload[2], s_count);
    wr_u16(&payload[4], buffer_free());
    wr_u32(&payload[6], s_accepted_count);
    payload[10] = (uint8_t)s_state;
    payload[11] = 0;
    send_frame(err == BT_ERR_OK ? BT_TYPE_ACK : BT_TYPE_NACK, seq, payload, sizeof(payload));
}

static void enqueue_point(const BinaryTrajPoint *point)
{
    uint32_t primask = irq_save();
    s_points[s_head] = *point;
    s_head = next_index(s_head);
    s_count++;
    s_accepted_count++;
    irq_restore(primask);
}

static bool decode_point(const uint8_t *payload, BinaryTrajPoint *point)
{
    point->p1_abs = rd_i32(payload);
    point->p2_abs = rd_i32(payload + 4);
    point->v_dom_pps = rd_u16(payload + 8);
    point->flags = rd_u16(payload + 10);
    if (point->v_dom_pps == 0u) {
        return false;
    }
    if ((point->flags & BT_POINT_FLAG_HOST_TIMED) != 0u) {
        return (point->flags & BT_POINT_FLAG_CARTESIAN_LINE) == 0u;
    }
    if (point->v_dom_pps > APP_MAX_PPS_DEFAULT) {
        return false;
    }
    return true;
}

static bool point_allowed(const BinaryTrajPoint *point)
{
    if ((point->flags & BT_POINT_FLAG_CARTESIAN_LINE) != 0u) {
        ScaraJoint joint;
        int64_t p1 = 0;
        int64_t p2 = 0;
        return ScaraKinematics_InverseUm(point->p1_abs, point->p2_abs, &joint) &&
               ScaraKinematics_JointToPulse(joint.theta1_mrad, joint.theta2_mrad, &p1, &p2) &&
               Stepper_TargetsAllowed(p1, p2);
    }
    return Stepper_TargetsAllowed(point->p1_abs, point->p2_abs);
}

static void process_frame(void)
{
    if (s_frame_ver != APP_BINARY_TRAJ_VERSION) {
        send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_VERSION);
        return;
    }

    if (s_frame_type == BT_TYPE_HELLO) {
        send_status_frame(s_frame_seq, s_frame_type);
    } else if (s_frame_type == BT_TYPE_BEGIN) {
        if (s_payload_len < 4u || Stepper_IsBusy() || s_run_requested || s_state == BINARY_TRAJ_STATE_RUNNING) {
            send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_STATE);
            return;
        }
        clear_queue();
        s_total_expected = rd_u32(s_payload);
        s_accepted_count = 0;
        s_executed_count = 0;
        s_run_requested = false;
        reset_runtime_stats();
        s_state = BINARY_TRAJ_STATE_LOADING;
        send_ack(s_frame_seq, s_frame_type, BT_ERR_OK);
    } else if (s_frame_type == BT_TYPE_CHUNK) {
        if (s_payload_len == 0u || (s_payload_len % 12u) != 0u) {
            send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_LEN);
            return;
        }
        uint16_t count = (uint16_t)(s_payload_len / 12u);
        if (count > buffer_free()) {
            send_ack(s_frame_seq, s_frame_type, BT_ERR_NO_SPACE);
            return;
        }
        for (uint16_t i = 0; i < count; ++i) {
            BinaryTrajPoint point;
            if (!decode_point(&s_payload[i * 12u], &point) || !point_allowed(&point)) {
                s_state = BINARY_TRAJ_STATE_ERROR;
                send_ack(s_frame_seq, s_frame_type, BT_ERR_MOTION);
                return;
            }
            enqueue_point(&point);
        }
        if (s_accepted_count >= s_total_expected && s_state == BINARY_TRAJ_STATE_LOADING) {
            s_state = BINARY_TRAJ_STATE_READY;
        }
        send_ack(s_frame_seq, s_frame_type, BT_ERR_OK);
    } else if (s_frame_type == BT_TYPE_VALIDATE) {
        if (s_count == 0u) {
            send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_STATE);
        } else {
            if (s_accepted_count >= s_total_expected) {
                s_state = BINARY_TRAJ_STATE_READY;
            }
            send_ack(s_frame_seq, s_frame_type, BT_ERR_OK);
        }
    } else if (s_frame_type == BT_TYPE_RUN) {
        uint32_t required_prefill = s_total_expected < APP_BINARY_TRAJ_MIN_PREFILL
                                        ? s_total_expected
                                        : APP_BINARY_TRAJ_MIN_PREFILL;
        if (s_count == 0u || s_count < required_prefill) {
            send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_STATE);
        } else {
            s_run_requested = true;
            s_min_buffer_count = s_count;
            s_state = BINARY_TRAJ_STATE_RUNNING;
            send_ack(s_frame_seq, s_frame_type, BT_ERR_OK);
        }
    } else if (s_frame_type == BT_TYPE_ABORT) {
        BinaryTraj_Stop();
        MotionPlanner_Stop();
        send_ack(s_frame_seq, s_frame_type, BT_ERR_OK);
    } else if (s_frame_type == BT_TYPE_STATUS) {
        send_status_frame(s_frame_seq, s_frame_type);
    } else {
        send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_STATE);
    }
}

void BinaryTraj_Init(void)
{
    clear_queue();
    s_total_expected = 0;
    s_accepted_count = 0;
    s_executed_count = 0;
    s_state = BINARY_TRAJ_STATE_IDLE;
    s_run_requested = false;
    reset_runtime_stats();
    parser_reset();
}

void BinaryTraj_Stop(void)
{
    clear_queue();
    s_run_requested = false;
    s_state = BINARY_TRAJ_STATE_IDLE;
}

static bool current_xy_um(int32_t *x_um, int32_t *y_um)
{
    StepperState snapshot;
    ScaraJoint joint;
    ScaraPose pose;
    Stepper_GetStateSnapshot(&snapshot);
    if (!ScaraKinematics_PulseToJoint(snapshot.axis[0].position_pulse,
                                      snapshot.axis[1].position_pulse,
                                      &joint) ||
        !ScaraKinematics_Forward(&joint, &pose)) {
        return false;
    }
    *x_um = pose.x_um;
    *y_um = pose.y_um;
    return true;
}

static bool xy_to_pulse(int32_t x_um, int32_t y_um, int64_t *p1, int64_t *p2)
{
    ScaraJoint joint;
    return ScaraKinematics_InverseUm(x_um, y_um, &joint) &&
           ScaraKinematics_JointToPulse(joint.theta1_mrad, joint.theta2_mrad, p1, p2);
}

static void discard_current_point(void)
{
    uint32_t primask = irq_save();
    s_tail = next_index(s_tail);
    s_count--;
    if (s_executed_count > 0u && s_max_dispatch_gap_ticks < 1u) {
        /* Binary dispatch follows Stepper_Tick10kHz in the same ISR cycle. */
        s_max_dispatch_gap_ticks = 1u;
    }
    s_executed_count++;
    irq_restore(primask);
}

static bool cart_line_ready_for_next(void)
{
    StepperState snapshot;
    Stepper_GetStateSnapshot(&snapshot);
    if (snapshot.axis[0].mode == STEPPER_MODE_ESTOP || snapshot.axis[1].mode == STEPPER_MODE_ESTOP) {
        return false;
    }
    if (!Stepper_IsBusy()) {
        return true;
    }
    int64_t r1 = snapshot.axis[0].remaining_pulse;
    int64_t r2 = snapshot.axis[1].remaining_pulse;
    return r1 <= 8 && r2 <= 8;
}

static bool start_cart_line_segment(int32_t x_um, int32_t y_um, int32_t distance_um, uint16_t feed_mm_min, bool final_segment)
{
    StepperState snapshot;
    int64_t p1 = 0;
    int64_t p2 = 0;
    if (!xy_to_pulse(x_um, y_um, &p1, &p2) || !Stepper_TargetsAllowed(p1, p2)) {
        return false;
    }
    Stepper_GetStateSnapshot(&snapshot);
    int64_t dp1 = p1 - snapshot.axis[0].position_pulse;
    int64_t dp2 = p2 - snapshot.axis[1].position_pulse;
    if (distance_um < 1) {
        distance_um = 1;
    }
    int32_t v1 = 0;
    int32_t v2 = 0;
    if (dp1 != 0) {
        int64_t n1 = i64_abs_local(dp1) * (int64_t)feed_mm_min * 50;
        v1 = clamp_pps_i32((int32_t)(n1 / ((int64_t)distance_um * 3)));
    }
    if (dp2 != 0) {
        int64_t n2 = i64_abs_local(dp2) * (int64_t)feed_mm_min * 50;
        v2 = clamp_pps_i32((int32_t)(n2 / ((int64_t)distance_um * 3)));
    }
    int32_t exit1 = final_segment ? 0 : v1;
    int32_t exit2 = final_segment ? 0 : v2;
    return MotionPlanner_MoveAbsBlend(p1, p2, v1, v2, exit1, exit2);
}

static bool service_cartesian_line_10khz(BinaryTrajPoint *point)
{
    if ((point->flags & BT_POINT_FLAG_CARTESIAN_LINE) == 0u) {
        return false;
    }

    if (!s_cart_line.active) {
        if (!current_xy_um(&s_cart_line.start_x_um, &s_cart_line.start_y_um)) {
            s_state = BINARY_TRAJ_STATE_ERROR;
            s_run_requested = false;
            return true;
        }
        s_cart_line.end_x_um = point->p1_abs;
        s_cart_line.end_y_um = point->p2_abs;
        int32_t dx = s_cart_line.end_x_um - s_cart_line.start_x_um;
        int32_t dy = s_cart_line.end_y_um - s_cart_line.start_y_um;
        s_cart_line.total_um = i32_isqrt_i64((int64_t)dx * dx + (int64_t)dy * dy);
        s_cart_line.target_um = 0;
        s_cart_line.feed_mm_min = point->v_dom_pps;
        s_cart_line.active = true;
        if (s_cart_line.total_um < 1) {
            s_cart_line.active = false;
            discard_current_point();
            return true;
        }
    }

    if (s_cart_line.target_um >= s_cart_line.total_um) {
        if (!Stepper_IsBusy()) {
            s_cart_line.active = false;
            discard_current_point();
        }
        return true;
    }

    if (!cart_line_ready_for_next()) {
        return true;
    }

    int32_t step_um = 500;
    int32_t next_um = s_cart_line.target_um + step_um;
    if (next_um > s_cart_line.total_um) {
        next_um = s_cart_line.total_um;
    }
    int32_t dx = s_cart_line.end_x_um - s_cart_line.start_x_um;
    int32_t dy = s_cart_line.end_y_um - s_cart_line.start_y_um;
    int32_t tx = s_cart_line.start_x_um + (int32_t)(((int64_t)dx * next_um) / s_cart_line.total_um);
    int32_t ty = s_cart_line.start_y_um + (int32_t)(((int64_t)dy * next_um) / s_cart_line.total_um);
    bool final_segment = next_um >= s_cart_line.total_um;
    if (!start_cart_line_segment(tx, ty, next_um - s_cart_line.target_um, s_cart_line.feed_mm_min, final_segment)) {
        s_state = BINARY_TRAJ_STATE_ERROR;
        s_run_requested = false;
        return true;
    }
    s_cart_line.target_um = next_um;
    return true;
}

static void update_stream_underrun_10khz(void)
{
    if (s_run_requested && s_state == BINARY_TRAJ_STATE_RUNNING) {
        /* Final drain after all points are accepted is expected, not a stream low-water event. */
        if (s_accepted_count < s_total_expected && s_count < s_min_buffer_count) {
            s_min_buffer_count = s_count;
        }
        if (s_count == 0u && s_accepted_count < s_total_expected) {
            if (!s_stream_underrun_active) {
                s_stream_underrun_active = true;
                s_stream_underrun_count++;
            }
            s_stream_underrun_ticks++;
            s_stream_underrun_active_ticks++;
            if (!Stepper_IsBusy() && s_stream_underrun_active_ticks > APP_CONTROL_HZ) {
                s_run_requested = false;
                s_state = BINARY_TRAJ_STATE_ERROR;
            }
        } else {
            s_stream_underrun_active = false;
            s_stream_underrun_active_ticks = 0u;
        }
    }
}

static void service_motion(void)
{
    BinaryTrajPoint *point = s_count > 0u ? &s_points[s_tail] : NULL;
    bool can_dispatch = point != NULL &&
                        (((point->flags & BT_POINT_FLAG_HOST_TIMED) != 0u)
                             ? Stepper_CanQueueTimedSegment()
                             : Stepper_CanAcceptMove());
    if (!s_run_requested || s_state != BINARY_TRAJ_STATE_RUNNING || s_count == 0u || !can_dispatch) {
        if (s_run_requested &&
            s_state == BINARY_TRAJ_STATE_RUNNING &&
            s_count == 0u &&
            s_accepted_count >= s_total_expected &&
            !Stepper_IsBusy()) {
            s_run_requested = false;
            s_state = BINARY_TRAJ_STATE_DONE;
        }
        return;
    }

    StepperState snapshot;
    Stepper_GetStateSnapshot(&snapshot);
    point = &s_points[s_tail];
    if ((point->flags & BT_POINT_FLAG_HOST_TIMED) != 0u) {
        if (Stepper_MoveAbsTicks(point->p1_abs, point->p2_abs, point->v_dom_pps)) {
            discard_current_point();
        } else {
            s_state = BINARY_TRAJ_STATE_ERROR;
            s_run_requested = false;
        }
        return;
    }
    if (service_cartesian_line_10khz(point)) {
        return;
    }
    int32_t d1 = point->p1_abs - (int32_t)snapshot.axis[0].position_pulse;
    int32_t d2 = point->p2_abs - (int32_t)snapshot.axis[1].position_pulse;
    uint16_t dom = abs_i32_to_u16(abs(d1) > abs(d2) ? d1 : d2);
    int32_t v1 = 0;
    int32_t v2 = 0;
    int32_t exit1 = 0;
    int32_t exit2 = 0;

    if (dom > 0u) {
        v1 = (int32_t)(((uint32_t)abs_i32_to_u16(d1) * point->v_dom_pps) / dom);
        v2 = (int32_t)(((uint32_t)abs_i32_to_u16(d2) * point->v_dom_pps) / dom);
        if (d1 != 0 && v1 < APP_MIN_EFFECTIVE_PPS) {
            v1 = APP_MIN_EFFECTIVE_PPS;
        }
        if (d2 != 0 && v2 < APP_MIN_EFFECTIVE_PPS) {
            v2 = APP_MIN_EFFECTIVE_PPS;
        }
    }

    if ((point->flags & BT_POINT_FLAG_EXACT_STOP) != 0u) {
        exit1 = 0;
        exit2 = 0;
    } else if (s_count > 1u) {
        BinaryTrajPoint *next = &s_points[next_index(s_tail)];
        int32_t nd1 = next->p1_abs - point->p1_abs;
        int32_t nd2 = next->p2_abs - point->p2_abs;
        uint16_t ndom = abs_i32_to_u16(abs(nd1) > abs(nd2) ? nd1 : nd2);
        int32_t nv1 = 0;
        int32_t nv2 = 0;
        if (ndom > 0u) {
            nv1 = (int32_t)(((uint32_t)abs_i32_to_u16(nd1) * next->v_dom_pps) / ndom);
            nv2 = (int32_t)(((uint32_t)abs_i32_to_u16(nd2) * next->v_dom_pps) / ndom);
            if (nd1 != 0 && nv1 < APP_MIN_EFFECTIVE_PPS) {
                nv1 = APP_MIN_EFFECTIVE_PPS;
            }
            if (nd2 != 0 && nv2 < APP_MIN_EFFECTIVE_PPS) {
                nv2 = APP_MIN_EFFECTIVE_PPS;
            }
        }
        if ((d1 > 0 && nd1 > 0) || (d1 < 0 && nd1 < 0)) {
            exit1 = v1 < nv1 ? v1 : nv1;
        }
        if ((d2 > 0 && nd2 > 0) || (d2 < 0 && nd2 < 0)) {
            exit2 = v2 < nv2 ? v2 : nv2;
        }
    }

    if (MotionPlanner_MoveAbsBlend(point->p1_abs, point->p2_abs, v1, v2, exit1, exit2)) {
        uint32_t primask = irq_save();
        s_tail = next_index(s_tail);
        s_count--;
        if (s_executed_count > 0u && s_max_dispatch_gap_ticks < 1u) {
            /* The next block is dispatched in the first tick after completion. */
            s_max_dispatch_gap_ticks = 1u;
        }
        s_executed_count++;
        irq_restore(primask);
    } else {
        s_state = BINARY_TRAJ_STATE_ERROR;
        s_run_requested = false;
    }
}

void BinaryTraj_Loop(void)
{
    /* GRBL-style split: prepare/dispatch segments in the main loop, never in the 10 kHz ISR. */
    service_motion();
}

void BinaryTraj_Tick10kHz(void)
{
    update_stream_underrun_10khz();
}

bool BinaryTraj_FeedByte(uint8_t byte)
{
    switch (s_parser_state) {
    case PARSER_SOF0:
        if (byte == BT_SOF0) {
            parser_reset();
            s_parser_state = PARSER_SOF1;
            return true;
        }
        return false;
    case PARSER_SOF1:
        if (byte == BT_SOF1) {
            s_parser_state = PARSER_HEADER;
            return true;
        }
        parser_reset();
        return true;
    case PARSER_HEADER:
        s_header[s_header_index++] = byte;
        s_calc_crc = crc16_update(s_calc_crc, byte);
        if (s_header_index >= sizeof(s_header)) {
            s_frame_ver = s_header[0];
            s_frame_type = s_header[1];
            s_frame_seq = rd_u16(&s_header[2]);
            s_payload_len = rd_u16(&s_header[4]);
            if (s_payload_len > APP_BINARY_TRAJ_MAX_PAYLOAD) {
                send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_LEN);
                parser_reset();
                return true;
            }
            s_payload_index = 0;
            s_parser_state = s_payload_len == 0u ? PARSER_CRC0 : PARSER_PAYLOAD;
        }
        return true;
    case PARSER_PAYLOAD:
        s_payload[s_payload_index++] = byte;
        s_calc_crc = crc16_update(s_calc_crc, byte);
        if (s_payload_index >= s_payload_len) {
            s_parser_state = PARSER_CRC0;
        }
        return true;
    case PARSER_CRC0:
        s_rx_crc = byte;
        s_parser_state = PARSER_CRC1;
        return true;
    case PARSER_CRC1:
        s_rx_crc |= (uint16_t)byte << 8;
        if (s_rx_crc == s_calc_crc) {
            process_frame();
        } else {
            send_ack(s_frame_seq, s_frame_type, BT_ERR_BAD_CRC);
        }
        parser_reset();
        return true;
    default:
        parser_reset();
        return true;
    }
}

uint16_t BinaryTraj_BufferFree(void)
{
    return buffer_free();
}

uint16_t BinaryTraj_BufferCount(void)
{
    return s_count;
}

uint32_t BinaryTraj_AcceptedCount(void)
{
    return s_accepted_count;
}

uint32_t BinaryTraj_ExecutedCount(void)
{
    return s_executed_count;
}

uint32_t BinaryTraj_StreamUnderrunTicks(void)
{
    return s_stream_underrun_ticks;
}

uint32_t BinaryTraj_StreamUnderrunCount(void)
{
    return s_stream_underrun_count;
}

uint32_t BinaryTraj_MaxDispatchGapTicks(void)
{
    return s_max_dispatch_gap_ticks;
}

uint16_t BinaryTraj_MinBufferCount(void)
{
    return s_min_buffer_count == APP_BINARY_TRAJ_POINTS ? s_count : s_min_buffer_count;
}

BinaryTrajState BinaryTraj_GetState(void)
{
    return s_state;
}

void BinaryTraj_GetSnapshot(BinaryTrajSnapshot *out)
{
    if (out == NULL) {
        return;
    }

    uint32_t primask = irq_save();
    out->buffer_count = s_count;
    out->buffer_free = (uint16_t)(APP_BINARY_TRAJ_POINTS - s_count);
    out->min_buffer_count = s_min_buffer_count == APP_BINARY_TRAJ_POINTS ? s_count : s_min_buffer_count;
    out->total_expected = s_total_expected;
    out->accepted_count = s_accepted_count;
    out->executed_count = s_executed_count;
    out->stream_underrun_ticks = s_stream_underrun_ticks;
    out->stream_underrun_count = s_stream_underrun_count;
    out->max_dispatch_gap_ticks = s_max_dispatch_gap_ticks;
    out->state = s_state;
    out->run_requested = s_run_requested;
    irq_restore(primask);
}

const char *BinaryTraj_StateName(BinaryTrajState state)
{
    switch (state) {
    case BINARY_TRAJ_STATE_IDLE: return "Idle";
    case BINARY_TRAJ_STATE_LOADING: return "Loading";
    case BINARY_TRAJ_STATE_READY: return "Ready";
    case BINARY_TRAJ_STATE_RUNNING: return "Running";
    case BINARY_TRAJ_STATE_DONE: return "Done";
    case BINARY_TRAJ_STATE_ERROR: return "Error";
    default: return "Unknown";
    }
}
