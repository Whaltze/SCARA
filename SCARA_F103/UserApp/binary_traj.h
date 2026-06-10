#ifndef BINARY_TRAJ_H
#define BINARY_TRAJ_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BINARY_TRAJ_STATE_IDLE = 0,
    BINARY_TRAJ_STATE_LOADING,
    BINARY_TRAJ_STATE_READY,
    BINARY_TRAJ_STATE_RUNNING,
    BINARY_TRAJ_STATE_DONE,
    BINARY_TRAJ_STATE_ERROR
} BinaryTrajState;

typedef struct {
    uint16_t buffer_count;
    uint16_t buffer_free;
    uint16_t min_buffer_count;
    uint32_t total_expected;
    uint32_t accepted_count;
    uint32_t executed_count;
    uint32_t stream_underrun_ticks;
    uint32_t stream_underrun_count;
    uint32_t max_dispatch_gap_ticks;
    BinaryTrajState state;
    bool run_requested;
} BinaryTrajSnapshot;

void BinaryTraj_Init(void);
void BinaryTraj_Loop(void);
void BinaryTraj_Tick10kHz(void);
bool BinaryTraj_FeedByte(uint8_t byte);
void BinaryTraj_Stop(void);
uint16_t BinaryTraj_BufferFree(void);
uint16_t BinaryTraj_BufferCount(void);
uint32_t BinaryTraj_AcceptedCount(void);
uint32_t BinaryTraj_ExecutedCount(void);
uint32_t BinaryTraj_StreamUnderrunTicks(void);
uint32_t BinaryTraj_StreamUnderrunCount(void);
uint32_t BinaryTraj_MaxDispatchGapTicks(void);
uint16_t BinaryTraj_MinBufferCount(void);
BinaryTrajState BinaryTraj_GetState(void);
void BinaryTraj_GetSnapshot(BinaryTrajSnapshot *out);
const char *BinaryTraj_StateName(BinaryTrajState state);

#endif
