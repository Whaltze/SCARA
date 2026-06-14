#ifndef GCODE_STREAM_H
#define GCODE_STREAM_H

#include <stdbool.h>
#include <stdint.h>

void GcodeStream_Init(void);
void GcodeStream_Loop(void);
void GcodeStream_Tick1kHz(void);
bool GcodeStream_TryProcessLine(const char *line);
void GcodeStream_RequestStatus(void);
void GcodeStream_Clear(void);
uint8_t GcodeStream_PlannerFree(void);
uint8_t GcodeStream_PlannerCount(void);
bool GcodeStream_HomeAckPending(void);

#endif
