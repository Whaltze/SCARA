#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

void App_Init(void);
void App_Loop(void);
void App_StepEventIrq(void);
void App_Tick1kHz(void);
uint32_t App_MaxTickCycles(void);

#endif
