#ifndef SERIAL_DMA_H
#define SERIAL_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void SerialDma_Init(void);
void SerialDma_Poll(void);
bool SerialDma_ReadLine(char *out, size_t out_size);
bool SerialDma_Send(const char *text);
bool SerialDma_SendBytes(const uint8_t *data, uint16_t len);
bool SerialDma_SendFormat(const char *fmt, ...);
bool SerialDma_IsTxBusy(void);
uint32_t SerialDma_RxOverflowCount(void);
uint32_t SerialDma_RxFreeCount(void);
uint32_t SerialDma_RxFreeBytes(void);
uint32_t SerialDma_TxDropCount(void);
uint32_t SerialDma_TxQueuedCount(void);
void SerialDma_TxCpltCallback(void);

#endif
