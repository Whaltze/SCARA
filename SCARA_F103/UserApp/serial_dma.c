#include "serial_dma.h"

/* USART1 DMA 串口层：接收字节流、切成行队列，并用 TX 队列避免响应丢失。 */

#include "app_config.h"
#include "board_pins.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_rx_dma[APP_SERIAL_RX_DMA_SIZE];
static volatile uint16_t s_rx_old_pos;
static char s_rx_queue[APP_SERIAL_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint16_t s_rx_count;
static char s_realtime_queue[APP_SERIAL_REALTIME_QUEUE_DEPTH];
static volatile uint8_t s_realtime_head;
static volatile uint8_t s_realtime_tail;
static volatile uint8_t s_realtime_count;
static volatile uint32_t s_rx_overflow_count;
static char s_tx_queue[APP_SERIAL_TX_QUEUE_DEPTH][APP_SERIAL_TX_SIZE];
static uint16_t s_tx_len[APP_SERIAL_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile uint8_t s_tx_count;
static volatile bool s_tx_busy;
static volatile uint32_t s_tx_drop_count;

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

static uint8_t queue_next(uint8_t index, uint8_t depth)
{
    index++;
    if (index >= depth) {
        index = 0;
    }
    return index;
}

static uint16_t rx_next(uint16_t index)
{
    index++;
    return index >= APP_SERIAL_RX_RING_SIZE ? 0u : index;
}

static bool enqueue_rx_byte(char byte)
{
    bool queued = false;
    uint32_t primask = irq_save();
    if (s_rx_count < APP_SERIAL_RX_RING_SIZE) {
        s_rx_queue[s_rx_head] = byte;
        s_rx_head = rx_next(s_rx_head);
        s_rx_count++;
        queued = true;
    } else {
        s_rx_overflow_count++;
    }
    irq_restore(primask);
    return queued;
}

static bool enqueue_realtime(char ch)
{
    bool queued = false;
    uint32_t primask = irq_save();

    if (s_realtime_count < APP_SERIAL_REALTIME_QUEUE_DEPTH) {
        s_realtime_queue[s_realtime_head] = ch;
        s_realtime_head = queue_next(s_realtime_head, APP_SERIAL_REALTIME_QUEUE_DEPTH);
        s_realtime_count++;
        queued = true;
    } else {
        s_rx_overflow_count++;
    }

    irq_restore(primask);
    return queued;
}

static bool tx_start_next_locked(void)
{
    if (s_tx_busy || s_tx_count == 0u) {
        return true;
    }

    s_tx_busy = true;
    if (HAL_UART_Transmit_DMA(BOARD_UART,
                              (uint8_t *)s_tx_queue[s_tx_tail],
                              s_tx_len[s_tx_tail]) != HAL_OK) {
        s_tx_busy = false;
        s_tx_tail = queue_next(s_tx_tail, APP_SERIAL_TX_QUEUE_DEPTH);
        s_tx_count--;
        s_tx_drop_count++;
        return false;
    }
    return true;
}

void SerialDma_Init(void)
{
    s_rx_old_pos = 0;
    s_rx_head = 0;
    s_rx_tail = 0;
    s_rx_count = 0;
    s_realtime_head = 0;
    s_realtime_tail = 0;
    s_realtime_count = 0;
    s_rx_overflow_count = 0;
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_count = 0;
    s_tx_busy = false;
    s_tx_drop_count = 0;
    HAL_UART_Receive_DMA(BOARD_UART, s_rx_dma, APP_SERIAL_RX_DMA_SIZE);
}

static void feed_char(char ch)
{
    /* 实时字符 ?/!/~/Ctrl-X 单独成行；普通命令以换行结束。 */
    if (ch == '?' || ch == '!' || ch == '~' || (uint8_t)ch == 0x18u || (uint8_t)ch == 0x85u) {
        (void)enqueue_realtime(ch);
        return;
    }

    if (ch != '\r') {
        (void)enqueue_rx_byte(ch);
    }
}

void SerialDma_Poll(void)
{
    /* 主循环轮询 DMA 写入位置，把新增字节投递给行解析器。 */
    uint16_t pos = (uint16_t)(APP_SERIAL_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(BOARD_UART->hdmarx));

    while (s_rx_old_pos != pos) {
        uint8_t byte = s_rx_dma[s_rx_old_pos];
        feed_char((char)byte);
        s_rx_old_pos++;
        if (s_rx_old_pos >= APP_SERIAL_RX_DMA_SIZE) {
            s_rx_old_pos = 0;
        }
    }
}

bool SerialDma_ReadRealtime(char *out)
{
    if (out == NULL) {
        return false;
    }

    uint32_t primask = irq_save();
    if (s_realtime_count == 0u) {
        irq_restore(primask);
        return false;
    }

    *out = s_realtime_queue[s_realtime_tail];
    s_realtime_tail = queue_next(s_realtime_tail, APP_SERIAL_REALTIME_QUEUE_DEPTH);
    s_realtime_count--;
    irq_restore(primask);
    return true;
}

void SerialDma_FlushRxLines(void)
{
    /*
     * Discard buffered command bytes on soft reset / abort. Without this the tail
     * of an aborted stream still sitting in the DMA buffer and line ring is parsed
     * after the reset, re-planned, and can re-latch a preparation fault so the
     * controller never reaches the Idle/Q:0/Seg:0 state the host waits for. Realtime
     * characters (?/!/~) are intentionally left untouched. Mirrors Grbl's soft-reset
     * flushing of the serial RX buffer.
     */
    uint32_t primask = irq_save();
    s_rx_old_pos = (uint16_t)(APP_SERIAL_RX_DMA_SIZE - __HAL_DMA_GET_COUNTER(BOARD_UART->hdmarx));
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_count = 0u;
    irq_restore(primask);
}

bool SerialDma_ReadLine(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0u) {
        return false;
    }

    uint32_t primask = irq_save();
    if (s_rx_count == 0u) {
        irq_restore(primask);
        return false;
    }
    uint16_t scan = s_rx_tail;
    uint16_t available = s_rx_count;
    bool complete = false;
    for (uint16_t i = 0u; i < available; ++i) {
        if (s_rx_queue[scan] == '\n') {
            complete = true;
            break;
        }
        scan = rx_next(scan);
    }
    if (!complete) {
        irq_restore(primask);
        return false;
    }
    size_t length = 0u;
    while (s_rx_count > 0u) {
        char ch = s_rx_queue[s_rx_tail];
        s_rx_tail = rx_next(s_rx_tail);
        s_rx_count--;
        if (ch == '\n') {
            break;
        }
        if (length + 1u < out_size) {
            out[length++] = ch;
        }
    }
    out[length] = '\0';
    irq_restore(primask);
    return length > 0u;
}

bool SerialDma_Send(const char *text)
{
    if (text == NULL) {
        return false;
    }

    return SerialDma_SendBytes((const uint8_t *)text, (uint16_t)strlen(text));
}

bool SerialDma_SendBytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return false;
    }

    if (len == 0u) {
        return true;
    }
    if (len >= APP_SERIAL_TX_SIZE) {
        len = APP_SERIAL_TX_SIZE - 1u;
    }

    uint32_t primask = irq_save();
    if (s_tx_count >= APP_SERIAL_TX_QUEUE_DEPTH) {
        s_tx_drop_count++;
        irq_restore(primask);
        return false;
    }

    memcpy(s_tx_queue[s_tx_head], data, len);
    s_tx_len[s_tx_head] = len;
    s_tx_head = queue_next(s_tx_head, APP_SERIAL_TX_QUEUE_DEPTH);
    s_tx_count++;
    bool ok = tx_start_next_locked();
    irq_restore(primask);
    return ok;
}

bool SerialDma_SendFormat(const char *fmt, ...)
{
    if (fmt == NULL) {
        return false;
    }

    char formatted[APP_SERIAL_TX_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(formatted, sizeof(formatted), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return false;
    }
    return SerialDma_Send(formatted);
}

bool SerialDma_IsTxBusy(void)
{
    return s_tx_count > 0u || s_tx_busy;
}

uint32_t SerialDma_RxOverflowCount(void)
{
    return s_rx_overflow_count;
}

uint32_t SerialDma_RxFreeCount(void)
{
    return (uint32_t)(APP_SERIAL_RX_RING_SIZE - s_rx_count);
}

uint32_t SerialDma_RxFreeBytes(void)
{
    return SerialDma_RxFreeCount();
}

uint32_t SerialDma_TxDropCount(void)
{
    return s_tx_drop_count;
}

uint32_t SerialDma_TxQueuedCount(void)
{
    return s_tx_count;
}

uint32_t SerialDma_TxFreeCount(void)
{
    return (uint32_t)(APP_SERIAL_TX_QUEUE_DEPTH - s_tx_count);
}

void SerialDma_TxCpltCallback(void)
{
    if (s_tx_count > 0u) {
        s_tx_tail = queue_next(s_tx_tail, APP_SERIAL_TX_QUEUE_DEPTH);
        s_tx_count--;
    }
    s_tx_busy = false;
    (void)tx_start_next_locked();
}
