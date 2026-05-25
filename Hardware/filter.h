#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>
#include <string.h>

/* 滑动窗口长度，可根据实际滤波需求调整 */
#define FILTER_SIZE  10

/**
 * @brief 滑动平均滤波器实例结构体
 */
typedef struct {
    float buffer[FILTER_SIZE];
    uint16_t index;
    float sum;
    uint8_t is_full;
} SlidingAvgFilter;

/**
 * @brief  Initialize sliding window average filter
 * @param  f: filter instance pointer
 */
void Filter_Init(SlidingAvgFilter *f);

/**
 * @brief  Update filter with new sample value (sliding window average)
 * @param  f: filter instance pointer
 * @param  input: new raw sample
 * @retval filtered output value (window average)
 */
float Filter_Update(SlidingAvgFilter *f, float input);

#endif /* __FILTER_H__ */
