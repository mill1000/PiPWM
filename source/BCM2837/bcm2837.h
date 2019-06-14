#ifndef __BCM2837__
#define __BCM2837__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

// Enable GPIOs 32 - 54
//#define BCM2837_EXTENDED_GPIO

#define BCM2837_BUS_PERIPHERAL_BASE (0x7E000000)

// Use GCC internal __sync functions until we know how to do this better
#define WMB() do {__sync_synchronize();} while (0)
#define RMB() WMB()

void bcm2837_init(void);
void bcm2837_delay_microseconds(uint32_t microseconds);

#endif