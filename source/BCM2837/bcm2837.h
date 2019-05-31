#ifndef __BCM2837__
#define __BCM2837__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

// Enable GPIOs 32 - 54
//#define BCM2837_EXTENDED_GPIO

// Use GCC internal __sync functions until we know how to do this better
#define WMB() do {__sync_synchronize();} while (0)
#define RMB() WMB()

void bcm2837_init(void);

#endif