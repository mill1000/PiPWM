#ifndef __BCM2837__
#define __BCM2837__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

// Use GCC internal __sync functions until we know how to do this better
#define WMB() do {__sync_synchronize();} while (0)
#define RMB() WMB()

#endif