#include <stddef.h>
#include <assert.h>

#include <bcm_host.h>

#include "bcm2837.h"
#include "bcm2837_pwm.h"
#include "bcm2837_clock.h"
#include "bcm2837_gpio.h"
#include "bcm2837_dma.h"
#include "log.h"
#include "memory.h"

#define TAG "BCM2837"

/**
  @brief  Initalize BCM2837 peripheral modules

  @param  none
  @retval none
*/
void bcm2837_init()
{
  static void* virtualBase = NULL;

  // Don't initalize twice
  if (virtualBase != NULL)
  {
    LOGW(TAG, "Already initialized.");
    return;
  }

  // Fetch physical address and length of peripherals for our system
  off_t physicalBase = bcm_host_get_peripheral_address();
  size_t length = bcm_host_get_peripheral_size();

  // Map to virtual memory
  virtualBase = memoryMapPhysical(physicalBase, length);
  if (virtualBase == NULL)
  {
    LOGF(TAG, "Failed to map physical memory.");
    return;
  }

  // Initalize modules at their base addresses
  pwmInit(virtualBase + PWM_BASE_OFFSET);
  clockInit(virtualBase + CLOCK_BASE_OFFSET);
  gpioInit(virtualBase + GPIO_BASE_OFFSET);
  dmaInit(virtualBase + DMA_BASE_OFFSET);
}
