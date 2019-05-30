#include <stddef.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <bcm_host.h>

#include "bcm2837.h"
#include "bcm2837_pwm.h"
#include "bcm2837_clock.h"
#include "bcm2837_gpio.h"
#include "log.h"

#define TAG "BCM2837"

/**
  @brief  Map physical memory located at offset into the virtual memory space

  @param  offset Offset in physical memory
  @param  length Length of memory to map
  @retval void* - Address of mapped memory in virtual space
*/
static void* mapPhysicalMemory(off_t offset, size_t length)
{
  int32_t file = open("/dev/mem", O_RDWR);
  if (file == -1)
  {
    LOGF(TAG, "Failed to open /dev/mem. Error: %s", strerror(errno));
    return NULL;
  }

  // Map the physical memory (via /dev/mem) located at offset into our virtual memory
  void* virtual = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, file, offset);
  if (virtual == MAP_FAILED)
  {
    LOGF(TAG, "Failed to map physical address 0x%X of length %d. Error: %s", offset, length, strerror(errno));
    return NULL;
  }

  int32_t result = close(file);
  if (result == -1)
    LOGE(TAG, "Failed to close /dev/mem. Error: %s", strerror(errno));

  LOGI(TAG, "Mapped physical address 0x%X to virtual address 0x%X", offset, virtual);

  return virtual;
}

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
  virtualBase = mapPhysicalMemory(physicalBase, length);
  if (virtualBase == NULL)
  {
    LOGF(TAG, "Failed to map physical memory.");
    return;
  }

  // Initalize PWM module at it's base address
  pwmInit(virtualBase + PWM_BASE_OFFSET);

  // Initlaize clock module at it's base address
  clockInit(virtualBase + CLOCK_BASE_OFFSET);

  // Initlaize clock module at it's base address
  gpioInit(virtualBase + GPIO_BASE_OFFSET);
}
