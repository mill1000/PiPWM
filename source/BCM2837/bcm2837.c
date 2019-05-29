#include <stddef.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <bcm_host.h>

#include <bcm2837.h>
#include <bcm2837_pwm.h>

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
    // LOG the mesage
    return NULL;
  }

  // Map the physical memory (via /dev/mem) located at offset into our virtual memory
  void* virtual = mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, file, offset);
  if (virtual == MAP_FAILED)
  {
    // LOG the failure
    return NULL;
  }

  int32_t result = close(file);
  if (result == -1)
  {
    // LOG the error
  }

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
    // LOG the error
    // Already initalized
    return;
  }

  // Fetch physical address and length of peripherals for our system
  off_t physicalBase = bcm_host_get_peripheral_address();
  size_t length = bcm_host_get_peripheral_size();

  // Map to virtual memory
  virtualBase = mapPhysicalMemory(physicalBase, length);
  if (virtualBase == NULL)
  {
    // LOG the failure
    return;
  }

  // Initalize the PWM module at it's base address
  pwmInit(virtualBase + PWM_BASE_OFFSET);
}
