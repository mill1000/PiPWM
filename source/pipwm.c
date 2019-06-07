#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include <bcm_host.h>

#include "bcm2837_pwm.h"
#include "bcm2837_dma.h"
#include "bcm2837_clock.h"
#include "bcm2837_gpio.h"
#include "bcm2837.h"
#include "log.h"
#include "memory.h"
#include "utils.h"

#define TAG "MAIN"

void main()
{
  bcm2837_init();

  gpio_configuration_t gpioConfig;
  gpioConfig.eventDetect = gpio_event_detect_none;
  gpioConfig.function = gpio_function_af0;
  gpioConfig.pull = gpio_pull_no_change;
  gpioConfigure(12, &gpioConfig);
  
  clock_configuration_t clockConfig;
  clockConfig.source = CLOCK_SOURCE_OSCILLATOR;
  clockConfig.mash = CLOCK_MASH_NONE;
  clockConfig.invert = false;
  clockConfig.divi = 300;
  clockConfig.divf = 0;

  LOGI(TAG, "Configuring clock");
  clockConfigure(clock_peripheral_pwm, &clockConfig);

  LOGI(TAG, "Enabling clock");
  clockEnable(clock_peripheral_pwm, true);

  pwmReset();

  pwm_configuration_t pwmConfig;
  pwmConfig.mode = pwm_mode_mark_space;
  pwmConfig.fifoEnable = true;
  pwmConfig.repeatLast = false;
  pwmConfig.invert = false;
  pwmConfig.silenceBit = 0;

  LOGI(TAG, "Configuring PWM");
  pwmConfigure(pwm_channel_1, &pwmConfig);
  pwmSetRange(pwm_channel_1, 1 << 10);

  LOGI(TAG, "Enabling PWM");
  pwmEnable(pwm_channel_1, true);

  //pwmSetData(pwm_channel_1, 256);
  //return;

  

  // Not wokring reliable. Allocation -> physical -> remapping
  // Transfers do occur occasionally, but sometimes source data is missing as well
  //void* virtual2 = memoryAllocate(4096);
  ////if (posix_memalign(&virtual2, 4096, 4096) != 0)
  ////    LOGE(TAG, "Memalign %s", strerror(errno));
  //((uint8_t*)virtual2)[0] = 1;
  //if (mlock(virtual2, 4096) != 0)
  //  LOGI(TAG, "lock %s", strerror(errno));
  //void* physical = memoryVirtualToPhysical(virtual2);
  //volatile void* virtual = memoryMapPhysical((off_t)physical, 4096);
  //void* bus = physical +  bcm_host_get_sdram_address();

  // udmabuf provided buffer
  //void* physical = (void*)0x3ad0a000;//memoryVirtualToPhysical(virtual);
  //void* virtual = memoryMapPhysical((off_t)physical, 4096);
  //void* bus = physical +  bcm_host_get_sdram_address();


  // Confirmed working on mem to mem transfer
  // Dump function does not always display proper data
  memory_physical_t memory = memoryAllocatePhysical(4096);
  if (memory.address == NULL)
    LOGF(TAG, "Failed to allocate physical memory.");
	
  void* bus = memory.address;
  void* physical = bus - bcm_host_get_sdram_address();
  void* virtual = memoryMapPhysical((off_t)physical, 4096);

  assert(physical != NULL);
  assert(bus != NULL);
  assert(virtual != NULL);

  LOGI(TAG, "Physical addr %X, Bus addr %X, Virtual %X", physical, bus, virtual);
  bcm2837_pwm_t* pwmBus = (bcm2837_pwm_t*) (0x7e000000 + PWM_BASE_OFFSET);
  

  uint32_t* data = virtual + 2048;
  data[0] = 100;
  data[1] = 500;
  data[2] = 300;
  data[3] = 400;
  data[4] = 500;
  data[5] = 600;
  data[6] = 700;
  data[7] = 800;
  data[8] = 900;
  data[9] = 1000;//getpid();

  //uint32_t* data2 = virtual + 3092;
  //data2[0] = 0;
  //data2[1] = 0;
  //data2[2] = 0;
  //data2[3] = 0;
  //data2[4] = 0;
  //data2[5] = 0;
  //data2[6] = 0;
  //data2[7] = 0;
  //data2[8] = 0;
  //data2[9] = 0;

  //for (uint8_t i = 0; i < 10; i++)
   // LOGI(TAG, "Src %d, Dst %d", data[i], data2[i]);
  
  dma_control_block_t* control = (dma_control_block_t*) virtual;
  memset((void*)control, 0, sizeof(dma_control_block_t));

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.SRC_DREQ = 1;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;
  control->transferInformation.DEST_INC = 0;
  control->transferInformation.INTEN = 0;

  control->sourceAddress = bus + 2048;
  control->destinationAddress = (void*) &pwmBus->FIF1;
  control->transferLength.XLENGTH = 10 * sizeof(uint32_t);
  
  control->nextControlBlock = (dma_control_block_t*) bus;

  dmaDumpControlBlock(control);

  // Reset target channel
  dmaReset(dma_channel_13);

  // Configure and enable to begin transfer
  dmaSetControlBlock(dma_channel_13, (void*) bus);
  dmaEnable(dma_channel_13, true);

  dmaDump(dma_channel_13);  

  pwmConfigureDma(true, 7,7);

  // Let DMA function
  sleep(1);
  
  dmaDump(dma_channel_13);  
  

  //for (uint8_t i = 0; i < 10; i++)
      //LOGI(TAG, "Src %d, Dst %d", data[i], data2[i]);

  dmaReset(dma_channel_13);
  pwmReset();

  memoryReleasePhysical(&memory);
  
  return;
}