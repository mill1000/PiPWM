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
#include "mailbox.h"

#define TAG "MAIN"

void main()
{
  LOGI(TAG, "Channels %X", mailboxGetDmaChannelMask());

  bcm2837_init();

  gpio_configuration_t gpioConfig;
  gpioConfig.eventDetect = gpio_event_detect_none;
  gpioConfig.function = gpio_function_output;
  gpioConfig.pull = gpio_pull_no_change;
  gpioConfigure(12, &gpioConfig);
  
  clock_configuration_t clockConfig;
  clockConfig.source = CLOCK_SOURCE_OSCILLATOR;
  clockConfig.mash = CLOCK_MASH_NONE;
  clockConfig.invert = false;
  clockConfig.divi = 192;
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
  pwmSetRange(pwm_channel_1, 1000);

  

  //pwmSetData(pwm_channel_1, 0);

  gpioClear(12);

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
  bcm2837_gpio_t* gpioBus = (bcm2837_gpio_t*) (0x7e000000 + GPIO_BASE_OFFSET);
  
  uint32_t* vMask = virtual + 2048;
  uint32_t* bMask = bus + 2048;
  
  *vMask = 1 << 12;

  dma_control_block_t* vBlocks = (dma_control_block_t*) virtual;
  dma_control_block_t* bBlocks = (dma_control_block_t*) bus;

  dma_control_block_t* control = &vBlocks[0];
  memset((void*)control, 0, sizeof(dma_control_block_t));

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = 0;
  control->transferInformation.SRC_DREQ = 0;
  control->transferInformation.DEST_DREQ = 0;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;
  control->transferInformation.DEST_INC = 0;
  control->transferInformation.INTEN = 0;

  control->sourceAddress = bMask;
  control->destinationAddress = (void*) &gpioBus->GPSETx[0].SET;
  control->transferLength.XLENGTH = 1 * sizeof(uint32_t);

  control->nextControlBlock = (dma_control_block_t*)  &bBlocks[1];

  control = &vBlocks[1];
  memset((void*)control, 0, sizeof(dma_control_block_t));

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.SRC_DREQ = 1;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 0;
  control->transferInformation.DEST_INC = 0;
  control->transferInformation.INTEN = 0;

  control->sourceAddress = bMask;
  control->destinationAddress = (void*) &pwmBus->FIF1;
  control->transferLength.XLENGTH = 1 * sizeof(uint32_t);

  control->nextControlBlock = (dma_control_block_t*) &bBlocks[2];

  control = &vBlocks[2];
  memset((void*)control, 0, sizeof(dma_control_block_t));

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = 0;
  control->transferInformation.SRC_DREQ = 0;
  control->transferInformation.DEST_DREQ = 0;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;
  control->transferInformation.DEST_INC = 0;
  control->transferInformation.INTEN = 0;

  control->sourceAddress = bMask;
  control->destinationAddress = (void*) &gpioBus->GPCLRx[0].CLR;
  control->transferLength.XLENGTH = 1 * sizeof(uint32_t);

  control->nextControlBlock = (dma_control_block_t*)  &bBlocks[3];

  control = &vBlocks[3];
  memset((void*)control, 0, sizeof(dma_control_block_t));

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.SRC_DREQ = 1;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 0;
  control->transferInformation.DEST_INC = 0;
  control->transferInformation.INTEN = 0;

  control->sourceAddress = bMask;
  control->destinationAddress = (void*) &pwmBus->FIF1;
  control->transferLength.XLENGTH = 1 * sizeof(uint32_t);

  control->nextControlBlock = (dma_control_block_t*) &bBlocks[0];

  dmaDumpControlBlock(&vBlocks[0]);
  dmaDumpControlBlock(&vBlocks[1]);
  dmaDumpControlBlock(&vBlocks[2]);

  // Reset target channel
  dmaReset(dma_channel_5);

  //dmaDump(dma_channel_5);  

  // Configure and enable to begin transfer
  dmaSetControlBlock(dma_channel_5, (void*) bBlocks);
  dmaEnable(dma_channel_5, true);

  //dmaDump(dma_channel_5);  
  LOGI(TAG, "Enabling PWM");
  pwmConfigureDma(true, 15,15);
  pwmEnable(pwm_channel_1, true);

  // Let DMA function
  sleep(2);

  dmaDump(dma_channel_5);  

  //for (uint8_t i = 0; i < 10; i++)
//    LOGI(TAG, "Src %d, Dst %d", data[i], data2[i]);

  dmaReset(dma_channel_5);
  pwmReset();

  memoryReleasePhysical(&memory);
  gpioClear(12);

  return;
}