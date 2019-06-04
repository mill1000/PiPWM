#include <stddef.h>
#include <assert.h>

#include "bcm2837_dma.h"
#include "log.h"

static void* dma = NULL;

/**
  @brief  Initalize the DMA object at the given base address

  @param  base Base address of DMA peripheral
  @retval none
*/
void dmaInit(void* base)
{
  assert(base != NULL);
  assert(dma == NULL);

  dma = base;
}

/**
  @brief  Fetch the channel registers for the supplied channel index.

  @param  channel DMA channel number
  @retval bcm2837_dma_channel_t*
*/
static bcm2837_dma_channel_t* dmaGetChannel(dma_channel_t channel)
{
  assert(dma != NULL);
  assert(channel < dma_channel_max);

  return (bcm2837_dma_channel_t*) (dma + (channel * DMA_CHANNEL_OFFSET));
}

/**
  @brief  Reset the target DMA channel

  @param  channel DMA channel number
  @retval void
*/
void dmaReset(dma_channel_t channel)
{
  bcm2837_dma_channel_t* handle = dmaGetChannel(channel);

  WMB();
  handle->CS.RESET = 1;

  // Clear interupt and end status flags
  handle->CS.INT = 1;
  handle->CS.END = 1;

  // Clear error flags in debug register
  handle->DEBUG.READ_ERROR = 1;
  handle->DEBUG.FIFO_ERROR = 1;
  handle->DEBUG.READ_LAST_NOT_SET_ERROR = 1;
}

/**
  @brief  Set the control block for the selected DMA channel

  @param  channel DMA channel number
  @param  control DMA control block to set on channel
  @retval void
*/
void dmaSetControlBlock(dma_channel_t channel, const dma_control_block_t* control)
{
  // Ensure block is 256 byte aligned
  assert(((uint32_t)control & 0x1F) == 0);

  bcm2837_dma_channel_t* handle = dmaGetChannel(channel);

  WMB();

  handle->CONBLK_AD = (dma_control_block_t*) control;
}

/**
  @brief  Enable/disable select DMA channel

  @param  channel DMA channel number
  @param  enable Enable the channel
  @retval void
*/
void dmaEnable(dma_channel_t channel, bool enable)
{
  bcm2837_dma_channel_t* handle = dmaGetChannel(channel);

  WMB();

  handle->CS.ACTIVE = enable;
}

void dmaDump(dma_channel_t channel)
{
  volatile uint32_t* handle = (volatile uint32_t*) dmaGetChannel(channel);

  LOGI("DMA", "Channel %d", channel);
  LOGI("DMA", "CS\t\t0x%08X", handle[0]);
  LOGI("DMA", "CONBLK_AD\t0x%08X", handle[1]);
  LOGI("DMA", "TI\t\t0x%08X", handle[2]);
  LOGI("DMA", "SOURCE_AD\t0x%08X", handle[3]);
  LOGI("DMA", "DEST_AD\t\t0x%08X", handle[4]);
  LOGI("DMA", "TXFR_LEN\t0x%08X", handle[5]);
  LOGI("DMA", "STRIDE\t\t0x%08X", handle[6]);
  LOGI("DMA", "NEXTCONBK\t0x%08X", handle[7]);
  LOGI("DMA", "DEBUG\t\t0x%08X", handle[8]);

  RMB();
}

void dmaDumpControlBlock(const dma_control_block_t* control)
{
  const volatile uint32_t* data = (volatile uint32_t*) control;

  LOGI("DMA", "Control Block %X", control);
  LOGI("DMA", "TI\t\t0x%08X", data[0]);
  LOGI("DMA", "SOURCE_AD\t0x%08X", data[1]);
  LOGI("DMA", "DEST_AD\t\t0x%08X", data[2]);
  LOGI("DMA", "TXFR_LEN\t0x%08X", data[3]);
  LOGI("DMA", "STRIDE\t\t0x%08X", data[4]);
  LOGI("DMA", "NEXTCONBK\t0x%08X", data[5]);
  LOGI("DMA", "R1\t\t0x%08X", data[6]);
  LOGI("DMA", "R2\t\t0x%08X", data[7]);

  RMB();
}