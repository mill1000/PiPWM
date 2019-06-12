#ifndef __PIPWM__
#define __PIPWM__

#include "bcm2837_pwm.h"
#include "bcm2837_dma.h"
#include "bcm2837_clock.h"
#include "bcm2837_gpio.h"
#include "bcm2837.h"
#include "memory.h"

typedef struct pipwm_control_t
{
  dma_control_block_t controlBlocks[4];
  gpio_pin_mask_t     setMask;
  gpio_pin_mask_t     clearMask;
} pipwm_control_t;

typedef struct pipwm_channel_t
{
  dma_channel_t     dmaChannel;
  gpio_pin_mask_t   pinMask;
  memory_physical_t memory;
  pipwm_control_t*  vControl;
  uint32_t          steps;
} pipwm_channel_t;

void piPwm_initialize(uint16_t divi, uint16_t divf, uint16_t range);
void piPwm_shutdown(void);
pipwm_channel_t* piPwm_initalizeChannel(dma_channel_t dmaChannel, gpio_pin_mask_t pinMask, double frequency_Hz);
void piPwm_freeChannel(pipwm_channel_t* channel);
void piPwm_enableChannel(const pipwm_channel_t* channel, bool enable);
void piPwm_setRatio(pipwm_channel_t* channel, double ratio);
#endif