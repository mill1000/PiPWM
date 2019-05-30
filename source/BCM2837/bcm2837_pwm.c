#include <stddef.h>
#include <assert.h>

#include <bcm2837_pwm.h>

static bcm2837_pwm_t* pwm = NULL;

/**
  @brief  Initalize the PWM object at the given base address

  @param  base Base address of PWM peripheral
  @retval none
*/
void pwmInit(void* base)
{
  assert(base != NULL);
  assert(pwm == NULL);

  pwm = base;
}

/**
  @brief  Set the range of the target PWM channel

  @param  channel PWM channel
  @param  range Range value to set
  @retval none
*/
void pwmSetRange(pwm_channel_t channel, uint32_t range)
{
  assert(pwm != NULL);
  assert(channel < pwm_channel_max);
  
  WMB();
  
  switch (channel)
  {
    case pwm_channel_1:
      pwm->RNG1 = range;
      break;

    case pwm_channel_2:
      pwm->RNG2 = range;
      break;

    default:
      break;
  }
}

/**
  @brief  Set the data of the target PWM channel

  @param  channel PWM channel
  @param  data Data value to set
  @retval none
*/
void pwmSetData(pwm_channel_t channel, uint32_t data)
{
  assert(pwm != NULL);
  assert(channel < pwm_channel_max);

  WMB();

  switch (channel)
  {
    case pwm_channel_1:
      pwm->DAT1 = data;
      break;

    case pwm_channel_2:
      pwm->DAT2 = data;
      break;

    default:
      break;
  }
}

/**
  @brief  Configure the PWM DMA settings

  @param  enable Enable DMA requests from the PWM peripheral
  @param  panicThreshold Threshold for PANIC signal
  @param  dreqThreshold Threshold for DREQ signal
  @retval none
*/
void pwmConfigureDma(bool enable, uint32_t panicThreshold, uint32_t dreqThreshold)
{
  assert(pwm != NULL);

  WMB();

  pwm->DMAC.DREQ = dreqThreshold;  
  pwm->DMAC.PANIC = panicThreshold;
  pwm->DMAC.ENAB = enable;
}

/**
  @brief  Reset the PWM peripheral

  @param  none
  @retval none
*/
void pwmReset()
{
  assert(pwm != NULL);

  WMB();

  pwm->CTL = (pwm_control_t) {0};

  pwm->DMAC.ENAB = 0;
  pwm->DMAC.PANIC = 0x7;
  pwm->DMAC.DREQ = 0x7;
}


