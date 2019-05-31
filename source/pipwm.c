#include "bcm2837_pwm.h"
#include "bcm2837_dma.h"
#include "bcm2837_clock.h"
#include "bcm2837.h"
#include "log.h"
#include <unistd.h>
#define TAG "MAIN"
void piPwm_initializeHardware()
{

}

void main()
{
  bcm2837_init();

  clock_configuration_t clockConfig;
  clockConfig.source = CLOCK_SOURCE_OSCILLATOR;
  clockConfig.mash = CLOCK_MASH_NONE;
  clockConfig.invert = false;
  clockConfig.divi = 50;
  clockConfig.divf = 0;

  LOGI(TAG, "Configuring clock");
  clockConfigure(clock_peripheral_pwm, &clockConfig);

  LOGI(TAG, "Enabling clock");
  clockEnable(clock_peripheral_pwm, true);

  pwm_configuration_t pwmConfig;
  pwmConfig.mode = pwm_mode_mark_space;
  pwmConfig.fifoEnable = false;
  pwmConfig.repeatLast = false;
  pwmConfig.invert = false;
  pwmConfig.silenceBit = 0;

  LOGI(TAG, "Configuring PWM");
  pwmConfigure(pwm_channel_1, &pwmConfig);
  pwmSetRange(pwm_channel_1, 1 << 12);

  LOGI(TAG, "Enabling PWM");
  pwmEnable(pwm_channel_1, true);

  //pwmSetData(pwm_channel_1, 1);
  //return;
  
  while(1)
  {
  for (int16_t i = 1; i < 1 << 12; i++)
  {
    LOGI(TAG, "Setting PWM output %d", i );
    pwmSetData(pwm_channel_1, i );
    usleep(.5e3);
  }
  for (int16_t i = 1 << 12; i >= 1; i--)
  {
    LOGI(TAG, "Setting PWM output %d", i );
    pwmSetData(pwm_channel_1, i );
    usleep(.5e3);
  }
  }
}