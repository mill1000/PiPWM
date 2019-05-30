#include <stddef.h>
#include <assert.h>

#include <bcm2837_gpio.h>

static bcm2837_gpio_t* gpio = NULL;

/**
  @brief  Initalize the GPIO object at the given base address

  @param  base Base address of GPIO peripheral
  @retval none
*/
void gpioInit(void* base)
{
  assert(base != NULL);
  assert(gpio == NULL);

  gpio = base;
}