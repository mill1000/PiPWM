#ifndef __BCM2837_CLOCK__
#define __BCM2837_CLOCK__

#include "bcm2837.h"

typedef enum clock_peripheral_t
{
  clock_peripheral_gp0,
  clock_peripheral_gp1,
  clock_peripheral_gp2,
  clock_peripheral_pcm,
  clock_peripheral_pwm,
  clock_peripheral_max,
} clock_peripheral_t;

#define CLOCK_BASE_OFFSET (0x00101000)
#define CLOCK_GP0_OFFSET  (0x70)
#define CLOCK_GP1_OFFSET  (0x78)
#define CLOCK_GP2_OFFSET  (0x80)
#define CLOCK_PCM_OFFSET  (0x98)
#define CLOCK_PWM_OFFSET  (0xA0)

#define CLOCK_MANAGER_PASSWORD (0x5A)

typedef enum __attribute__((packed))
{
  CLOCK_MASH_NONE        = 0,
  CLOCK_MASH_1_STAGE     = 1,
  CLOCK_MASH_2_STAGE     = 2,
  CLOCK_MASH_3_STAGE     = 3,
} CLOCK_MASH;

typedef enum __attribute__((packed))
{
  CLOCK_SOURCE_GND          = 0,
  CLOCK_SOURCE_OSCILLATOR   = 1,
  CLOCK_SOURCE_TEST_DEBUG0  = 2,
  CLOCK_SOURCE_TEST_DEBUG1  = 3,
  CLOCK_SOURCE_PLLA         = 4,
  CLOCK_SOURCE_PLLC         = 5,
  CLOCK_SOURCE_PLLD         = 6,
  CLOCK_SOURCE_HDMI_AUX     = 7,
  // 8 - 15 also GND
} CLOCK_SOURCE;

typedef struct clock_configuration_t
{
  CLOCK_SOURCE source;
  CLOCK_MASH   mash;
  bool         invert;
  uint16_t     divi;
  uint16_t     divf;
} clock_configuration_t;

typedef struct clock_control_t
{
  uint32_t SRC : 4;
  uint32_t ENAB : 1;
  uint32_t KILL : 1;
  uint32_t _reserved : 1;
  uint32_t BUSY : 1;
  uint32_t FLIP : 1;
  uint32_t MASH : 2;
  uint32_t _reserved2 : 13;
  uint32_t PASSWD : 8;
} clock_control_t;

typedef struct clock_divisor_t
{
  uint32_t DIVF : 12;
  uint32_t DIVI : 12;
  uint32_t PASSWD : 8;
} clock_divisor_t;

typedef struct bcm2837_clock_t
{
  volatile clock_control_t  CTL;
  volatile clock_divisor_t  DIV;
} bcm2837_clock_t;

void clockInit(void* base);

static_assert(sizeof(clock_control_t) == sizeof(uint32_t), "clock_control_t must be 4 bytes.");
static_assert(sizeof(clock_divisor_t) == sizeof(uint32_t), "clock_divisor_t must be 4 bytes.");
static_assert(sizeof(bcm2837_clock_t) == 2 * sizeof(uint32_t), "bcm2837_clock_t must be 8 bytes.");

#endif