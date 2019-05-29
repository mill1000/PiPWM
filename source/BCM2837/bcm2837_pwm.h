#ifndef __BCM2837_PWM__
#define __BCM2837_PWM__

#include "bcm2837.h"

typedef enum pwm_channel_t
{
  pwm_channel_1,
  pwm_channel_2,
  pwm_channel_max,
} pwm_channel_t;

#define PWM_BASE_OFFSET (0x0020C000)

// PWM CTL Register Bits
#define PWM_CTL_MSEN2 (1 << 15)
#define PWM_CTL_USEF2 (1 << 13)
#define PWM_CTL_POLA2 (1 << 12)
#define PWM_CTL_SBIT2 (1 << 11)
#define PWM_CTL_RPTL2 (1 << 10)
#define PWM_CTL_MODE2 (1 << 9)
#define PWM_CTL_PWEN2 (1 << 8)

#define PWM_CTL_MSEN1 (1 << 7)
#define PWM_CTL_CLRF1 (1 << 6)
#define PWM_CTL_USEF1 (1 << 5)
#define PWM_CTL_POLA1 (1 << 4)
#define PWM_CTL_SBIT1 (1 << 3)
#define PWM_CTL_RPTL1 (1 << 2)
#define PWM_CTL_MODE1 (1 << 1)
#define PWM_CTL_PWEN1 (1 << 0)

// PWM STA Register Bits
#define PWM_STA_STA2  (1 << 10)
#define PWM_STA_STA1  (1 << 9)
#define PWM_STA_BERR  (1 << 8)
#define PWM_STA_GAPO2 (1 << 5)
#define PWM_STA_GAPO1 (1 << 4)
#define PWM_STA_RERR1 (1 << 3)
#define PWM_STA_WERR1 (1 << 2)
#define PWM_STA_EMPT1 (1 << 1)
#define PWM_STA_FULL1 (1 << 0)

// PWM DMAC Register Bits
#define PWM_DMAC_ENAB         (1 << 31)
#define PWM_DMAC_PANIC_SHIFT  (8)
#define PWM_DMAC_PANIC_MASK   (0xFF << 8)
#define PWM_DMAC_DREQ_SHIFT   (0)
#define PWM_DMAC_DREQ_MASK    (0xFF << 0)

typedef struct pwm_control_t
{
  uint32_t PWEN1 : 1;
  uint32_t MODE1 : 1;
  uint32_t RPTL1 : 1;
  uint32_t SBIT1 : 1;
  uint32_t POLA1 : 1;
  uint32_t USEF1 : 1;
  uint32_t CLRF1 : 1;
  uint32_t MSEN1 : 1;
  uint32_t PWEN2 : 1;
  uint32_t MODE2 : 1;
  uint32_t RPTL2 : 1;
  uint32_t SBIT2 : 1;
  uint32_t POLA2 : 1;
  uint32_t USEF2 : 1;
  uint32_t _reserved : 1;
  uint32_t MSEN2 : 1;
  uint32_t _reserved2 : 16;
} pwm_control_t;

typedef struct pwm_status_t
{
  uint32_t FULL1 : 1;
  uint32_t EMPT1 : 1;
  uint32_t WERR1 : 1;
  uint32_t RERR1 : 1;
  uint32_t GAPO1 : 1;
  uint32_t GAPO2 : 1;
  uint32_t GAPO3 : 1; // Ch 3 doesn't exist?
  uint32_t GAPO4 : 1; // Ch 4 doesn't exist?
  uint32_t BERR : 1;
  uint32_t STA1 : 1;
  uint32_t STA2 : 1;
  uint32_t STA3 : 1; // Ch 3 doesn't exist?
  uint32_t STA4 : 1; // Ch 4 doesn't exist?
  uint32_t _reserved : 18;
} pwm_status_t;

typedef struct pwm_dma_config_t
{
  uint32_t DREQ : 8;
  uint32_t PANIC : 8;
  uint32_t _reserved : 15;
  uint32_t ENAB : 1;
} pwm_dma_config_t;

typedef struct bcm2837_pwm_t
{
  volatile pwm_control_t    CTL;
  volatile pwm_status_t     STA;
  volatile pwm_dma_config_t DMAC;
  volatile uint32_t         _reserved;
  volatile uint32_t         RNG1;
  volatile uint32_t         DAT1;
  volatile uint32_t         FIF1;
  volatile uint32_t         _reserved2;
  volatile uint32_t         RNG2;
  volatile uint32_t         DAT2;
} bcm2837_pwm_t;

void pwmInit(void* base);
void pwmReset(void);
void pwmConfigureDma(bool enable, uint32_t panicThreshold, uint32_t dreqThreshold);
void pwmSetData(pwm_channel_t channel, uint32_t data);
void pwmSetRange(pwm_channel_t channel, uint32_t data);

static_assert(sizeof(pwm_control_t) == sizeof(uint32_t), "pwm_control_t must be 4 bytes.");
static_assert(sizeof(pwm_status_t) == sizeof(uint32_t), "pwm_status_t must be 4 bytes.");
static_assert(sizeof(pwm_dma_config_t) == sizeof(uint32_t), "pwm_dma_config_t must be 4 bytes.");
static_assert(sizeof(bcm2837_pwm_t) == 10 * sizeof(uint32_t), "bcm2837_pwm_t must be 40 bytes.");

#endif