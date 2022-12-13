#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/timer.h>

#include "gpio.h"
#include "dro.h"
#include "nvic.h"
#include "config.h"

// linear glass scale resolution in microns.
#define  HBM_DRO_X_RESOLUTION 5
#define  HBM_DRO_Z_RESOLUTION 5

#define HBM_DRO_TIMER                     TIM6
#define HBM_DRO_TIMER_RELOAD			  50
#define HBM_DRO_TIMER_IRQ                 NVIC_TIM6_DAC_IRQ
#define HBM_DRO_TIMER_RCC                 RCC_TIM6
#define HBM_DRO_TIMER_RST                 RST_TIM6
#define HBM_DRO_TIMER_ISR 				  tim6_dac_isr

//==============================================================================
// State
//==============================================================================
long RulerX, RulerZ;  // store the raw values from the rulers

//==============================================================================
// Internal functions
//==============================================================================
static void hbm_dro_configure_gpio(void);
static void hbm_dro_timer_isr(void);
static void hbm_dro_timer_setup(void);

//==============================================================================
// ISR
//==============================================================================
static void hbm_dro_timer_isr(void) {
  TIM_SR(HBM_DRO_TIMER) &= ~TIM_SR_UIF;

  static uint8_t TimeSlice = 0;
  static bool Pause = 0;
  static long TempRulerX = 0, TempRulerZ = 0;
  static long OldRulerX = 0, OldRulerZ = 0;

  // Readout of rulers
  switch (TimeSlice++ & 0x03) // modulo 4
  {
  case 0:  // set clock high
    if (!Pause) {
    	els_gpio_set(GPIOA, GPIO12); // clock high
    	els_gpio_set(GPIOB, GPIO5);
    }
    break;

  case 1: // sample and clock low
    if (!Pause) {
      if (els_gpio_get(GPIOA, GPIO11))  TempRulerX |= 0x80000000; // Set MSB bit
        else TempRulerX &= ~0x80000000; // Clear MSB bit of result
      if (els_gpio_get(GPIOB, GPIO1)) TempRulerZ |= 0x80000000; // Set MSB bit
        else TempRulerZ &= ~0x80000000; // Clear MSB bit of result
      // Clearing the MSB is nescessary because of sign extend while shifting
      els_gpio_clear(GPIOA, GPIO12); // clock low
      els_gpio_clear(GPIOB, GPIO5);
    }
    break;

  case 2:
    if (!Pause) {
      TempRulerX = TempRulerX >> 1; // shift right. Rulers give LSB first
      TempRulerZ = TempRulerZ >> 1; // shift right
    }
    break;

  case 3:
    if (TimeSlice == 84) { // 21 cycles of 4 steps
      RulerX = TempRulerX >> 10; // Final shift to get LSB right
      RulerZ = TempRulerZ >> 10;
      Pause = true;
      if (abs(OldRulerX - RulerX) < 5) // Debouncing
    	  els_dro.xpos_um = (RulerX * 10) - els_dro.xpos_zero;
      if (abs(OldRulerZ - RulerZ) < 5)
      els_dro.zpos_um = (RulerZ * 10) - els_dro.zpos_zero;
    }
    if (TimeSlice >= 200) {
      OldRulerX = RulerX;
      OldRulerZ = RulerZ;
      TimeSlice = 0;
      TempRulerX = 0;
      TempRulerZ = 0;
      Pause = false;  // Start again
    }
    break;
  }
  // if you need to do time consuming things do it while Pause == true
  // or in timeslices 84 < TS < 200
  // so missing some interrupts will not harm the ruler readout
}

//==============================================================================
// API
//==============================================================================
void hbm_dro_setup(void) {
  els_dro.xpos_um = 0;
  els_dro.zpos_um = 0;

  hbm_dro_configure_gpio();
  hbm_dro_timer_setup();
}


//==============================================================================
// Internal functions
//==============================================================================
static void hbm_dro_configure_gpio(void) {
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO11);
  els_gpio_mode_output(GPIOA, GPIO12); // clock low

  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO1);
  els_gpio_mode_output(GPIOB, GPIO5);

  els_gpio_clear(GPIOA, GPIO12); // clock low
  els_gpio_clear(GPIOB, GPIO5);
}

static void hbm_dro_timer_setup(void) {
  rcc_periph_clock_enable(HBM_DRO_TIMER_RCC);
  rcc_periph_reset_pulse(HBM_DRO_TIMER_RST);

  // clock division 0, alignment edge, count up.
  timer_set_mode(HBM_DRO_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
//rcc_apb1_frequency = 16.000.000;

  // Set prescalar to 1 us
  timer_set_prescaler(HBM_DRO_TIMER, 90 - 1);

  // keep running
  timer_disable_preload(HBM_DRO_TIMER);
  timer_continuous_mode(HBM_DRO_TIMER);

  // set to 50 us ~ 20KHz
  timer_set_period(HBM_DRO_TIMER, HBM_DRO_TIMER_RELOAD);

  els_nvic_irq_set_handler(HBM_DRO_TIMER_IRQ, hbm_dro_timer_isr);
  nvic_set_priority(HBM_DRO_TIMER_IRQ, 3);
  nvic_enable_irq(HBM_DRO_TIMER_IRQ);
  timer_enable_update_event(HBM_DRO_TIMER);


  timer_set_counter(HBM_DRO_TIMER, 0);
  timer_enable_counter(HBM_DRO_TIMER);
  timer_enable_irq(HBM_DRO_TIMER, TIM_DIER_UIE);
}


