#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//=============================================================================
// Indicator LED
//=============================================================================
#define LED_PORT GPIOC
#define LED_PIN  GPIO9

//=============================================================================
// Defaults
//=============================================================================
#define ELS_Z_PULSES_PER_MM   (400)
#define ELS_X_PULSES_PER_MM   (400)

#define ELS_Z_BACKLASH_UM     (120)
#define ELS_X_BACKLASH_UM     (225)

// backlash pulse period / 2
#define ELS_BACKLASH_DELAY_US (1e3)

#define ELS_Z_MAX_MM          (550)
#define ELS_X_MAX_MM          (150)

// Actual PPR of the spindle encoder, factoring in any gear ratio.
#define ELS_S_ENCODER_PPR     (1200)
//=============================================================================
// Z-Axis Pins
//=============================================================================

#define ELS_Z_ENA_PORT        GPIOB
#define ELS_Z_ENA_PIN         GPIO13
#define ELS_Z_DIR_PORT        GPIOA
#define ELS_Z_DIR_PIN         GPIO15
#define ELS_Z_PUL_PORT        GPIOB
#define ELS_Z_PUL_PIN         GPIO2

#define ELS_Z_ENA_ACTIVE_LOW  1

//=============================================================================
// X-Axis Pins
//=============================================================================
#define ELS_X_ENA_PORT        GPIOC
#define ELS_X_ENA_PIN         GPIO13
#define ELS_X_DIR_PORT        GPIOC
#define ELS_X_DIR_PIN         GPIO10
#define ELS_X_PUL_PORT        GPIOC
#define ELS_X_PUL_PIN         GPIO11

#define ELS_X_ENA_ACTIVE_LOW  1

//=============================================================================
// Spindle Encoder
//=============================================================================

// ----------------------------------------------------------------------------
// Settings for spindle module that uses timer in encoder mode.
//
// IMPORTANT: These pins are tied to timer config in spindle.c and should not
//            be updated in isolation.
// ----------------------------------------------------------------------------
// Timer in encoder mode counts on rise & fall.
//
#define ELS_S_ENCODER1_PPR    (ELS_S_ENCODER_PPR * 2)
#define ELS_S_ENCODER1_PORTA  GPIOC
#define ELS_S_ENCODER1_PINA   GPIO6
#define ELS_S_ENCODER1_AFA    GPIO_AF2

#define ELS_S_ENCODER1_PORTB  GPIOC
#define ELS_S_ENCODER1_PINB   GPIO7
#define ELS_S_ENCODER1_AFB    GPIO_AF2

// ----------------------------------------------------------------------------
// Settings for threading module that uses interrupts to keep track of spindle.
// ----------------------------------------------------------------------------
// Interrupt counts on rise
//
#define ELS_S_ENCODER2_PPR    (ELS_S_ENCODER_PPR)
#define ELS_S_ENCODER2_PORTA  GPIOB
#define ELS_S_ENCODER2_PINA   GPIO3
#define ELS_S_ENCODER2_PORTB  GPIOB
#define ELS_S_ENCODER2_PINB   GPIO4

#define ELS_S_ENCODER2_IRQ    NVIC_EXTI3_IRQ
#define ELS_S_ENCODER2_EXTI   EXTI3

//=============================================================================
// Input Encoder
//=============================================================================
#define ELS_I_ENCODER_PPR     (100)

#define ELS_I_ENCODER_PORTA   GPIOC
#define ELS_I_ENCODER_PINA    GPIO2

#define ELS_I_ENCODER_PORTB   GPIOC
#define ELS_I_ENCODER_PINB    GPIO3

//-----------------------------------------------------------------------------
// These need to be updated based on the config above.
//-----------------------------------------------------------------------------
#define ELS_I_ENCODER_ISR     exti2_isr
#define ELS_I_ENCODER_IRQ     NVIC_EXTI2_IRQ
#define ELS_I_ENCODER_EXTI    EXTI2

//=============================================================================
// PS/2 Keypad D-8203
//=============================================================================
#define ELS_KEYPAD_CLK_PORT   GPIOC
#define ELS_KEYPAD_CLK_PIN    GPIO4
#define ELS_KEYPAD_DAT_PORT   GPIOC
#define ELS_KEYPAD_DAT_PIN    GPIO5

//-----------------------------------------------------------------------------
// These need to be updated based on the config above.
//-----------------------------------------------------------------------------
#define ELS_KEYPAD_ISR        exti4_isr
#define ELS_KEYPAD_IRQ        NVIC_EXTI4_IRQ
#define ELS_KEYPAD_EXTI       EXTI4

//-----------------------------------------------------------------------------
// PS/2 keypad scancodes, update if the keypad changes.
//-----------------------------------------------------------------------------
#define ELS_KEY_EOF           -1
#define ELS_KEY_OK            0xb4
#define ELS_KEY_EXIT          0xf2
#define ELS_KEY_SET_FEED      0xd8
#define ELS_KEY_REV_FEED      0xea

// basic functions that are manually driven on x-axis
#define ELS_KEY_FUN_TURN      0xee
#define ELS_KEY_FUN_THREAD    0x94

// selection menu for additional functions
#define ELS_KEY_SETTINGS      0xf8
#define ELS_KEY_FUN_SELECT    0xf6
#define ELS_KEY_FUN_F1        0xfa
#define ELS_KEY_FUN_F2        0xd6

#define ELS_KEY_SET_ZX        0xe6

#define ELS_KEY_SET_ZX_ORI    0xe8
#define ELS_KEY_SET_ZX_MIN    0xd2
#define ELS_KEY_SET_ZX_MAX    0xe4
#define ELS_KEY_JOG_ZX_MIN    0xcc
#define ELS_KEY_JOG_ZX_MAX    0xe0

#define ELS_KEY_SET_ZX_ORI    0xe8
#define ELS_KEY_JOG_ZX_ORI    0xd2
#define ELS_KEY_UNUSED_1      0xe4
#define ELS_KEY_UNUSED_2      0xcc
#define ELS_KEY_UNUSED_3      0xe0

#define ELS_KEY_LOCK          0xe2
#define ELS_KEY_ENC_MULT      0xf4


//-----------------------------------------------------------------------------
// Layout
//-----------------------------------------------------------------------------
//
// 0xee 0x94 0xf8 0xf6
// 0xd8 0xea 0xfa 0xf2
// 0xd6 0xe6 0xe8
// 0xd2 0xe4 0xcc 0xb4
// 0xe0 0xf4 0xe2
//
//  Numlock  /  *  -
//  7        8  9  +
//  4        5  6
//  1        2  3  OK
//  0        BS DEL
//
//-----------------------------------------------------------------------------

//=============================================================================
// Storage
//=============================================================================
#define ELS_EEPROM_I2C_SCK_PORT   GPIOB
#define ELS_EEPROM_I2C_SCK_PIN    GPIO10
#define ELS_EEPROM_I2C_SDA_PORT   GPIOC
#define ELS_EEPROM_I2C_SDA_PIN    GPIO12


//=============================================================================
// Z & X Axis setup - Ignore, purely for documentation of hardware.
//=============================================================================
//
// speed-up & reduction gives the drive ratio from stepper to leadscrew.
//
// ELS_Z_PITCH_UM        (3000)
// ELS_Z_STEPS_PER_REV   (200)
// ELS_Z_MICRO_STEPS     (4)
// ELS_Z_GEAR_RATIO_1    (2)      // stepper to pulley
// ELS_Z_GEAR_RATIO_2    (3)      // pulley to leadscrew
//
// ELS_X_PITCH_UM        (2000)
// ELS_X_STEPS_PER_REV   (200)
// ELS_X_MICRO_STEPS     (4)
// ELS_X_GEAR_RATIO_1    (1)      // stepper to pulley
// ELS_X_GEAR_RATIO_2    (1)      // pulley to leadscrew

#ifdef __cplusplus
}
#endif
