#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>

#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/timer.h>

#include "delay.h"
#include "gpio.h"
#include "nvic.h"
#include "timer.h"

#include "tft/tft.h"
#include "tft/fonts/fonts.h"

#include "constants.h"

#include "dro.h"
#include "encoder.h"
#include "keypad.h"
#include "spindle.h"
#include "stepper.h"

#include "bore_hole.h"
#include "utils.h"

#define ELS_Z_JOG_MM_S  8
#define ELS_X_JOG_MM_S  4

#define ELS_Z_SLO_MM_S  2
#define ELS_X_SLO_MM_S  1

//==============================================================================
// Externs
//==============================================================================
extern tft_device_t tft;

extern const tft_font_t noto_sans_mono_bold_arrows_24;
extern const tft_font_t noto_sans_mono_bold_26;
extern const tft_font_t gears_regular_32;
extern const tft_font_t gears_regular_50;
extern const tft_font_t inconsolata_lgc_bold_14;

//==============================================================================
// Config
//==============================================================================
typedef enum {
  ELS_BORE_HOLE_IDLE        = 1,
  ELS_BORE_HOLE_PAUSED      = 2,
  ELS_BORE_HOLE_ACTIVE      = 4,
  ELS_BORE_HOLE_SET_ZAXES   = 8,
  ELS_BORE_HOLE_ZJOG        = 16,
  ELS_BORE_HOLE_SET_XAXES   = 32,
  ELS_BORE_HOLE_XJOG        = 64,
  ELS_BORE_HOLE_SET_FEED    = 128,
  ELS_BORE_HOLE_SET_DOC     = 256,
  ELS_BORE_HOLE_SET_DEPTH   = 512,
  ELS_BORE_HOLE_SET_WIDTH  = 1024
} els_bore_hole_state_t;

typedef enum {
  ELS_BORE_HOLE_OP_NA      = -1,
  ELS_BORE_HOLE_OP_IDLE    = 0,
  ELS_BORE_HOLE_OP_READY   = 1,
  ELS_BORE_HOLE_OP_MOVEZ0  = 2,
  ELS_BORE_HOLE_OP_MOVEX0  = 3,
  ELS_BORE_HOLE_OP_START   = 4,
  ELS_BORE_HOLE_OP_ATZ0    = 5,
  ELS_BORE_HOLE_OP_ATZL    = 6,
  ELS_BORE_HOLE_OP_ATZLXM  = 7,
  ELS_BORE_HOLE_OP_ATZ0XM  = 8,
  ELS_BORE_HOLE_OP_FEED_IN = 9,
  ELS_BORE_HOLE_OP_SPRING  = 10,
  ELS_BORE_HOLE_OP_DONE    = 11
} els_bore_hole_op_state_t;

static const char *op_labels[] = {
  "CONFIG ",
  "READY  ",
  "MOVE Z0",
  "MOVE X0",
  "ORIGIN ",
  "BORING ",
  "LIMIT  ",
  "BACKOFF",
  "RETURN ",
  "FEED IN",
  "SPRING ",
  "DONE   "
};

//==============================================================================
// Internal state
//==============================================================================
#define ELS_BORE_HOLE_FEED_MIN  (100)
#define ELS_BORE_HOLE_FEED_MAX  (5000)

#define ELS_BORE_HOLE_DOC_MIN   (50)
#define ELS_BORE_HOLE_DOC_MAX   (2000)

static struct {
  uint32_t feed_um;
  uint32_t feed_mm_s;

  // not supported.
  bool     feed_reverse;

  int32_t  depth_of_cut_um;

  bool     locked;

  double   depth;
  double   width;

  // input read for jogging
  int32_t  encoder_pos;

  // module state
  els_bore_hole_state_t state;

  // tracking variables for display refresh.
  els_bore_hole_state_t prev_state;
  els_spindle_direction_t prev_dir;

  // operation state
  els_bore_hole_op_state_t op_state;

  // dro
  bool show_dro;

} els_bore_hole = {
  .depth_of_cut_um = 100,
  .feed_um = 1000,
  .depth = 2,
  .width = 0.5
};

//==============================================================================
// Internal functions
//==============================================================================
static void els_bore_hole_run(void);
static void els_bore_hole_turn(void);

static void els_bore_hole_display_refresh(void);
static void els_bore_hole_display_setting(void);
static void els_bore_hole_display_axes(void);
static void els_bore_hole_display_header(void);
static void els_bore_hole_display_diagram(void);

static void els_bore_hole_set_feed(void);
static void els_bore_hole_set_depth_of_cut(void);
static void els_bore_hole_set_depth(void);
static void els_bore_hole_set_width(void);

static void els_bore_hole_set_zaxes(void);
static void els_bore_hole_set_xaxes(void);

static void els_bore_hole_keypad_process(void);

// manual jogging
static void els_bore_hole_zjog(void);
static void els_bore_hole_xjog(void);

//==============================================================================
// API
//==============================================================================
void els_bore_hole_setup(void) {
  // NO-OP
}

void els_bore_hole_start(void) {
  char text[32];

  tft_filled_rectangle(&tft, 0,   0, 480, 320, ILI9481_BLACK);

  tft_filled_rectangle(&tft, 0,  50, 480,   5, ILI9481_GRAY);
  tft_filled_rectangle(&tft, 0,  50, 480,   1, ILI9481_LITEGRAY);

  tft_filled_rectangle(&tft, 0, 100, 480,   5, ILI9481_GRAY);
  tft_filled_rectangle(&tft, 0, 100, 480,   1, ILI9481_LITEGRAY);

  tft_filled_rectangle(&tft, 0, 190, 480,   5, ILI9481_GRAY);
  tft_filled_rectangle(&tft, 0, 190, 480,   1, ILI9481_LITEGRAY);

  els_bore_hole_display_header();

  // angle symbol
  tft_font_write_bg(&tft, 165,  62, "J", &gears_regular_32, ILI9481_WHITE, ILI9481_BLACK);

  // pulse symbol
  tft_font_write_bg(&tft, 355,  58, "K", &gears_regular_32, ILI9481_WHITE, ILI9481_BLACK);

  // reset state
  els_bore_hole.prev_state = 0;
  els_bore_hole.prev_dir = 0;
  els_bore_hole.feed_mm_s = els_bore_hole.feed_um / 1000.0;

  if (!els_bore_hole.show_dro)
    els_bore_hole_display_diagram();

  els_bore_hole.state = ELS_BORE_HOLE_IDLE;

  els_bore_hole_display_setting();
  els_bore_hole_display_axes();
  els_bore_hole_display_refresh();

  els_stepper_start();
}

void els_bore_hole_stop(void) {
  els_stepper_stop();
}

bool els_bore_hole_busy(void) {
  return els_bore_hole.state != ELS_BORE_HOLE_IDLE;
}

void els_bore_hole_update(void) {
  static uint64_t last_refreshed_at = 0;

  uint64_t elapsed = els_timer_elapsed_microseconds();

  bool kp_locked = els_keypad_locked();
  if (els_bore_hole.locked != kp_locked) {
    els_bore_hole.locked = kp_locked;
    els_bore_hole_display_header();
  }

  if (els_bore_hole.state & (ELS_BORE_HOLE_IDLE | ELS_BORE_HOLE_PAUSED | ELS_BORE_HOLE_ACTIVE))
    els_bore_hole_keypad_process();

  switch (els_bore_hole.state) {
    case ELS_BORE_HOLE_PAUSED:
    case ELS_BORE_HOLE_ACTIVE:
      els_bore_hole_run();
      break;
    case ELS_BORE_HOLE_SET_FEED:
      els_bore_hole_set_feed();
      break;
    case ELS_BORE_HOLE_SET_DOC:
      els_bore_hole_set_depth_of_cut();
      break;
    case ELS_BORE_HOLE_SET_DEPTH:
      els_bore_hole_set_depth();
      break;
    case ELS_BORE_HOLE_SET_WIDTH:
      els_bore_hole_set_width();
      break;
    default:
      if (els_bore_hole.state & (ELS_BORE_HOLE_SET_ZAXES | ELS_BORE_HOLE_ZJOG))
        els_bore_hole_set_zaxes();
      else if (els_bore_hole.state & (ELS_BORE_HOLE_SET_XAXES | ELS_BORE_HOLE_XJOG))
        els_bore_hole_set_xaxes();
      break;
  }

  if (elapsed - last_refreshed_at > 1e5) {
    last_refreshed_at = elapsed;
    els_bore_hole_display_refresh();
  }
}

//==============================================================================
// Internal functions
//==============================================================================

// ----------------------------------------------------------------------------------
// Display functions
// ----------------------------------------------------------------------------------
static void els_bore_hole_display_setting(void) {
  char text[32];

  els_sprint_double2(text, sizeof(text), els_bore_hole.feed_um / 1000.0, "Zf");
  if (els_bore_hole.state == ELS_BORE_HOLE_SET_FEED)
    tft_font_write_bg(&tft, 310, 102, text, &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  else
    tft_font_write_bg(&tft, 310, 102, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  els_sprint_double2(text, sizeof(text), els_bore_hole.depth_of_cut_um / 1000.0, "Xs");
  if (els_bore_hole.state == ELS_BORE_HOLE_SET_DOC)
    tft_font_write_bg(&tft, 310, 135, text, &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  else
    tft_font_write_bg(&tft, 310, 135, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  els_sprint_double3(text, sizeof(text), els_bore_hole.depth, "D");
  if (els_bore_hole.state == ELS_BORE_HOLE_SET_DEPTH)
    tft_font_write_bg(&tft, 310, 228, text, &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  else
    tft_font_write_bg(&tft, 310, 228, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  els_sprint_double3(text, sizeof(text), els_bore_hole.width, "W");
  if (els_bore_hole.state == ELS_BORE_HOLE_SET_WIDTH)
    tft_font_write_bg(&tft, 310, 262, text, &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  else
    tft_font_write_bg(&tft, 310, 262, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);
}

static void els_bore_hole_display_axes(void) {
  char text[32];

  els_sprint_double3(text, sizeof(text), els_stepper->zpos, "Z");

  if (els_bore_hole.state & ELS_BORE_HOLE_SET_ZAXES)
    tft_font_write_bg(&tft, 8, 102, text, &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  else
    tft_font_write_bg(&tft, 8, 102, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  els_sprint_double3(text, sizeof(text), els_stepper->xpos, "X");
  if (els_bore_hole.state & ELS_BORE_HOLE_SET_XAXES)
    tft_font_write_bg(&tft, 8, 135, text, &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  else
    tft_font_write_bg(&tft, 8, 135, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  if (els_bore_hole.show_dro) {
    tft_font_write_bg(&tft, 8, 190, "DRO", &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

    els_sprint_double33(text, sizeof(text), (els_dro.zpos_um / 1000.0), "Z");
    tft_font_write_bg(&tft, 8, 228, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

    els_sprint_double33(text, sizeof(text), (els_dro.xpos_um / 1000.0), "X");
    tft_font_write_bg(&tft, 8, 262, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);
  }
}

static void els_bore_hole_display_diagram(void) {
  // ----------------------------------------------------------------------------------------------
  // diagram
  // ----------------------------------------------------------------------------------------------
  tft_filled_rectangle(&tft, 100, 220, 156, 80, ILI9481_WHITE);
  tft_filled_rectangle(&tft, 156, 245, 100, 30, ILI9481_BLACK);
  tft_filled_rectangle(&tft, 156, 230, 100, 15, ILI9481_ORANGE);
  tft_filled_rectangle(&tft, 156, 275, 100, 15, ILI9481_ORANGE);

  tft_font_write_bg(&tft, 156, 178, "A", &noto_sans_mono_bold_arrows_24, ILI9481_WHITE, ILI9481_BLACK);
  tft_font_write_bg(&tft, 205, 194, "D", &inconsolata_lgc_bold_14, ILI9481_WHITE, ILI9481_BLACK);
  tft_font_write_bg(&tft, 240, 178, "C", &noto_sans_mono_bold_arrows_24, ILI9481_WHITE, ILI9481_BLACK);

  tft_font_write_bg(&tft, 260, 194, "D", &noto_sans_mono_bold_arrows_24, ILI9481_WHITE, ILI9481_BLACK);
  tft_font_write_bg(&tft, 280, 224, "W", &inconsolata_lgc_bold_14, ILI9481_WHITE, ILI9481_BLACK);
  tft_font_write_bg(&tft, 260, 229, "B", &noto_sans_mono_bold_arrows_24, ILI9481_WHITE, ILI9481_BLACK);

  // origin
  tft_font_write_bg(&tft, 250, 240, ".", &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);

  // legend
  tft_font_write_bg(&tft, 8, 200, ".", &noto_sans_mono_bold_26, ILI9481_YELLOW, ILI9481_BLACK);
  tft_font_write_bg(&tft, 26, 220, "(0,0)", &inconsolata_lgc_bold_14, ILI9481_WHITE, ILI9481_BLACK);
  // ----------------------------------------------------------------------------------------------
}

static void els_bore_hole_display_header(void) {
  if (els_bore_hole.locked) {
    tft_filled_rectangle(&tft, 0, 0, 480, 50, ILI9481_RED);
    tft_font_write_bg(&tft, 8, 0, "BORING - HOLE", &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_RED);
    tft_font_write_bg(&tft, 446, 6, "C", &gears_regular_32, ILI9481_WHITE, ILI9481_RED);
  }
  else {
    tft_filled_rectangle(&tft, 0,   0, 480,  50, ILI9481_DIANNE);
    tft_font_write_bg(&tft, 8, 0, "BORING - HOLE", &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_DIANNE);
    tft_font_write_bg(&tft, 446, 6, "D", &gears_regular_32, ILI9481_WHITE, ILI9481_DIANNE);
  }
}

static void els_bore_hole_display_refresh(void) {
  els_spindle_direction_t dir = els_spindle_get_direction();
  if (dir != els_bore_hole.prev_dir) {
    els_bore_hole.prev_dir = dir;
    tft_filled_rectangle(&tft, 0, 55, 60, 45, ILI9481_BLACK);
    switch (dir) {
      case ELS_S_DIRECTION_CW:
        // cw symbol
        tft_font_write_bg(&tft, 5, 62, "A", &gears_regular_32, ILI9481_WHITE, ILI9481_BLACK);
        break;
      case ELS_S_DIRECTION_CCW:
        // ccw symbol
        tft_font_write_bg(&tft, 5, 62, "B", &gears_regular_32, ILI9481_WHITE, ILI9481_BLACK);
        break;
      default:
        // spindle symbol
        tft_font_write_bg(&tft, 3, 52, "N", &gears_regular_50, ILI9481_WHITE, ILI9481_BLACK);
        break;
    }
  }

  char text[32];
  snprintf(text, sizeof(text), "%04d", els_spindle_get_rpm());
  tft_font_write_bg(&tft, 62, 52, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  double angle = els_spindle_get_angle();
  snprintf(text, sizeof(text), "%03d.%02d", (int)angle, (int)((angle - (int)angle) * 100));
  tft_font_write_bg(&tft, 200, 52, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  snprintf(text, sizeof(text), "%04d", els_spindle_get_counter());
  tft_font_write_bg(&tft, 396, 52, text, &noto_sans_mono_bold_26, ILI9481_WHITE, ILI9481_BLACK);

  els_bore_hole_display_axes();

  static els_bore_hole_op_state_t prev_op_state = ELS_BORE_HOLE_OP_NA;
  if (els_bore_hole.op_state != prev_op_state) {
    prev_op_state = els_bore_hole.op_state;
    const char *label = op_labels[els_bore_hole.op_state];
    tft_filled_rectangle(&tft, 310, 195, 169, 30, ILI9481_BLACK);
    tft_font_write_bg(&tft, 310, 190, label, &noto_sans_mono_bold_26, ILI9481_CERULEAN, ILI9481_BLACK);
  }
}

// ----------------------------------------------------------------------------------
// Top module level keypad handler.
// ----------------------------------------------------------------------------------
static void els_bore_hole_keypad_process(void) {
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
      if (els_bore_hole.state == ELS_BORE_HOLE_IDLE) {
        if (els_bore_hole.depth > 0 && els_bore_hole.width > 0) {
          els_bore_hole.state = ELS_BORE_HOLE_PAUSED;
          els_bore_hole.op_state = ELS_BORE_HOLE_OP_READY;
        }
        else {
          els_bore_hole.state = ELS_BORE_HOLE_SET_DEPTH;
          els_bore_hole_display_setting();
        }
      }
      break;
    case ELS_KEY_EXIT:
      if (els_bore_hole.state & (ELS_BORE_HOLE_PAUSED | ELS_BORE_HOLE_ACTIVE)) {
        els_bore_hole.state = ELS_BORE_HOLE_IDLE;
        els_bore_hole.op_state = ELS_BORE_HOLE_OP_IDLE;
      }
      break;
    case ELS_KEY_SET_FEED:
      if (els_bore_hole.state & (ELS_BORE_HOLE_IDLE | ELS_BORE_HOLE_PAUSED)) {
        els_bore_hole.state = ELS_BORE_HOLE_SET_FEED;
        els_bore_hole.encoder_pos = 0;
        els_bore_hole_display_setting();
        els_encoder_reset();
      }
      break;
    case ELS_KEY_FUN_F1:
      if (els_bore_hole.state & (ELS_BORE_HOLE_IDLE | ELS_BORE_HOLE_PAUSED)) {
        els_bore_hole.state = ELS_BORE_HOLE_SET_DEPTH;
        els_bore_hole.encoder_pos = 0;
        els_bore_hole_display_setting();
        els_encoder_reset();
      }
      break;
    case ELS_KEY_FUN_F2:
      els_bore_hole.show_dro = !els_bore_hole.show_dro;
      tft_filled_rectangle(&tft, 0, 200, 300, 120, ILI9481_BLACK);
      if (els_bore_hole.show_dro)
        els_bore_hole_display_axes();
      else
        els_bore_hole_display_diagram();
      break;
    case ELS_KEY_SET_ZX:
      if (els_bore_hole.state & (ELS_BORE_HOLE_IDLE | ELS_BORE_HOLE_PAUSED)) {
        els_bore_hole.state = ELS_BORE_HOLE_SET_ZAXES;
        els_bore_hole.encoder_pos = 0;
        els_bore_hole_display_axes();
        els_encoder_reset();
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------------------
// Function 1: primary turning handler, detects direction change and toggles the DIR pin.
// ---------------------------------------------------------------------------------------
static void els_bore_hole_run(void) {
  int zdir = (els_bore_hole.feed_reverse ? 1 : -1);

  switch (els_spindle_get_direction()) {
    case ELS_S_DIRECTION_CW:
    case ELS_S_DIRECTION_CCW:
      if (els_bore_hole.state == ELS_BORE_HOLE_PAUSED) {
        if (!els_stepper->zbusy && !els_stepper->xbusy && els_spindle_get_counter() == 0)
          els_bore_hole.state = ELS_BORE_HOLE_ACTIVE;
      }
      break;
    default:
      if (els_bore_hole.state == ELS_BORE_HOLE_ACTIVE)
        els_bore_hole.state = ELS_BORE_HOLE_PAUSED;
      break;
  }

  if (els_bore_hole.state == ELS_BORE_HOLE_ACTIVE)
    els_bore_hole_turn();
}

#define TURN_PRECISION (1e-2)
static void els_bore_hole_turn(void) {
  switch (els_bore_hole.op_state) {
    case ELS_BORE_HOLE_OP_NA:
      break;
    case ELS_BORE_HOLE_OP_IDLE:
      break;
    case ELS_BORE_HOLE_OP_READY:
      els_bore_hole.op_state = ELS_BORE_HOLE_OP_MOVEZ0;
      break;
    case ELS_BORE_HOLE_OP_MOVEZ0:
      if (els_stepper->zbusy)
        break;
      // move to Z=0
      if (fabs(els_stepper->zpos) > TURN_PRECISION)
        els_stepper_move_z(0 - els_stepper->zpos, ELS_Z_SLO_MM_S);
      else
        els_bore_hole.op_state = ELS_BORE_HOLE_OP_MOVEX0;
      break;
    case ELS_BORE_HOLE_OP_MOVEX0:
      if (els_stepper->xbusy)
        break;

      // move to X=0
      if (fabs(els_stepper->xpos) > TURN_PRECISION)
        els_stepper_move_x(0 - els_stepper->xpos, ELS_X_SLO_MM_S);
      else
        els_bore_hole.op_state = ELS_BORE_HOLE_OP_START;
      break;
    case ELS_BORE_HOLE_OP_START:
      // initial move.
      els_stepper_move_z(-els_bore_hole.depth, els_bore_hole.feed_mm_s);
      els_bore_hole.op_state = ELS_BORE_HOLE_OP_ATZ0;
      break;
    case ELS_BORE_HOLE_OP_ATZ0:
      if (els_stepper->zbusy)
        break;

      // backoff
      els_stepper_move_x(-1, ELS_X_SLO_MM_S);
      els_bore_hole.op_state = ELS_BORE_HOLE_OP_ATZL;
      break;
    case ELS_BORE_HOLE_OP_ATZL:
      if (els_stepper->xbusy)
        break;
      // back to Z=0
      els_stepper_move_z(els_bore_hole.depth, ELS_Z_SLO_MM_S);
      els_bore_hole.op_state = ELS_BORE_HOLE_OP_ATZLXM;
      break;
    case ELS_BORE_HOLE_OP_ATZLXM:
      if (els_stepper->zbusy)
        break;

      els_stepper_move_x(1, ELS_X_SLO_MM_S);
      els_bore_hole.op_state = ELS_BORE_HOLE_OP_ATZ0XM;
      break;
    case ELS_BORE_HOLE_OP_ATZ0XM:
      if (els_stepper->xbusy)
        break;

      if (fabs(els_bore_hole.width - els_stepper->xpos) > TURN_PRECISION) {
        double xd;
        xd = MIN(
          els_bore_hole.width - els_stepper->xpos,
          els_bore_hole.depth_of_cut_um / 1000.0
        );

        els_stepper_move_x(xd, ELS_X_SLO_MM_S);
        els_bore_hole.op_state = ELS_BORE_HOLE_OP_FEED_IN;
      }
      else {
        // spring pass
        els_bore_hole.op_state = ELS_BORE_HOLE_OP_SPRING;
        els_stepper_move_z(0 - els_stepper->zpos, els_bore_hole.feed_mm_s / 2.0);
      }
      break;
    case ELS_BORE_HOLE_OP_FEED_IN:
      if (els_stepper->xbusy)
        break;

      els_bore_hole.op_state = ELS_BORE_HOLE_OP_ATZ0;
      els_stepper_move_z(-els_bore_hole.depth, els_bore_hole.feed_mm_s);
      break;
    case ELS_BORE_HOLE_OP_SPRING:
      if (els_stepper->zbusy)
        break;

      els_bore_hole.op_state = ELS_BORE_HOLE_OP_DONE;
      els_stepper_move_x(0 - els_stepper->xpos, ELS_X_SLO_MM_S);
      break;
    case ELS_BORE_HOLE_OP_DONE:
      // beer time
      els_bore_hole.state = ELS_BORE_HOLE_IDLE;
      break;
  }
}

// ----------------------------------------------------------------------------------
// Function 2.1: feed settings.
// ----------------------------------------------------------------------------------
static void els_bore_hole_set_feed(void) {
  int32_t encoder_curr;
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
    case ELS_KEY_EXIT:
      els_bore_hole.state = ELS_BORE_HOLE_IDLE;
      els_bore_hole_display_setting();
      break;
    case ELS_KEY_SET_FEED:
      els_bore_hole.state = ELS_BORE_HOLE_SET_DOC;
      els_bore_hole_display_setting();
      break;
    case ELS_KEY_REV_FEED:
      els_bore_hole_display_setting();
      break;
    default:
      encoder_curr = els_encoder_read();
      if (els_bore_hole.encoder_pos != encoder_curr) {
        int32_t delta = (encoder_curr - els_bore_hole.encoder_pos) * 100;
        if (els_bore_hole.feed_um + delta <= ELS_BORE_HOLE_FEED_MIN)
          els_bore_hole.feed_um = ELS_BORE_HOLE_FEED_MIN;
        else if (els_bore_hole.feed_um + delta >= ELS_BORE_HOLE_FEED_MAX)
          els_bore_hole.feed_um = ELS_BORE_HOLE_FEED_MAX;
        else
          els_bore_hole.feed_um += delta;
        els_bore_hole.encoder_pos = encoder_curr;
        els_bore_hole_display_setting();
        els_bore_hole.feed_mm_s = els_bore_hole.feed_um / 1000.0;
      }
      break;
  }
}

// ----------------------------------------------------------------------------------
// Function 2.2: depth of cut settings.
// ----------------------------------------------------------------------------------
static void els_bore_hole_set_depth_of_cut(void) {
  int32_t encoder_curr;
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
    case ELS_KEY_EXIT:
      els_bore_hole.state = ELS_BORE_HOLE_IDLE;
      els_bore_hole_display_setting();
      break;
    case ELS_KEY_SET_FEED:
      els_bore_hole.state = ELS_BORE_HOLE_SET_FEED;
      els_bore_hole_display_setting();
      break;
    default:
      encoder_curr = els_encoder_read();
      if (els_bore_hole.encoder_pos != encoder_curr) {
        int32_t delta = (encoder_curr - els_bore_hole.encoder_pos) * 50;
        if (els_bore_hole.depth_of_cut_um + delta <= ELS_BORE_HOLE_DOC_MIN)
          els_bore_hole.depth_of_cut_um = ELS_BORE_HOLE_DOC_MIN;
        else if (els_bore_hole.depth_of_cut_um + delta >= ELS_BORE_HOLE_DOC_MAX)
          els_bore_hole.depth_of_cut_um = ELS_BORE_HOLE_DOC_MAX;
        else
          els_bore_hole.depth_of_cut_um += delta;
        els_bore_hole.encoder_pos = encoder_curr;
        els_bore_hole_display_setting();
      }
      break;
  }
}

void els_bore_hole_set_depth(void) {
  int32_t encoder_curr;
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
    case ELS_KEY_EXIT:
      els_bore_hole.state = ELS_BORE_HOLE_IDLE;
      els_bore_hole_display_setting();
      break;
    case ELS_KEY_FUN_F1:
      els_bore_hole.state = ELS_BORE_HOLE_SET_WIDTH;
      els_bore_hole_display_setting();
      break;
    default:
      encoder_curr = els_encoder_read();
      if (els_bore_hole.encoder_pos != encoder_curr) {
        double delta = (encoder_curr - els_bore_hole.encoder_pos) * 0.1;
        if (els_bore_hole.depth + delta <= 0)
          els_bore_hole.depth = 0;
        else if (els_bore_hole.depth + delta >= ELS_Z_MAX_MM)
          els_bore_hole.depth = ELS_Z_MAX_MM;
        else
          els_bore_hole.depth += delta;
        els_bore_hole.encoder_pos = encoder_curr;
        els_bore_hole_display_setting();
      }
      break;
  }
}

void els_bore_hole_set_width(void) {
  int32_t encoder_curr;
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
    case ELS_KEY_EXIT:
      els_bore_hole.state = ELS_BORE_HOLE_IDLE;
      els_bore_hole_display_setting();
      break;
    case ELS_KEY_FUN_F1:
      els_bore_hole.state = ELS_BORE_HOLE_SET_DEPTH;
      els_bore_hole_display_setting();
      break;
    default:
      encoder_curr = els_encoder_read();
      if (els_bore_hole.encoder_pos != encoder_curr) {
        double delta = (encoder_curr - els_bore_hole.encoder_pos) * 0.01;
        if (els_bore_hole.width + delta <= 0)
          els_bore_hole.width = 0;
        else if (els_bore_hole.width + delta >= ELS_X_MAX_MM)
          els_bore_hole.width = ELS_X_MAX_MM;
        else
          els_bore_hole.width += delta;
        els_bore_hole.encoder_pos = encoder_curr;
        els_bore_hole_display_setting();
      }
      break;
  }
}

// ----------------------------------------------------------------------------------
// Function 3: Axis - position, origin & jogging
// ----------------------------------------------------------------------------------
static void els_bore_hole_set_zaxes(void) {
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
    case ELS_KEY_EXIT:
      els_bore_hole.state = (els_bore_hole.state & ELS_BORE_HOLE_ZJOG) ? ELS_BORE_HOLE_SET_ZAXES : ELS_BORE_HOLE_IDLE;
      els_bore_hole_display_axes();
      break;
    case ELS_KEY_SET_ZX_ORI:
      if (els_bore_hole.state == ELS_BORE_HOLE_SET_ZAXES) {
        els_stepper_zero_z();
        els_dro_zero_z();
        els_bore_hole_display_axes();
      }
      break;
    case ELS_KEY_SET_ZX:
      els_bore_hole.state = ELS_BORE_HOLE_SET_XAXES;
      els_bore_hole_display_axes();
      break;
    default:
      els_bore_hole_zjog();
      break;
  }
}

static void els_bore_hole_set_xaxes(void) {
  switch(els_keypad_read()) {
    case ELS_KEY_OK:
    case ELS_KEY_EXIT:
      els_bore_hole.state = ELS_BORE_HOLE_IDLE;
      els_bore_hole_display_axes();
      break;
    case ELS_KEY_SET_ZX_ORI:
      if (els_bore_hole.state == ELS_BORE_HOLE_SET_XAXES) {
        els_stepper_zero_x();
        els_dro_zero_x();
        els_bore_hole_display_axes();
      }
      break;
    case ELS_KEY_SET_ZX:
      els_bore_hole.state = ELS_BORE_HOLE_SET_ZAXES;
      els_bore_hole_display_axes();
      break;
    default:
      els_bore_hole_xjog();
      break;
  }
}

// ----------------------------------------------------------------------------------
// Manual Jog
// ----------------------------------------------------------------------------------

static void els_bore_hole_zjog(void) {
  int32_t delta;
  int32_t encoder_curr;

  encoder_curr = els_encoder_read();
  if (els_bore_hole.encoder_pos != encoder_curr) {
    // ----------------------------------------------------------------------------------
    // Acceleration
    // ----------------------------------------------------------------------------------
    uint64_t now;
    static uint16_t accel = 0;
    static uint16_t velocity = 1;
    static uint64_t last_updated_at = 0;

    now = els_timer_elapsed_microseconds();
    if ((now - last_updated_at) < 1e5)
      accel++;
    else
      accel = 0;
    velocity = (accel > 10 ? 10 : 1);
    last_updated_at = now;

    delta = (encoder_curr - els_bore_hole.encoder_pos);
    els_bore_hole.encoder_pos = encoder_curr;
    els_stepper_move_z(delta * velocity * 0.1, ELS_Z_JOG_MM_S);
  }
}

static void els_bore_hole_xjog(void) {
  int32_t delta;
  int32_t encoder_curr;

  encoder_curr = els_encoder_read();
  if (els_bore_hole.encoder_pos != encoder_curr) {
    // ----------------------------------------------------------------------------------
    // Acceleration
    // ----------------------------------------------------------------------------------
    uint64_t now;
    static uint16_t accel = 0;
    static uint16_t velocity = 1;
    static uint64_t last_updated_at = 0;

    now = els_timer_elapsed_microseconds();
    if ((now - last_updated_at) < 1e5)
      accel++;
    else
      accel = 0;
    velocity = (accel > 10 ? 10 : 1);
    last_updated_at = now;

    delta = (encoder_curr - els_bore_hole.encoder_pos);
    els_bore_hole.encoder_pos = encoder_curr;
    els_stepper_move_x(delta * velocity * 0.01, ELS_X_JOG_MM_S);
  }
}