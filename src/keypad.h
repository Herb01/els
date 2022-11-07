#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>

void els_keypad_setup(void);
void els_ps2_write(bool writemode);
int  els_keypad_read(void);
void els_keypad_flush(void);
int  els_keypad_peek(void);
void els_keypad_write(uint8_t c);
void els_keypad_lock(void);
void els_keypad_unlock(void);
bool els_keypad_locked(void);
uint8_t getCurBits(void);

#ifdef __cplusplus
}
#endif

