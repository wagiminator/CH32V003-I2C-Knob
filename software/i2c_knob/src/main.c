// ===================================================================================
// Project:   Rotary Encoder with I2C Interface based on CH32V003
// Version:   v1.0
// Year:      2024
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// This is a simple implementation of a rotary encoder with I2C interface.
//
// References:
// -----------
// - CNLohr ch32v003fun: https://github.com/cnlohr/ch32v003fun
// - WCH Nanjing Qinheng Microelectronics: http://wch.cn
//
// Compilation Instructions:
// -------------------------
// - Make sure GCC toolchain (gcc-riscv64-unknown-elf, newlib) and Python3 with
//   rvprog (via pip) are installed. In addition, Linux requires access rights to
//   WCH-LinkE programmer.
// - Connect the WCH-LinkE programmer to the MCU board.
// - Run 'make flash'.
//
// Operating Instructions:
// -----------------------
// The device has four 16-bit and two 8-bit registers that can be read and written.
// The 16-bit registers are signed and the least significant byte is always
// transmitted first. With each access (reading or writing), the registers are 
// always transferred starting with the first in the following order:
// 1. Encoder wheel value (16-bit)
// 2. Encoder switch state (8-bit, 0=switch released, 1=switch pressed)
// 3. Encoder wheel value loop flag (8-bit, 0=do not loop, 1=loop around)
// 4. Encoder wheel minimum value (16-bit)
// 5. Encoder wheel maximum value (16-bit)
// 6. Encoder wheel value change step (16-bit)


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================
#include <config.h>                     // user configurations
#include <system.h>                     // system functions
#include <gpio.h>                       // GPIO functions
#include <i2c_slave.h>                  // I2C slave functions

// ===================================================================================
// Rotary Encoder Implementation
// ===================================================================================

// Global variables
volatile uint8_t  ENC_a0, ENC_b0, ENC_ab0, ENC_loop, ENC_changed;
volatile int16_t  ENC_count, ENC_countMin, ENC_countMax, ENC_countStep;

// Init rotary encoder
void ENC_init(void) {
  PIN_input_PU(PIN_ENC_A);              // enable pullup on encoder pins ...
  PIN_input_PU(PIN_ENC_B);
  PIN_input_PU(PIN_ENC_SW);
}

// Set parameters for rotary encoder
void ENC_set(int16_t rmin, int16_t rmax, int16_t rstep, int16_t rval, uint8_t rloop) {
  ENC_countMin  = rmin << 1;            // min wheel value
  ENC_countMax  = rmax << 1;            // max wheel value
  ENC_countStep = rstep;                // wheel steps (negative if CCW)
  ENC_count     = rval << 1;            // actual wheel value
  ENC_loop      = rloop;                // loop if true
  ENC_a0  = !PIN_read(PIN_ENC_A);       // set initial internal values ...
  ENC_b0  = !PIN_read(PIN_ENC_B);
  ENC_ab0 = (ENC_a0 == ENC_b0);
}

// Read encoder and update values, returns 0 if no change
uint8_t ENC_read(void) {
  uint8_t a = !PIN_read(PIN_ENC_A);     // get A state
  uint8_t b = !PIN_read(PIN_ENC_B);     // get B state
  if(a != ENC_a0) {                     // A changed?
    ENC_a0 = a;
    if(b != ENC_b0) {                   // B changed?
      ENC_b0 = b;
      ENC_count += (a == b) ? -ENC_countStep : ENC_countStep;
      if((a == b) != ENC_ab0) ENC_count += (a == b) ? -ENC_countStep : ENC_countStep;
      if(ENC_count < ENC_countMin) ENC_count = (ENC_loop) ? ENC_countMax : ENC_countMin;
      if(ENC_count > ENC_countMax) ENC_count = (ENC_loop) ? ENC_countMin : ENC_countMax;
      ENC_ab0 = (a == b);
      ENC_changed = 1;
    }
  }
  return ENC_changed;
}

// ===================================================================================
// Main Function
// ===================================================================================
int main(void) {
  // Local variables
  int16_t rval, rmin, rmax, rstep;          // for handling encoder parameters

  // Setup
  I2C_init();                               // setup I2C
  ENC_init();                               // setup rotary encoder

  // Loop
  while(1) {
    // Update rotary encoder settings if I2C registers have changed
    if(I2C_changed() && !I2C_busy()) {
      INT_disable();                        // disable interrupts for atomic ops
      rval  = ((uint16_t)I2C_REG[1] << 8) | (uint16_t)I2C_REG[0];
      rmin  = ((uint16_t)I2C_REG[5] << 8) | (uint16_t)I2C_REG[4];
      rmax  = ((uint16_t)I2C_REG[7] << 8) | (uint16_t)I2C_REG[6];
      rstep = ((uint16_t)I2C_REG[9] << 8) | (uint16_t)I2C_REG[8];
      I2C_clear();                          // clear register changed flag
      INT_enable();                         // enable interrupts again
      ENC_set(rmin, rmax, rstep, rval, I2C_REG[3]); // set rotary encoder
    }

    // Update I2C registers if rotary encoder wheel value has changed
    if(ENC_read() && !I2C_changed() && !I2C_busy()) {
      INT_disable();                        // disable interrupts for atomic ops
      rval = ENC_count >> 1;                // get encoder counter value
      I2C_REG[0] = (uint8_t)(rval & 0xff);  // store low byte to register
      I2C_REG[1] = (uint8_t)(rval >> 8);    // store high byte to register
      ENC_changed = 0;                      // clear encoder changed flag
      INT_enable();                         // enable interrupts again
    }

    // Update rotary encoder switch register
    I2C_REG[2] = !PIN_read(PIN_ENC_SW);
  }
}
