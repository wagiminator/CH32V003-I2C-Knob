// ===================================================================================
// User Configurations for I2C Rotary Encoder
// ===================================================================================
//
// Description:
// ------------
// This is a simple implementation of the rotary encoder with I2C interface.
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

#pragma once

// Pin definitions
#define PIN_ENC_A     PA2       // connected to rotary encoder A (active low)
#define PIN_ENC_B     PA1       // connected to rotary encoder B (active low)
#define PIN_ENC_SW    PC4       // connected to rotary encoder switch (active low)
#define PIN_SDA       PC1       // connect to I2C SDA (do not change!)
#define PIN_SCL       PC2       // connect to I2C SCL (do not change!)

// I2C definitions
#define I2C_ADDR      0x36      // I2C address of the device
