/*
  LoRaWAN.h - Library header file for LoRaWAN protocol, uses RFM95W module
  Ported to STM32L011 C by Joachim Banzhaf, March 28, 2020.
  Created by Leo Korbee, March 31, 2018.
  Released into the public domain.
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed before me on this code.
*/

#ifndef LoRaWAN_h
#define LoRaWAN_h

#include "rfm95.h"

typedef struct lorawan {
  rfm95_t *rfm95;
  uint8_t *NwkSkey;
  uint8_t *AppSkey;
  uint8_t *DevAddr;
} lorawan_t;

void lorawan_init(lorawan_t *lorawan, rfm95_t *rfm95);
void lorawan_set_keys(lorawan_t *lorawan, uint8_t NwkSkey[], uint8_t AppSkey[], uint8_t DevAddr[]);
uint32_t lorawan_send_data(lorawan_t *lorawan, uint8_t *data, unsigned len, uint16_t frame_counter_up); // returns freqency

#endif
