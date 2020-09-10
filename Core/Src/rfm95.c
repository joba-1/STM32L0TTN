/*
  rfm95.c - ported to C on STM32 by Joachim Banzhaf 2020
  RFM95.cpp - Library for RFM95 LoRa module.
  Created by Leo Korbee, March 31, 2018.
  Released into the public domain.
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed on the base of this code.
  (Gerben den Hartog, et al - Ideetron.nl)
*/

#include "rfm95.h"


static uint32_t miState;

int32_t rand1( void )
{
    miState ^= (miState << 13);
    miState ^= (miState >> 17);
    miState ^= (miState << 15);

    return (miState * 1332534557) & 0x7FFFFFFF;
}

void srand1( uint32_t seed )
{
    // a zero seed will not work!
    if (seed == 0)
        seed = 0x55aaff01;

    miState = seed;
}


/*
*****************************************************************************************
* Description : Function that sets carrier frequency of the RFM
*
* Arguments   : dev:     id of the NSS pin in an array of port/pin structs
*               channel: id of channels array
*
* Returns   : nothing
*****************************************************************************************
*/

typedef struct freq { uint8_t hi, mid, lo; } freq_t;

void rfm95_freq( rfm95_t *dev, uint8_t channel ) {
  // EU863-870 specifications
  // FSK       868.800 Mhz => not used in this config
  // 869.525 - SF9BW125 (RX2 downlink only) for package received
  static const freq_t channels[] = {
    { 0xD9, 0x06, 0x8B }, // Channel 0 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
    { 0xD9, 0x13, 0x58 }, // Channel 1 868.300 MHz / 61.035 Hz = 14226264 = 0xD91358
    { 0xD9, 0x20, 0x24 }, // Channel 2 868.500 MHz / 61.035 Hz = 14229540 = 0xD92024
    { 0xD8, 0xC6, 0x8B }, // Channel 3 867.100 MHz / 61.035 Hz = 14206603 = 0xD8C68B
    { 0xD8, 0xD3, 0x58 }, // Channel 4 867.300 MHz / 61.035 Hz = 14209880 = 0xD8D358
    { 0xD8, 0xE0, 0x24 }, // Channel 5 867.500 MHz / 61.035 Hz = 14213156 = 0xD8E024
    { 0xD8, 0xEC, 0xF1 }, // Channel 6 867.700 MHz / 61.035 Hz = 14216433 = 0xD8ECF1
    { 0xD8, 0xF9, 0xBE }  // Channel 7 867.900 MHz / 61.035 Hz = 14219710 = 0xD8F9BE
  };

  if( channel < sizeof(channels)/sizeof(channels[0]) ) {
    freq_t data = channels[channel];
    (*dev->spi_write)(dev->nss_pin_id, 0x06 | 0x80, (uint8_t *)&data, sizeof(data));
  }
}


/*
*****************************************************************************************
* Description: Function used to initialize the RFM module on startup
*****************************************************************************************
*/
uint8_t rfm95_init( rfm95_t *dev, uint32_t seed ) {
  /*
   * Reminder:
   * set pin_nss as output in STM32 ioc target configuration file and connect the pin to the RFM95 NSS pin
   * set pin_dio0 as input in STM32 ioc target configuration file and connect the pin to the RFM95 DIO0 pin
   */

  srand1(seed);

  //Switch RFM to sleep
  rfm95_write(dev, 0x01, 0x00);

  //Set RFM in LoRa mode
  rfm95_write(dev, 0x01, 0x80);

  //Set RFM in Standby mode wait on mode ready
  rfm95_write(dev, 0x01, 0x81);

  // while (digitalRead(DIO5) == LOW);
  uint8_t max_wait = 200;
  while (max_wait && !(*dev->pin_read)(dev->dio5_pin_id) ) {
    max_wait--;
    (*dev->delay)(1);
  }
  if( !max_wait ) putstr("dio5! ");
  // (*dev->delay)(10);

  // while( rfm95_read(dev, 0x42) != 0x12 ); // check if we can communicate

  //Set carrier frequency
  // 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
  rfm95_freq(dev, 0);

  //PA pin (maximal power)
  rfm95_write(dev, 0x09, 0xFF);

  //BW = 125 kHz, Coding rate 4/5, Explicit header mode
  rfm95_write(dev, 0x1D, 0x72);

  //Spreading factor 7, PayloadCRC On
  rfm95_write(dev, 0x1E, 0xB4);

  //Rx Timeout set to 37 symbols
  rfm95_write(dev, 0x1F, 0x25);

  //Preamble length set to 8 symbols
  //0x0008 + 4 = 12
  rfm95_write(dev, 0x20, 0x00);
  rfm95_write(dev, 0x21, 0x08);

  //Low datarate optimization off AGC auto on
  rfm95_write(dev, 0x26, 0x0C);

  //Set LoRa sync word
  rfm95_write(dev, 0x39, 0x34);

  //Set IQ to normal values
  rfm95_write(dev, 0x33, 0x27);
  rfm95_write(dev, 0x3B, 0x1D);

  //Set FIFO pointers
  //TX base adress
  rfm95_write(dev, 0x0E, 0x80);
  //Rx base adress
  rfm95_write(dev, 0x0F, 0x00);

  uint8_t ver = rfm95_read(dev, 0x42);

  //Switch RFM to sleep
  rfm95_write(dev, 0x01, 0x00);

  return ver;
}


/*
*****************************************************************************************
* Description : Funtion that writes a register from the RFM
*
* Arguments   : RFM_Address Address of register to be written
*         RFM_Data    Data to be written
*****************************************************************************************
*/

uint8_t rfm95_write( rfm95_t *dev, uint8_t addr, uint8_t data ) {
  (*dev->spi_write)(dev->nss_pin_id, addr | 0x80, &data, sizeof(data));
  return data;
}


/*
*****************************************************************************************
* Description : Funtion that reads a register from the RFM and returns the value
*
* Arguments   : RFM_Address Address of register to be read
*
* Returns   : Value of the register
*****************************************************************************************
*/

uint8_t rfm95_read( rfm95_t *dev, uint8_t addr ) {
  uint8_t data;
  (*dev->spi_read)(dev->nss_pin_id, addr & ~0x80, &data, sizeof(data));
  return data;
}


/*
*****************************************************************************************
* Description : Function for sending a package with the RFM
*
* Arguments   : buffer:  Pointer to array with data to be send
*               len:     Length of the package to send
*****************************************************************************************
*/

uint32_t rfm95_send( rfm95_t *dev, uint8_t *buffer, uint32_t len ) {
  // unsigned char RFM_Tx_Location = 0x00;

  if( len == 0 || len > 64 ) {
	  return 0; // Nothing to send or fifo length exceeded
  }

  //Set RFM in Standby mode wait on mode ready
  rfm95_write(dev, 0x01, 0x81);

  // while (digitalRead(DIO5) == LOW);
  //Wait for TxDone
  uint8_t max_wait = 200;
  while (max_wait && !(*dev->pin_read)(dev->dio5_pin_id) ) {
    max_wait--;
    (*dev->delay)(1);
  }
  if( !max_wait ) putstr("dio5! ");
  // (*dev->delay)(10);

  //Switch DIO0 to TxDone
  rfm95_write(dev, 0x40, 0x40);

  // while( rfm95_read(dev, 0x42) != 0x12 ); // check if we can communicate

  //Set carrier frequency

  /*
  fixed frequency
  // 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
  _rfm95.RFM_Write(0x06,0xD9);
  _rfm95.RFM_Write(0x07,0x06);
  _rfm95.RFM_Write(0x08,0x8B);
  */

  rfm95_freq(dev, rand1() % 8);

  //SF7 BW 125 kHz
  rfm95_write(dev, 0x1E, 0x74); //SF7 CRC On
  rfm95_write(dev, 0x1D, 0x72); //125 kHz 4/5 coding rate explicit header mode
  rfm95_write(dev, 0x26, 0x04); //Low datarate optimization off AGC auto on

  //Set IQ to normal values
  rfm95_write(dev, 0x33, 0x27);
  rfm95_write(dev, 0x3B, 0x1D);

  //Set payload length to the right length
  rfm95_write(dev, 0x22, len);

  //Get location of Tx part of FiFo
  //RFM_Tx_Location = RFM_Read(0x0E);

  //Set SPI pointer to start of Tx part in FiFo
  //RFM_Write(0x0D,RFM_Tx_Location);
  rfm95_write(dev, 0x0D, 0x80); // hardcoded fifo location according RFM95 specs

  //Write Payload to FiFo
  while (len--) {
    rfm95_write(dev, 0x00, *(buffer++));
  }

  //Switch RFM to Tx
  rfm95_write(dev, 0x01, 0x83);

  //Wait for TxDone
  max_wait = 200;
  while (max_wait && !(*dev->pin_read)(dev->dio0_pin_id) ) {
    max_wait--;
    (*dev->delay)(1);
  }
  if( !max_wait ) putstr("dio0! ");

  //What's the frequency, kenneth?
  uint32_t freq = rfm95_read(dev, 0x06);
  freq <<= 8;
  freq |= rfm95_read(dev, 0x07);
  freq <<= 8;
  freq |= rfm95_read(dev, 0x08);
  // multiply frequency with 61.035 without 32bit overflow
  freq *= 195;
  freq /= 1000;
  freq *= 313;

  //Switch RFM to sleep
  rfm95_write(dev, 0x01, 0x00);

  return freq;
}
