/*
  LoRaWAN.cpp - Library for LoRaWAN protocol, uses RFM95W module
  Ported to STM32L011 C by Joachim Banzhaf, March 28, 2020.
  Created by Leo Korbee, March 31, 2018.
  Released into the public domain.
  @license Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  Thanks to all the folks who contributed on the base of this code.
  (Gerben den Hartog, et al - Ideetron.nl)
*/

#include "lorawan.h"

#include <string.h>

// security stuff:
static void lorawan_encrypt_payload(lorawan_t *lorawan, uint8_t *data, unsigned len, uint16_t frame_counter, unsigned char direction);
static void lorawan_calculate_mic(lorawan_t *lorawan, uint8_t *data, uint8_t *final_mic, unsigned len, uint16_t frame_counter, unsigned char direction);
static void lorawan_generate_keys(lorawan_t *lorawan, uint8_t *k1, uint8_t *k2);
static void lorawan_shift_left(lorawan_t *lorawan, uint8_t *data);
static void lorawan_xor(lorawan_t *lorawan, uint8_t *new_data, uint8_t *old_data);
static void lorawan_aes_encrypt(lorawan_t *lorawan, uint8_t *data, uint8_t *key);
static void lorawan_aes_add_round_key(lorawan_t *lorawan, uint8_t *round_key, uint8_t (*state)[4]);
static uint8_t lorawan_aes_sub_byte(lorawan_t *lorawan, uint8_t byte);
static void lorawan_aes_shift_rows(lorawan_t *lorawan, uint8_t (*state)[4]);
static void lorawan_aes_mix_columns(lorawan_t *lorawan, uint8_t (*state)[4]);
static void lorawan_aes_calculate_round_key(lorawan_t *lorawan, uint8_t round, uint8_t *round_key);

// for AES encryption
static const unsigned char S_Table[16][16] = {
  {0x63,0x7C,0x77,0x7B,0xF2,0x6B,0x6F,0xC5,0x30,0x01,0x67,0x2B,0xFE,0xD7,0xAB,0x76},
  {0xCA,0x82,0xC9,0x7D,0xFA,0x59,0x47,0xF0,0xAD,0xD4,0xA2,0xAF,0x9C,0xA4,0x72,0xC0},
  {0xB7,0xFD,0x93,0x26,0x36,0x3F,0xF7,0xCC,0x34,0xA5,0xE5,0xF1,0x71,0xD8,0x31,0x15},
  {0x04,0xC7,0x23,0xC3,0x18,0x96,0x05,0x9A,0x07,0x12,0x80,0xE2,0xEB,0x27,0xB2,0x75},
  {0x09,0x83,0x2C,0x1A,0x1B,0x6E,0x5A,0xA0,0x52,0x3B,0xD6,0xB3,0x29,0xE3,0x2F,0x84},
  {0x53,0xD1,0x00,0xED,0x20,0xFC,0xB1,0x5B,0x6A,0xCB,0xBE,0x39,0x4A,0x4C,0x58,0xCF},
  {0xD0,0xEF,0xAA,0xFB,0x43,0x4D,0x33,0x85,0x45,0xF9,0x02,0x7F,0x50,0x3C,0x9F,0xA8},
  {0x51,0xA3,0x40,0x8F,0x92,0x9D,0x38,0xF5,0xBC,0xB6,0xDA,0x21,0x10,0xFF,0xF3,0xD2},
  {0xCD,0x0C,0x13,0xEC,0x5F,0x97,0x44,0x17,0xC4,0xA7,0x7E,0x3D,0x64,0x5D,0x19,0x73},
  {0x60,0x81,0x4F,0xDC,0x22,0x2A,0x90,0x88,0x46,0xEE,0xB8,0x14,0xDE,0x5E,0x0B,0xDB},
  {0xE0,0x32,0x3A,0x0A,0x49,0x06,0x24,0x5C,0xC2,0xD3,0xAC,0x62,0x91,0x95,0xE4,0x79},
  {0xE7,0xC8,0x37,0x6D,0x8D,0xD5,0x4E,0xA9,0x6C,0x56,0xF4,0xEA,0x65,0x7A,0xAE,0x08},
  {0xBA,0x78,0x25,0x2E,0x1C,0xA6,0xB4,0xC6,0xE8,0xDD,0x74,0x1F,0x4B,0xBD,0x8B,0x8A},
  {0x70,0x3E,0xB5,0x66,0x48,0x03,0xF6,0x0E,0x61,0x35,0x57,0xB9,0x86,0xC1,0x1D,0x9E},
  {0xE1,0xF8,0x98,0x11,0x69,0xD9,0x8E,0x94,0x9B,0x1E,0x87,0xE9,0xCE,0x55,0x28,0xDF},
  {0x8C,0xA1,0x89,0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16}
};


void lorawan_init(lorawan_t *lorawan, rfm95_t *rfm95) {
  lorawan->rfm95 = rfm95;
}


void lorawan_set_keys(lorawan_t *lorawan, uint8_t NwkSkey[], uint8_t AppSkey[], uint8_t DevAddr[]) {
  lorawan->NwkSkey = NwkSkey;
  lorawan->AppSkey = AppSkey;
  lorawan->DevAddr = DevAddr;
}


/*
*****************************************************************************************
* Description : Function contstructs a LoRaWAN package and sends it
*
* Arguments   : *Data pointer to the array of data that will be transmitted
*               Data_Length nuber of bytes to be transmitted
*               Frame_Counter_Up  Frame counter of upstream frames
*
*****************************************************************************************
*/

// Had issues with RAM when RFM_Data[64] from Send_Data() and 
// Block_A[16] from Encrypt_Payload() where allocated on the stack
// at the same time. Moving to global and share solved it.
// Works because the arrays are never used at the same time
// and always get initialized with new data
unsigned char Buffer[64];

uint32_t lorawan_send_data(lorawan_t *lorawan, uint8_t *data, unsigned len, uint16_t frame_counter_up) {
  //Define variables
  unsigned char i;

  // Test flag: if all data[i] == i then don't send but print encrypted data
  int is_test = 1;
  for( i=0; i<len; i++ ) {
    if( data[i] != i ) {
      is_test = 0;
      break;
    }
  }

  //Direction of frame is up
  unsigned char Direction = 0x00;

  // shared RAM with Encrypt_Payload()  
  unsigned char *RFM_Data = Buffer;
  unsigned char RFM_Package_Length;

  unsigned char MIC[4];

  /*
    @leo:
    https://hackmd.io/s/S1kg6Ymo-

    7…5 bits	4…2 bits	1…0 bits
    MType	    RFU	      Major

    MType	Description
    000	(0x00) Join Request
    001	(0x20) Join Accept
    010	(0x40) Unconfirmed Data Up
    011	(0x60) Unconfirmed Data Down
    100	(0x80) Confirmed Data Up
    101	(0xA0) Confirmed Data Down
    110	(0xC0) RFU
    111	(0xE0) Proprietary
  */

  // Unconfirmed data up
  unsigned char Mac_Header = 0x40;

  // Confirmed data up
  // unsigned char Mac_Header = 0x80;

  unsigned char Frame_Control = 0x00;
  unsigned char Frame_Port = 0x01;

  //Encrypt the data
  lorawan_encrypt_payload(lorawan, data, len, frame_counter_up, Direction);

  //Build the Radio Package
  RFM_Data[0] = Mac_Header;

  RFM_Data[1] = lorawan->DevAddr[3];
  RFM_Data[2] = lorawan->DevAddr[2];
  RFM_Data[3] = lorawan->DevAddr[1];
  RFM_Data[4] = lorawan->DevAddr[0];

  RFM_Data[5] = Frame_Control;

  RFM_Data[6] = (frame_counter_up & 0x00FF);
  RFM_Data[7] = ((frame_counter_up >> 8) & 0x00FF);

  RFM_Data[8] = Frame_Port;

  //Set Current package length
  RFM_Package_Length = 9;

  //Load Data
  for(i = 0; i < len; i++) {
    RFM_Data[RFM_Package_Length + i] = data[i];
  }

  //Add data length to package length
  RFM_Package_Length = RFM_Package_Length + len;

  //Calculate MIC
  lorawan_calculate_mic(lorawan, RFM_Data, MIC, RFM_Package_Length, frame_counter_up, Direction);

  //Load MIC in package
  for(i = 0; i < 4; i++) {
    RFM_Data[i + RFM_Package_Length] = MIC[i];
  }

  //Add MIC length to RFM package length
  RFM_Package_Length += 4;

  // Print test package
  if( is_test ) {
    putstr(" encrypted:");
    for( i=0; i<RFM_Package_Length; i++ ) {
      puthex(RFM_Data[i]);
    }
    return 0;
  }

  //Send Package
  return rfm95_send(lorawan->rfm95, RFM_Data, RFM_Package_Length);
}


/*
   Encryption stuff after this line
*/
void lorawan_encrypt_payload(lorawan_t *lorawan, uint8_t *data, unsigned len, uint16_t frame_counter, unsigned char direction) {
  unsigned char i = 0x00;
  unsigned char j;
  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;

  // shared RAM with Send_Data()  
  unsigned char *Block_A = Buffer;

  //Calculate number of blocks
  Number_of_Blocks = len / 16;
  Incomplete_Block_Size = len % 16;
  if(Incomplete_Block_Size != 0) {
    Number_of_Blocks++;
  }

  for(i = 1; i <= Number_of_Blocks; i++) {
    Block_A[0] = 0x01;
    Block_A[1] = 0x00;
    Block_A[2] = 0x00;
    Block_A[3] = 0x00;
    Block_A[4] = 0x00;

    Block_A[5] = direction;

    Block_A[6] = lorawan->DevAddr[3];
    Block_A[7] = lorawan->DevAddr[2];
    Block_A[8] = lorawan->DevAddr[1];
    Block_A[9] = lorawan->DevAddr[0];

    Block_A[10] = (frame_counter & 0x00FF);
    Block_A[11] = ((frame_counter >> 8) & 0x00FF);
    Block_A[12] = 0; // ((frame_counter >> 16) & 0x000000FF);
    Block_A[13] = 0; // ((frame_counter >> 24) & 0x000000FF);

    Block_A[14] = 0x00;

    Block_A[15] = i;

    //Calculate S
    lorawan_aes_encrypt(lorawan, Block_A, lorawan->AppSkey); //original

    //Check for last block
    if(i != Number_of_Blocks) {
      for(j = 0; j < 16; j++) {
        *data ^= Block_A[j];
        data++;
      }
    }
    else
    {
      if(Incomplete_Block_Size == 0) {
        Incomplete_Block_Size = 16;
      }
      for(j = 0; j < Incomplete_Block_Size; j++) {
        *data ^= Block_A[j];
        data++;
      }
    }
  }
}


// using global variables and initialize them at function start saves precious flash space.
unsigned char Key_K1[16];
unsigned char Key_K2[16];
unsigned char Old_Data[16];
unsigned char New_Data[16];
unsigned char Block_B[16];

void lorawan_calculate_mic(lorawan_t *lorawan, uint8_t *data, uint8_t *final_mic, unsigned len, uint16_t frame_counter, unsigned char direction) {
  unsigned char i;

  memset(Key_K1, 0, sizeof(Key_K1));
  memset(Key_K2, 0, sizeof(Key_K2));

  //unsigned char Data_Copy[16];

  memset(Old_Data, 0, sizeof(Old_Data));
  memset(New_Data, 0, sizeof(New_Data));

  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;
  unsigned char Block_Counter = 0x01;

  //Create Block_B
  Block_B[0] = 0x49;
  Block_B[1] = 0x00;
  Block_B[2] = 0x00;
  Block_B[3] = 0x00;
  Block_B[4] = 0x00;

  Block_B[5] = direction;

  Block_B[6] = lorawan->DevAddr[3];
  Block_B[7] = lorawan->DevAddr[2];
  Block_B[8] = lorawan->DevAddr[1];
  Block_B[9] = lorawan->DevAddr[0];

  Block_B[10] = (frame_counter & 0x00FF);
  Block_B[11] = ((frame_counter >> 8) & 0x00FF);

  Block_B[12] = 0x00; //Frame counter upper bytes
  Block_B[13] = 0x00;

  Block_B[14] = 0x00;
  Block_B[15] = len;

  //Calculate number of Blocks and blocksize of last block
  Number_of_Blocks = len / 16;
  Incomplete_Block_Size = len % 16;

  if(Incomplete_Block_Size != 0) {
    Number_of_Blocks++;
  }

  lorawan_generate_keys(lorawan, Key_K1, Key_K2);

  //Perform Calculation on Block B0

  //Perform AES encryption
  lorawan_aes_encrypt(lorawan, Block_B, lorawan->NwkSkey);

  //Copy Block_B to Old_Data
  for(i = 0; i < 16; i++) {
    Old_Data[i] = Block_B[i];
  }

  //Perform full calculating until n-1 messsage blocks
  while(Block_Counter < Number_of_Blocks) {
    //Copy data into array
    for(i = 0; i < 16; i++) {
      New_Data[i] = *data;
      data++;
    }

    //Perform XOR with old data
    lorawan_xor(lorawan, New_Data, Old_Data);

    //Perform AES encryption
    lorawan_aes_encrypt(lorawan, New_Data, lorawan->NwkSkey);

    //Copy New_Data to Old_Data
    for(i = 0; i < 16; i++) {
      Old_Data[i] = New_Data[i];
    }

    //Raise Block counter
    Block_Counter++;
  }

  //Perform calculation on last block
  //Check if data length is a multiple of 16
  if(Incomplete_Block_Size == 0) {
    //Copy last data into array
    for(i = 0; i < 16; i++) {
      New_Data[i] = *data;
      data++;
    }

    //Perform XOR with Key 1
    lorawan_xor(lorawan, New_Data, Key_K1);

    //Perform XOR with old data
    lorawan_xor(lorawan, New_Data, Old_Data);

    //Perform last AES routine
    // read NwkSkey
    lorawan_aes_encrypt(lorawan, New_Data, lorawan->NwkSkey);
  }
  else {
    //Copy the remaining data and fill the rest
    for(i =  0; i < 16; i++) {
      if(i < Incomplete_Block_Size) {
        New_Data[i] = *data;
        data++;
      }
      if(i == Incomplete_Block_Size) {
        New_Data[i] = 0x80;
      }
      if(i > Incomplete_Block_Size) {
        New_Data[i] = 0x00;
      }
    }

    //Perform XOR with Key 2
    lorawan_xor(lorawan, New_Data, Key_K2);

    //Perform XOR with Old data
    lorawan_xor(lorawan, New_Data, Old_Data);

    //Perform last AES routine
    lorawan_aes_encrypt(lorawan, New_Data, lorawan->NwkSkey);
  }

  final_mic[0] = New_Data[0];
  final_mic[1] = New_Data[1];
  final_mic[2] = New_Data[2];
  final_mic[3] = New_Data[3];
}


void lorawan_generate_keys(lorawan_t *lorawan, uint8_t *K1, uint8_t *K2) {
  unsigned char i;
  unsigned char MSB_Key;

  //Encrypt the zeros in K1 with the NwkSkey
  lorawan_aes_encrypt(lorawan, K1, lorawan->NwkSkey);

  //Create K1
  //Check if MSB is 1
  if((K1[0] & 0x80) == 0x80) {
    MSB_Key = 1;
  }
  else {
    MSB_Key = 0;
  }

  //Shift K1 one bit left
  lorawan_shift_left(lorawan, K1);

  //if MSB was 1
  if(MSB_Key == 1) {
    K1[15] ^= 0x87;
  }

  //Copy K1 to K2
  for( i = 0; i < 16; i++) {
    K2[i] = K1[i];
  }

  //Check if MSB is 1
  if((K2[0] & 0x80) == 0x80) {
    MSB_Key = 1;
  }
  else {
    MSB_Key = 0;
  }

  //Shift K2 one bit left
  lorawan_shift_left(lorawan, K2);

  //Check if MSB was 1
  if(MSB_Key == 1) {
    K2[15] ^= 0x87;
  }
}


void lorawan_shift_left(lorawan_t *lorawan, uint8_t *data) {
  unsigned char i;
  unsigned char Overflow = 0;
  //unsigned char High_Byte, Low_Byte;

  for(i = 0; i < 16; i++) {
    //Check for overflow on next byte except for the last byte
    if(i < 15) {
      //Check if upper bit is one
      if((data[i+1] & 0x80) == 0x80) {
        Overflow = 1;
      }
      else {
        Overflow = 0;
      }
    }
    else {
      Overflow = 0;
    }

    //Shift one left
    data[i] = (data[i] << 1) + Overflow;
  }
}


void lorawan_xor(lorawan_t *lorawan, uint8_t *new_data, uint8_t *old_data) {
  unsigned char i;

  for(i = 0; i < 16; i++) {
    new_data[i] ^= old_data[i];
  }
}


/*
*****************************************************************************************
* Title         : AES_Encrypt
* Description  :
*****************************************************************************************
*/
void lorawan_aes_encrypt(lorawan_t *lorawan, uint8_t *data, uint8_t *key) {
  unsigned char Row, Column, Round = 0;
  unsigned char Round_Key[16];
  unsigned char State[4][4];

  // Copy input to State array
  for( Column = 0; Column < 4; Column++ ) {
    for( Row = 0; Row < 4; Row++ ) {
      State[Row][Column] = data[Row + (Column << 2)];
    }
  }

  // Copy key to round key
  memcpy(Round_Key, key, 16);

  // Add round key
  lorawan_aes_add_round_key(lorawan, Round_Key, State);

  // Perform 9 full rounds with mixed columns
  for( Round = 1 ; Round < 10 ; Round++ ) {
    //  Perform Byte substitution with S table
    for( Column = 0 ; Column < 4 ; Column++ ) {
      for( Row = 0 ; Row < 4 ; Row++ ) {
        State[Row][Column] = lorawan_aes_sub_byte(lorawan, State[Row][Column]);
      }
    }

    //  Perform Row Shift
    lorawan_aes_shift_rows(lorawan, State);

    //  Mix Collums
    lorawan_aes_mix_columns(lorawan, State);

    //  Calculate new round key
    lorawan_aes_calculate_round_key(lorawan, Round, Round_Key);

        //  Add the round key to the Round_key
    lorawan_aes_add_round_key(lorawan, Round_Key, State);
  }

  //  Perform byte substitution with S table without mix columns
  for( Column = 0 ; Column < 4 ; Column++ ) {
    for( Row = 0; Row < 4; Row++ ) {
      State[Row][Column] = lorawan_aes_sub_byte(lorawan, State[Row][Column]);
    }
  }

  //  Shift rows
  lorawan_aes_shift_rows(lorawan, State);

  //  Calculate new round key
  lorawan_aes_calculate_round_key(lorawan, Round, Round_Key);

    //  Add round key
  lorawan_aes_add_round_key(lorawan, Round_Key, State);

  //  Copy the State into the data array
  for( Column = 0; Column < 4; Column++ ) {
    for( Row = 0; Row < 4; Row++ ) {
      data[Row + (Column << 2)] = State[Row][Column];
    }
  }
} // AES_Encrypt


/*
*****************************************************************************************
* Title         : AES_Add_Round_Key
* Description :
*****************************************************************************************
*/
void lorawan_aes_add_round_key(lorawan_t *lorawan, uint8_t *Round_Key, uint8_t (*State)[4]) {
  unsigned char Row, Column;

  for(Column = 0; Column < 4; Column++) {
    for(Row = 0; Row < 4; Row++) {
      State[Row][Column] ^= Round_Key[Row + (Column << 2)];
    }
  }
} // AES_Add_Round_Key


/*
*****************************************************************************************
* Title         : AES_Sub_Byte
* Description :
*****************************************************************************************
*/
uint8_t lorawan_aes_sub_byte(lorawan_t *lorawan, uint8_t byte) {
//  unsigned char S_Row,S_Column;
//  unsigned char S_Byte;
//
//  S_Row    = ((Byte >> 4) & 0x0F);
//  S_Column = ((Byte >> 0) & 0x0F);
//  S_Byte   = S_Table [S_Row][S_Column];

  return S_Table [ ((byte >> 4) & 0x0F) ] [ ((byte >> 0) & 0x0F) ]; // original
  //return pgm_read_byte(&(S_Table [((byte >> 4) & 0x0F)] [((byte >> 0) & 0x0F)]));
} //    AES_Sub_Byte


/*
*****************************************************************************************
* Title         : AES_Shift_Rows
* Description :
*****************************************************************************************
*/
void lorawan_aes_shift_rows(lorawan_t *lorawan, uint8_t (*state)[4]) {
  unsigned char Buffer;

  //Store firt byte in buffer
  Buffer      = state[1][0];
  //Shift all bytes
  state[1][0] = state[1][1];
  state[1][1] = state[1][2];
  state[1][2] = state[1][3];
  state[1][3] = Buffer;

  Buffer      = state[2][0];
  state[2][0] = state[2][2];
  state[2][2] = Buffer;
  Buffer      = state[2][1];
  state[2][1] = state[2][3];
  state[2][3] = Buffer;

  Buffer      = state[3][3];
  state[3][3] = state[3][2];
  state[3][2] = state[3][1];
  state[3][1] = state[3][0];
  state[3][0] = Buffer;
}   //  AES_Shift_Rows


/*
*****************************************************************************************
* Title         : AES_Mix_Columns
* Description :
*****************************************************************************************
*/
void lorawan_aes_mix_columns(lorawan_t *lorawan, uint8_t (*state)[4]) {
  uint8_t Row, Column;
  uint8_t a[4], b[4];

  for(Column = 0; Column < 4; Column++) {
    for(Row = 0; Row < 4; Row++) {
      a[Row] =  state[Row][Column];
      b[Row] = (state[Row][Column] << 1);

      if((state[Row][Column] & 0x80) == 0x80) {
        b[Row] ^= 0x1B;
      }
    }

    state[0][Column] = b[0] ^ a[1] ^ b[1] ^ a[2] ^ a[3];
    state[1][Column] = a[0] ^ b[1] ^ a[2] ^ b[2] ^ a[3];
    state[2][Column] = a[0] ^ a[1] ^ b[2] ^ a[3] ^ b[3];
    state[3][Column] = a[0] ^ b[0] ^ a[1] ^ a[2] ^ b[3];
  }
}   //  AES_Mix_Collums


/*
*****************************************************************************************
* Title         : AES_Calculate_Round_Key
* Description :
*****************************************************************************************
*/
void lorawan_aes_calculate_round_key(lorawan_t *lorawan, uint8_t round, uint8_t *round_key) {
  uint8_t i, j, b, Rcon;
  uint8_t Temp[4];

  //Calculate Rcon
  Rcon = 0x01;
  while(round != 1) {
    b = Rcon & 0x80;
    Rcon = Rcon << 1;

    if(b == 0x80) {
      Rcon ^= 0x1b;
    }
    round--;
  }

  //  Calculate first Temp
  //  Copy last byte from previous key and substitute the byte, but shift the array contents around by 1.
  Temp[0] = lorawan_aes_sub_byte(lorawan, round_key[12 + 1]);
  Temp[1] = lorawan_aes_sub_byte(lorawan, round_key[12 + 2]);
  Temp[2] = lorawan_aes_sub_byte(lorawan, round_key[12 + 3]);
  Temp[3] = lorawan_aes_sub_byte(lorawan, round_key[12 + 0]);

  //  XOR with Rcon
  Temp[0] ^= Rcon;

  //  Calculate new key
  for(i = 0; i < 4; i++) {
    for(j = 0; j < 4; j++) {
      round_key[j + (i << 2)] ^= Temp[j];
      Temp[j]                  = round_key[j + (i << 2)];
    }
  }
}   //  AES_Calculate_Round_Key
