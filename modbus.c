/**
 * File:
 *  modbus.c
 *
 * Notes:
 *  This file contains common modbus functions that are utilised by both slave
 * and master, serial and TCP implementations.
 *
 * Functions:
 *  addModbusBlock    Creates an I/O block of a specified type
 *  calcCRC           Calculates the CRC for the message content
 *  modbusSerialInit  Initialise serial port
 *  serviceIOBlocks   Checks I/O blocks, if update flag set, calls callback
 *  startTimeout      Starts the message timeout timer
 *  restartRx         Restart communications
 *
 * History:
 *  09/07/2012 Written by Simon Platten
 */
#include <stdarg.h>
#include <built_in.h>

#include "modbus.h"
// Pointer to the I/O linked lists
modbusBlockDef* pCoils       = NULL;
modbusBlockDef* pStatusBits  = NULL;
modbusBlockDef* pHoldingRegs = NULL;
modbusBlockDef* pInputRegs   = NULL;
#ifdef MODBUS_MASTER
// Pointer to the last block that was addressed
modbusBlockDef* pCurrBlock;
#endif
// Exception code
mbException eMbExceptionCode;
// Receiver & transmitter GAP set-points
uint uintMbRxGapSetPt1;
uint uintMbRxGapSetPt2;
uint uintMbRxGapSetPt3;
uint uintMbRxGapSetPt;
// Packet timeout in milliseconds
uint uintPacketTimeout;
// Calculated CRC
uint uintCRC;
// Receiver GAP counter
uint uintMbRxGap;
// The modbus slave address
byte bytMbSlaveAddress = 0;
// Received packet buffer
byte arybytMbBuffer[MAX_PACKET_LENGTH];
// The byte index
byte bytMbIndex = 0;
byte bytMbRxphase;
// RS-485 direction control bit
//sbit Tx_dir at LATE.B0;
/**
 * Function:
 *  addModbusBlock
 *
 * Parameters:
 *  bytSlaveAddress, the modbus slave address 1 to 247
 *  eType, see modbusSlave.h mbType for options
 *  pBlock, the block to set-up
 *  uintAddress, the start address base 1
 *  uintTotal, the total number of I/O in the block
 *  paryData, a pointer to the actual data array,
 *              for discretes this must be an array of bytes
 *              for registers this must be an array of uints
 *  pCallback, optional call-back routine called when block is updated
 *
 * Returns:
 *  TRUE if block added, FALSE if not
 */
boolean addModbusBlock(byte bytSlaveAddress,
                       mbType eType,
                       modbusBlockDef* pBlock,
                       uint uintAddress,
                       uint uintTotal,
                       void* paryData,
#ifdef MODBUS_MASTER
                       void (*pCallback)()
#elif defined MODBUS_SLAVE
                       void (*pCallback)(struct _modbusBlock* pBlock)
#endif
                       ) {
  modbusBlockDef** pLL;

  if ( bytSlaveAddress < 1 || bytSlaveAddress > 247 ) {
    return FALSE;
  }
// Validate parameters
  if ( pBlock == NULL
    || uintAddress == 0
    || uintTotal == 0
    || paryData == NULL ) {
    return FALSE;
  }
  if ( eType == COILS ) {
    pLL   = &pCoils;
  } else if ( eType == STATUS_INPUTS ) {
    pLL   = &pStatusBits;
  } else if ( eType == HOLDING_REGISTERS ) {
    pLL   = &pHoldingRegs;
  } else if ( eType == INPUT_REGISTERS ) {
    pLL   = &pInputRegs;
  } else {
    return FALSE;
  }
  if ( *pLL == NULL ) {
// This is the first block
    *pLL = pBlock;
  } else {
// Make sure there are no blocks with the same address and type
    modbusBlockDef* pNode;
    for( pNode=*pLL; pNode->pNext!=NULL; pNode=pNode->pNext ) {
      if ( uintAddress >= pNode->uintAddress
        && uintAddress <= (pNode->uintAddress + pNode->uintTotal - 1)  ) {
// A block already exists using this address and for this type
        return FALSE;
      }
    }
// Append this block to the linked list
    pNode->pNext = pBlock;
  }
// Populate the block
  pBlock->bytSlaveAddress = bytSlaveAddress;
  pBlock->eType           = eType;
  pBlock->uintAddress     = uintAddress;
  pBlock->uintTotal       = uintTotal;
  pBlock->paryData        = paryData;
  pBlock->pCallback       = pCallback;
  pBlock->pNext           = NULL;
}
/**
 * Function:
 *  calcCRC
 *
 * Parameters:
 *  uintLength, the number of bytes in the message
 *
 * Returns:
 *  A 16bit CRC for the passed message
 *
 * Uses: R0..3, FSR0
 * Remarks: 650 cycles for 8 bytes of data
 */
#pragma funcall calcCRC dummy

uint calcCRC(void) {
  asm {
   MBgetCRC:
         movff    _bytMbIndex, R2     // byte count in R2
         lfsr     0, _arybytMbBuffer  // point to arybytMbBuffer
         setf     R0                  // preset CRC
         setf     R1
   MBgetCRC1:
         movf     POSTINC0,W          // xor byte with CRClo
         xorwf    R0,F
         movlw    8                   // bit count in R3
         movwf    R3
   MBgetCRC2:
         bcf      STATUS,C
         rrcf     R1,F
         rrcf     R0,F
         bnc      $+10
         movlw    0xA0
         xorwf    R1,F
         movlw    0x01
         xorwf    R0,F
         decfsz   R3,F
         bra      MBgetCRC2
         decfsz   R2,F
         bra      MBgetCRC1
         return
  }
  return 0;
}
/**
 * Function:
 * modbusSerialInit
 *
 * Parameters:
 *  eBaud, see modbusSlave.h for options
 *  bytStopBits, number of transmitted stop bits, 1 or 2
 *  bytSlaveAddress, optional, modbus slave address, 1 to 247
 *
 * Returns:
 *  0 if ok, -1 if error
 */
int modbusSerialInit(baudRate eBaud, const byte bytStopBits, ...) {
  long lngTimeoutPreset;
  byte bytTemp;
  va_list ap;
// Ensure port C is configured for digital
  //ANSELC = 0;
// Initialise variable argument list
  va_start(ap, bytStopBits);
// Assign the slave address
  bytMbSlaveAddress = va_arg(ap, byte);
// Make sure the received array is cleared
  memset(arybytMbBuffer, 0, sizeof(arybytMbBuffer));
// Initialise the Rx Gap counter
  uintMbRxGap = 0;
// Ensure there is no exception code
  eMbExceptionCode = NO_EXCEPTION;
// Initialize UART module for the specified bps
  switch( eBaud ) {
  case BAUD_1200:
    UART1_Init(BAUD_1200 * 100);
    lngTimeoutPreset  = GAP_SETPT_1200;
    uintPacketTimeout = PACKET_TIMEOUT_1200;
    break;
  case BAUD_2400:
    UART1_Init(BAUD_2400 * 100);
    lngTimeoutPreset  = GAP_SETPT_2400;
    uintPacketTimeout = PACKET_TIMEOUT_2400;
    break;
  case BAUD_4800:
    UART1_Init(BAUD_4800 * 100);
    lngTimeoutPreset  = GAP_SETPT_4800;
    uintPacketTimeout = PACKET_TIMEOUT_4800;
    break;
  case BAUD_9600:
    UART1_Init(BAUD_9600 * 100);
    lngTimeoutPreset  = GAP_SETPT_9600;
    uintPacketTimeout = PACKET_TIMEOUT_9600;
    break;
  case BAUD_19200:
    UART1_Init(BAUD_19200 * 100);
    lngTimeoutPreset  = GAP_SETPT_19200;
    uintPacketTimeout = PACKET_TIMEOUT_19200;
    break;
 case BAUD_38400:
    UART1_Init(BAUD_38400 * 100);
    lngTimeoutPreset  = GAP_SETPT_38400;
    uintPacketTimeout = PACKET_TIMEOUT_38400;
    break;
  case BAUD_57600:
    UART1_Init(BAUD_57600 * 100);
    lngTimeoutPreset  = GAP_SETPT_57600;
    uintPacketTimeout = PACKET_TIMEOUT_57600;
    break;
  case BAUD_115200:
    UART1_Init(BAUD_115200 * 100);
    lngTimeoutPreset  = GAP_SETPT_115200;
    uintPacketTimeout = PACKET_TIMEOUT_115200;
    break;
  default:
    return -1;
  }
// Wait for UART module to stabilize
  Delay_ms(100);
// Timer0 Registers:
// 16-Bit Mode
// Prescaler=1:1
  T0CON.TMR0ON = 0;// Timer0 On/Off Control bit 1=Enables / 0=Stops
  T0CON.T08BIT = 0;// Timer0 8-bit/16-bit Control bit: 1=8-bit/ 0=16-bit
  T0CON.T0CS   = 0;// TMR0 Clock Source Select bit: 0=Internal
  T0CON.T0SE   = 0;// TMR0 Source Edge Select bit: 0=low/high
  T0CON.PSA    = 1;// Prescaler Assignment bit: 1=NOT assigned/bypassed
  T0CON.T0PS2  = 0;// bits 2-0  PS2:PS0: Prescaler Select bits
  T0CON.T0PS1  = 0;
  T0CON.T0PS0  = 0;
// Set prescaler if needed
  bytTemp = 0;
  while( lngTimeoutPreset > 29126 ) {
    lngTimeoutPreset >>= 1;
    bytTemp++;
  }
  if ( bytTemp > 0 && bytTemp <= 8 ) {
    T0CON.PSA = 0;   // Prescaler assigned
    T0CON = T0CON | (--bytTemp & 0x07);
  }
  uintMbRxGapSetPt1 = (uint)-(lngTimeoutPreset >> 1);   // 1 char
  uintMbRxGapSetPt2 = (uint)-lngTimeoutPreset;          // 2 chars
  uintMbRxGapSetPt3 = (uint)-(lngTimeoutPreset*5 >> 2); // 2.5 chars
  uintMbRxGapSetPt  = (uint)-(lngTimeoutPreset*9 >> 2); // 4.5 chars
// Double STOP bit while transmitting?
  if ( bytStopBits != 1 ) {
    TX9_bit  = 1;
    TX9D_bit = 1;
  }
// Clear errors, Rx buffer & Rx phase indicator
// and start silent interval in case bus is active
  restartRx();
  TMR0IE_bit = 1;
// Enable peripheral interrupts
  PEIE_bit   = 1;
// Enable GLOBAL interrupts
  GIE_bit    = 1;
}
/**
 * Function:
 *  restartRx
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 */
void restartRx(void) {
// Clear FIFO
  bytMbRxphase = RCREG;
  bytMbRxphase = RCREG;
// Clear errors and enable Rx
  CREN_bit = 0;
  CREN_bit = 1;
// Set RS-485 direction
  //Tx_dir = 0;
// Set parity check
  //RX9_bit = 1;
// ...or double STOP bit check
  //RX9D_bit = 1;
// Clear counters
  bytMbRxphase = 0;
  bytMbIndex = 0;
// Disable Tx
  TXEN_bit = 0;
  TXIE_bit = 0;
// Enable Rx interrupts
  RCIE_bit = 1;
  startTimeout();
}
/**
 * Function:
 *  serviceIOBlocksPrimitive
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 */
static void serviceIOBlocksPrimitive(modbusBlockDef* pBlockType) {
  modbusBlockDef* pNode;
  for( pNode=pBlockType; pNode!=NULL; pNode=pNode->pNext ) {
    if ( pNode->blnUpdate == TRUE ) {
      if ( pNode->pCallback != NULL ) {
        (*pNode->pCallback)(pNode);
      }
      pNode->blnUpdate = FALSE;
    }
  }
}
/**
 * Function:
 *  serviceIOBlocks
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 */
void serviceIOBlocks(void) {
  if ( pHoldingRegs != NULL ) {
    serviceIOBlocksPrimitive(pHoldingRegs);
  }
  if ( pStatusBits != NULL ) {
    serviceIOBlocksPrimitive(pStatusBits);
  }
  if ( pInputRegs != NULL ) {
    serviceIOBlocksPrimitive(pInputRegs);
  }
  if ( pCoils != NULL ) {
    serviceIOBlocksPrimitive(pCoils);
  }
}
/**
 * Function:
 *  startTimeout
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 */
void startTimeout(void) {
// Disable interrupt
  TMR0ON_bit = 0;
// Set timer presets
  if ( bytMbRxphase.B1 ) {
    TMR0H = Hi(uintMbRxGapSetPt3);
    TMR0L = Lo(uintMbRxGapSetPt3);
  } else if ( bytMbRxphase.B2 ) {
    TMR0H = Hi(uintMbRxGapSetPt2);
    TMR0L = Lo(uintMbRxGapSetPt2);
  } else if ( bytMbRxphase.B7 ) {
    TMR0H = Hi(uintMbRxGapSetPt1);
    TMR0L = Lo(uintMbRxGapSetPt1);
  } else {
    TMR0H = Hi(uintMbRxGapSetPt);
    TMR0L = Lo(uintMbRxGapSetPt);
  }
// Clear the interrupt mask
  TMR0IF_bit = 0;
// Enable interrupt
  TMR0ON_bit = 1;
}