/**
 * File:
 *  modbusSlave.c
 *
 * Notes:
 *  This file contains the implementation for a serial RTU modbus slave.
 *
 * Functions:
 *  coilState         Sets the state of a coil
 *  decodePacket      Decodes a packet received from modbus master
 *  packBits          Packs bits into a message buffer
 *  packRegisters     Packs registers into a message buffer
 *  setRegister       Sets the value of a holding register
 *
 * History:
 *  10/06/2012 Written by Simon Platten
 *  19/06/2012 Added receivePacket function which buffers a packet
 *  21/06/2012 Implemented exception responses
 *             Implemented Timer0 to time receipt of bytes
 *  25/06/2012 Implemented startTimeout
 *  27/06/2012 Replaced individual request and response buffers with a single
 *             message buffer.  Re-wrote CRC to use lookup tables.  Improved
 *             comments.
 *  30/06/2012 Implemented register mappings to allow 8 bit I/O ports to be
 *             mapped into 16 bit registers
 *  02/07/2012 Re-wrote I/O structures and functions, all I/O is now added
 *             using addModbusBlock.
 *  07/07/2012 Removed static variables:
 *              pCoilsTail,
 *              pStatusBitsTail,
 *              pHoldingRegsTail,
 *              pInputRegsTail,
 *              bytStartHi,
 *              bytStartLo, 
 *              bytTotalHi, 
 *              bytTotalLo, 
 *              blnValidStart,
 *              blnValidEnd
 *  08/07/2012 Added bytSlaveAddress to addModbusBlock.
 *             Removed bytAddress from modbusSlave as this is now part of block.
 *             Renamed modbusSlave to modbusSerialInit
 *             Renamed pWriteBlock to pCurrBlock
 *             Moved block location from coilStates and setRegister into
 *             decodePacket.
 *             Modified modbusSerialInit to take an optional argument, the
 *             slave address, this allows the decodePacket to return instantly
 *             if the packet is not addressed to this slave.
 *  09/07/2012 Created modbus.c and moved common functions from this module into
 *             modbus.c:
 *              addModbusBlock
 *              calcCRC
 *              modbusSerialInit
 *              startTimeout
 *              restartRx
 *  21/07/2012 Implementing packet timeouts
 */
#include <built_in.h>

#include "modbus.h"

// Pointer to the last block that was addressed
static modbusBlockDef* pCurrBlock;
/**
 * Function:
 *  coilState
 *
 * Parameters:
 *  uintAddress, the address of the coil to modify
 *  blnState, TRUE to turn coil on, FALSE to turn coil off
 *
 * Returns:
 *  TRUE if block modified, FALSE if not
 */
static boolean coilState(uint uintAddress, boolean blnState) {
  if ( pCurrBlock != NULL ) {
// Get byte and bit from address
    byte bytIndex, bytBit;
    uint uintOffset;
    uintOffset = uintAddress - pCurrBlock->uintAddress;
    bytIndex = uintOffset / 8;
    bytBit = 1 << (uintOffset % 8);

    if ( blnState == TRUE ) {
// Turn on
      ((byte*)pCurrBlock->paryData)[bytIndex] |= bytBit;
    } else {
// Turn off
      ((byte*)pCurrBlock->paryData)[bytIndex] &= (0xff ^ bytBit);
    }
    pCurrBlock->blnUpdate = TRUE;
    return TRUE;
  }
  return FALSE;
}
/**
 * Function:
 *  packBits
 *
 * Parameters:
 *  eType, should be either COILS or STATUS_INPUTS
 *  pBuffer, a pointer to the buffer to pack the bits into
 *  uintStart, the start address
 *  uintEnd, the end address
 *
 * Returns:
 *  The number of bytes the bits were packed into
 */
static uint packBits(mbType eType, byte* pBuffer,
                     uint uintStart, uint uintEnd) {
// Find the first I/O point
  modbusBlockDef* pNode;
  uint uintBytes = 0;

  if ( eType == COILS ) {
    pNode = pCoils;
  } else if( eType == STATUS_INPUTS ) {
    pNode = pStatusBits;
  } else {
    return 0;
  }
  while( pNode != NULL ) {
    if ( uintStart >= pNode->uintAddress ) {
      break;
    }
    pNode = pNode->pNext;
  }
  if ( pNode != NULL && uintEnd < pNode->uintAddress + pNode->uintTotal ) {
    uint uintOffset;
    byte bytIndex;
// Work out the address offset
    uintOffset = uintStart - pNode->uintAddress;
// The index into the data array
    bytIndex = uintOffset / 8;
// The bit in the data
    Hi(uintOffset) = 1 << (Lo(uintOffset) & 7);
// The bit of response bit field
    Lo(uintOffset) = 1;
// Clear first byte of response bit field
    *pBuffer = 0;
    while( uintStart <= uintEnd ) {
      if ( Lo(uintOffset).B0 ) {
        uintBytes++;
      }
// State
      if ( (((byte*)pNode->paryData)[bytIndex] & Hi(uintOffset)) ) {
        *pBuffer |= Lo(uintOffset);
      }
      Hi(uintOffset) <<= 1;
      
      if ( Hi(uintOffset) == 0 ) {
        Hi(uintOffset) = 1;
        bytIndex++;
      }
      Lo(uintOffset) <<= 1;
      
      if ( Lo(uintOffset) == 0 ) {
        Lo(uintOffset) = 1;
        pBuffer++;
        *pBuffer = 0;
      }
// Next address
      uintStart++;
    }
  }
  return uintBytes;
}
/**
 * Function:
 *  packRegisters
 *
 * Parameters:
 *  eType, should be either HOLDING_REGISTERS or INPUT_REGISTERS
 *  pBuffer, a pointer to the buffer to pack the registers into
 *  uintStart, the start address
 *  uintEnd, the end address
 *
 * Returns:
 *  The number of bytes the registers were packed into
 */
static uint packRegisters(mbType eType, byte* pBuffer,
                          uint uintStart, uint uintEnd) {
// Find the first I/O point
  modbusBlockDef* pNode;
  uint uintBytes = 0;

  if ( eType == HOLDING_REGISTERS ) {
    pNode = pHoldingRegs;
  } else if( eType == INPUT_REGISTERS ) {
    pNode = pInputRegs;
  } else {
    return 0;
  }
  while( pNode != NULL ) {
    if ( uintStart >= pNode->uintAddress ) {
      break;
    }
    pNode = pNode->pNext;
  }
  if ( pNode != NULL && uintEnd < (pNode->uintAddress + pNode->uintTotal) ) {
    uint uintOffset;
    uintOffset = uintStart - pNode->uintAddress;
    while( uintStart <= uintEnd ) {
      pBuffer[uintBytes++] = Hi(((uint*)pNode->paryData)[uintOffset]);
      pBuffer[uintBytes++] = Lo(((uint*)pNode->paryData)[uintOffset]);
// Next address
      uintStart++;
      uintOffset++;
     }
  }
  return uintBytes;
}
/**
 * Function:
 *  setRegister
 *
 * Parameters:
 *  uintAddress, the address of the register to modify
 *  bytDataHi, the hi-byte of the data
 *  bytDataLo, the lo-byte of the data
 *
 * Returns:
 *  TRUE if block modified, FALSE if not
 */
static boolean setRegister(uint uintAddress, byte bytDataHi, byte bytDataLo) {
  if ( pCurrBlock != NULL ) {
    uint uintTemp, uintOffset;
    Lo(uintTemp) = bytDataLo;
    Hi(uintTemp) = bytDataHi;
    uintOffset = uintAddress - pCurrBlock->uintAddress;
    ((uint*)pCurrBlock->paryData)[uintOffset] = uintTemp;
    pCurrBlock->blnUpdate = TRUE;
    return TRUE;
  }
  return FALSE;
}
/**
 * Function:
 *  decodePacket
 *
 * Parameters:
 *  none
 *
 * Returns:
 *  none
 */
#pragma funcall decodePacket dummy

void decodePacket(void) {
  if ( TMR0IF_bit ) {
    bytMbRxphase.B0 = 1;
    TMR0ON_bit = 0;
// Clear the interrupt mask
    TMR0IF_bit = 0;

    if ( bytMbRxphase.B7 ) {
// Last byte sent, restart reception & set silent interval
      restartRx();
      return;
    }
  }
  if( RCIF_bit ) {
    byte bytTemp;
// Overrun or framing error?
    if ( OERR_bit || FERR_bit ) {
      restartRx();
      return;
    }
// May need to store RX9D_bit here for parity check!
// Read received byte
    bytTemp = RCREG;
// Silent interval or inter-char interval passed?
// If passed, is there enough space in buffer?
    if ( bytMbRxphase == 0 || bytMbRxphase.B2 == 1 ||
         bytMbIndex >= MAX_PACKET_LENGTH ) {
// No, wait for end of current message
      bytMbIndex = 0;
      bytMbRxphase = 0;
      startTimeout();  // 4.5 chars
      return;
    }
    if ( bytMbIndex == 0) {
// Address received
      bytMbRxphase = 2;
    }
// Store the data in buffer
    arybytMbBuffer[bytMbIndex++] = bytTemp;
// Set inter-char interval
    startTimeout();  // 2.5 chars
  } else if ( bytMbRxphase.B7 ) {
    if ( TXIF_bit ) {
      if ( bytMbIndex < arybytMbBuffer[0] ) {
        TXREG = arybytMbBuffer[bytMbIndex++];
      } else {
// Wait for last byte to be transmitted
        startTimeout();  // 1 char
// Disable Tx interrupts
        TXIE_bit = 0;
      }
      return;
    }
  }
  if ( bytMbRxphase == 3 ) {
// Inter-char interval passed, check inter-frame timeout remainder
    bytMbRxphase = 4;
    startTimeout(); // 2 chars
  }
  if ( bytMbRxphase != 5 ) {
// Nothing more to do
    return;
  }
  if ( bytMbSlaveAddress == arybytMbBuffer[0] && bytMbIndex > 3 ) {
// Make sure write block is not set
    pCurrBlock = NULL;
// Calculate the packet CRC
    bytMbIndex -= 2;
    uintCRC = calcCRC();
// Compare the calculate checksum with the received checksum
    if ( Lo(uintCRC) == arybytMbBuffer[bytMbIndex] &&
         Hi(uintCRC) == arybytMbBuffer[bytMbIndex + 1] ) {
// Set the default placement of the CRC
      bytMbIndex = 3;
// Is the function supported?
      switch( arybytMbBuffer[1] ) {
      case READ_COILS:
      case FORCE_SINGLE_COIL:
      case FORCE_MULTIPLE_COILS:
        if ( pCoils == NULL ) {
// No coils defined!
          eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
        } else {
          pCurrBlock = pCoils;
        }
        break;
      case READ_STATUS_INPUTS:
        if ( pStatusBits == NULL ) {
// No status inputs defined!
          eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
        } else {
          pCurrBlock = pStatusBits;
        }
        break;
      case READ_HOLDING_REGISTERS:
      case PRESET_SINGLE_REGISTER:
      case PRESET_MULTIPLE_REGISTERS:
        if ( pHoldingRegs == NULL ) {
// No holding registers defined!
          eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
        } else {
          pCurrBlock = pHoldingRegs;
        }
        break;
      case READ_INPUT_REGISTERS:
        if ( pInputRegs == NULL ) {
// No input registers defined!
          eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
        } else {
          pCurrBlock = pInputRegs;
        }
        break;
      default:
// No, function not supported!
        eMbExceptionCode = ILLEGAL_FUNCTION;
        break;
      }
      if( eMbExceptionCode == NO_EXCEPTION ) {
        uint uintSAddr, uintEAddr, uintItemCount;
        uint uintOffset, uintAddr;
        boolean blnState;
        mbType eType;
        byte bytBit;
// Get the start address
        Lo(uintSAddr) = arybytMbBuffer[3];
        Hi(uintSAddr) = arybytMbBuffer[2];

        if ( arybytMbBuffer[1] == FORCE_SINGLE_COIL
          || arybytMbBuffer[1] == PRESET_SINGLE_REGISTER ) {
          uintItemCount = 1;
        } else {
// How many items have been requested?
          Lo(uintItemCount) = arybytMbBuffer[5];
          Hi(uintItemCount) = arybytMbBuffer[4];
        }
// Whats the last address?
        uintEAddr = uintSAddr + uintItemCount;
        uintSAddr++;
 // Find the I/O block that contains the address
        while( pCurrBlock != NULL ) {
          if ( uintSAddr >= pCurrBlock->uintAddress
            && uintEAddr <= (pCurrBlock->uintAddress +
                               pCurrBlock->uintTotal - 1)  ) {
            break;
          }
          pCurrBlock = pCurrBlock->pNext;
        }
// What was the requested function code?
        switch( arybytMbBuffer[1] ) {
        case READ_COILS:
        case READ_STATUS_INPUTS:
          if ( arybytMbBuffer[1] == READ_COILS ) {
            eType = COILS;
          } else {
            eType = STATUS_INPUTS;
          }
          arybytMbBuffer[2] = packBits(eType, &arybytMbBuffer[3], 
                                       uintSAddr, uintEAddr);

          if ( arybytMbBuffer[2] == 0 ) {
// Exception, address does not exist
            eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
          } else {
            bytMbIndex += arybytMbBuffer[2];
          }
          break;
        case READ_HOLDING_REGISTERS:
        case READ_INPUT_REGISTERS:
          if ( arybytMbBuffer[1] == READ_INPUT_REGISTERS ) {
            eType = INPUT_REGISTERS;
          } else {
            eType = HOLDING_REGISTERS;
          }
          arybytMbBuffer[2] = packRegisters(eType, &arybytMbBuffer[3],
                                            uintSAddr, uintEAddr);
          if ( arybytMbBuffer[2] == 0 ) {
// Exception, address does not exist
            eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
          } else {
            bytMbIndex += arybytMbBuffer[2];
          }
          break;
        case FORCE_SINGLE_COIL:
          if ( arybytMbBuffer[4] == 0xff && arybytMbBuffer[5] == 0x0 ) {
// Force coil on
            blnState = coilState(uintSAddr, TRUE);
          } else if ( arybytMbBuffer[4] == 0x0 && arybytMbBuffer[5] == 0x0 ) {
// Force coil off
            blnState = coilState(uintSAddr, FALSE);
          }
          if ( blnState == FALSE ) {
// Exception, address does not exist
            eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
          } else {
            bytMbIndex += 4;
          }
          break;
        case PRESET_SINGLE_REGISTER:
          if ( setRegister(uintSAddr, arybytMbBuffer[4], 
                                      arybytMbBuffer[5]) == FALSE ) {
// Exception, address does not exist
            eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
          } else {
            bytMbIndex += 4;
          }
          break;
        case FORCE_MULTIPLE_COILS:
          if ( uintItemCount > MAX_DISCRETES_IN_FC15 ||
               uintItemCount > (arybytMbBuffer[6] * 8) ) {
// Exception, either to many items or the coil count doesn't match byte count
            eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
            break;
          }
        case PRESET_MULTIPLE_REGISTERS:
          if ( (arybytMbBuffer[1] == PRESET_MULTIPLE_REGISTERS &&
               uintItemCount > MAX_REGISTERS_IN_FC16) ) {
// Exception, either to many items or the coil count doesn't match byte count
            eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
          } else {
            bytBit = 1;
            uintOffset = 0;
            uintAddr = uintSAddr;

            while( uintAddr <= uintEAddr ) {
              if ( arybytMbBuffer[1] == PRESET_MULTIPLE_REGISTERS ) {
                if ( setRegister(uintAddr,
                                 arybytMbBuffer[7 + uintOffset],
                                 arybytMbBuffer[8 + uintOffset]) == FALSE ) {
                  eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
                  break;
                }
                uintOffset += 2;
                uintAddr++;
              } else {
                blnState = FALSE;
                if ( (arybytMbBuffer[7 + uintOffset] & bytBit) > 0 ) {
                  blnState = TRUE;
                }
                if ( coilState(uintAddr, blnState) == FALSE ) {
                  eMbExceptionCode = ILLEGAL_DATA_ADDRESS;
                  break;
                }
                bytBit <<= 1;
                if ( bytBit == 0 ) {
                  bytBit = 1;
                  uintOffset++;
                }
                uintAddr++;
              }
            }
            if ( eMbExceptionCode == NO_EXCEPTION ) {
              bytMbIndex += 3;
            }
          }
          break;
        }
      }
      if( eMbExceptionCode != NO_EXCEPTION ) {
        arybytMbBuffer[1] |= EXCEPTION_FLAG;
        arybytMbBuffer[2] = (byte)eMbExceptionCode;
        eMbExceptionCode = NO_EXCEPTION;
      }
      if ( bytMbIndex > 0 ) {
// Calculate CRC for response
        uintCRC = calcCRC();
        arybytMbBuffer[bytMbIndex++] = Lo(uintCRC);
        arybytMbBuffer[bytMbIndex++] = Hi(uintCRC);
// Wait for end of last trasmission or silent interval
        while( bytMbRxphase.B0 == 0 ) ;
// Stop receiving
        CREN_bit = 0;
        RCIE_bit = 0;
// Enable transmission, set direction
        TXEN_bit = 1;
        //Tx_dir = 1;
// Mark start of transmission
        bytMbRxphase = 128;
// Send address
        TXREG = arybytMbBuffer[0];
// Store length and set index
        arybytMbBuffer[0] = bytMbIndex;
// Send function code to fill Tx buffer and avoid instant return to ISR
        while( !TXIF_bit ){}
        TXREG = arybytMbBuffer[1];
// Enable Tx interrupts for remainder of response to follow
        bytMbIndex = 2;
        TXIE_bit = 1;
        return;
      }
    }
  }
// No need to answer (not our address or message frame error, await next one)
  bytMbIndex = 0;
  bytMbRxphase = 1;
}