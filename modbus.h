/**
 * File:
 *  modbus.h
 *
 * Notes:
 *  This file contains the prototype for a modbus RTU implementation.
 *
 * Usage to define a slave:
 *  modbusSerialInit(BAUD_19200, 1, 1);
 *  while(1) {
 *  }
 *
 * History:
 *  10/06/2012 Written by Simon Platten
 *  27/06/2012 Corrected MAX_DISCRETES_IN_FC15 and MAX_DISCRETES_IN_FC16
 *             Removed constants for frequency and timer presets.
 *  02/07/2012 Re-wrote functions and structures to add I/O types, implementing:
 *               addModbusBlock
 *  08/07/2012 Added bytSlaveAddress to addModbusBlock.
 *             Removed bytAddress from modbusSlave as this is now part of block.
 *             Renamed modbusSlave to modbusSerialInit
 *  20/07/2012 Modified addModbusBlock to take a variable argument list, the
 *             callback routine is now optional
 *  15/02/2013 Added MODBUS_MASTER and MODBUS_SLAVE definitions
 *             Modified modbusBlockDef adding blnUpdate flag
 *             Added routine serviceIOBlocks
 */
#ifndef MODBUS_H
  #define MODBUS_H

  #include "types.h"
// Implementation, comment out the one you don't need
//  #define MODBUS_MASTER               1
  #define MODBUS_SLAVE                1
// Constants
  #define EXCEPTION_FLAG              0x80
  #define MAX_PACKET_LENGTH           256
  #define MAX_DISCRETES_IN_FC15       1968
  #define MAX_REGISTERS_IN_FC16       123
  #define MAX_DISCRETES_IN_1_AND_2    2000
  #define MAX_REGISTERS_IN_3_AND_4    125
  #define MODBUS_2CHAR                2   // in chars
  #define MAX_RETRIES                 3
// Interpacket delays
  #define GAP_SETPT_1200    (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_1200))
  #define GAP_SETPT_2400    (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_2400))
  #define GAP_SETPT_4800    (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_4800))
  #define GAP_SETPT_9600    (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_9600))
  #define GAP_SETPT_19200   (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_19200))
  #define GAP_SETPT_38400   (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_38400))
  #define GAP_SETPT_57600   (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_57600))
  #define GAP_SETPT_115200  (long)(MODBUS_2CHAR*110*Clock_kHz()/(4*BAUD_115200))
// Response packet timeouts for modbus master
  #define PACKET_TIMEOUT_1200        2750
  #define PACKET_TIMEOUT_2400        1375
  #define PACKET_TIMEOUT_4800        688
  #define PACKET_TIMEOUT_9600        344
  #define PACKET_TIMEOUT_19200       172
  #define PACKET_TIMEOUT_38400       86
  #define PACKET_TIMEOUT_57600       57
  #define PACKET_TIMEOUT_115200      29
// Supported Baud rates / 100
  typedef enum eBaudRates {
    BAUD_1200   = 12,
    BAUD_2400   = 24,
    BAUD_4800   = 48,
    BAUD_9600   = 96,
    BAUD_19200  = 192,
    BAUD_38400  = 384,
    BAUD_57600  = 576,
    BAUD_115200 = 1152
  } baudRate;
// Modbus I/O types
  typedef enum eModbusTypes {
    INVALID_MB_TYPE   = 0,
    COILS             = 1,
    STATUS_INPUTS     = 2,
    HOLDING_REGISTERS = 3,
    INPUT_REGISTERS   = 4
  } mbType;
// Modbus function codes
  typedef enum eModbusFunctions {
    READ_COILS                = 1,
    READ_STATUS_INPUTS        = 2,
    READ_HOLDING_REGISTERS    = 3,
    READ_INPUT_REGISTERS      = 4,
    FORCE_SINGLE_COIL         = 5,
    PRESET_SINGLE_REGISTER    = 6,
    FORCE_MULTIPLE_COILS      = 15,
    PRESET_MULTIPLE_REGISTERS = 16
  } mbFunction;
// Modbus exception codes
  typedef enum eModbusExceptions {
    NO_EXCEPTION              = 0,
    ILLEGAL_FUNCTION          = 1,
    ILLEGAL_DATA_ADDRESS      = 2,
    ILLEGAL_DATA_VALUE        = 3,
    SLAVE_DEVICE_FAILURE      = 4,
    ACKNOWLEDGE               = 5,
    SLAVE_DEVICE_BUSY         = 6,
    NEGATIVE_ACKNOWLEDGE      = 7,
    MEMORY_PARITY_ERROR       = 8
  } mbException;
// Block definition
  typedef struct _modbusBlock {
// The modbus slave address 1 to 247 this block is associated with
    byte   bytSlaveAddress;
// The type of modbus data in this block
    mbType eType;
// The first address in the block
    uint   uintAddress;
// The total number of items in the block
    uint   uintTotal;
// Pointer to the actual data storage area
    void*  paryData;
// Flag to indicate update of block, used to flag call-back
    boolean blnUpdate;
// Pointer to funciton to call when block updated
#ifdef MODBUS_MASTER
    void (*pCallback)();
#elif defined MODBUS_SLAVE
    void (*pCallback)(struct _modbusBlock* pBlock);
#endif
// Link to next block
    struct _modbusBlock* pNext;
  } modbusBlockDef;
// Prototypes
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
                         );
  uint    calcCRC(void);
  void    decodePacket(void);
  int     modbusSerialInit(baudRate eBaud, const byte bytStopBits, ...);
  void    restartRx(void);
  void    serviceIOBlocks(void);
  void    startTimeout(void);
// Globals
#ifdef MODBUS_MASTER
// Pointer to the last block that was addressed
  extern modbusBlockDef* pCurrBlock;
#endif
// Pointer to the I/O linked lists
  extern modbusBlockDef* pHoldingRegs;
  extern modbusBlockDef* pStatusBits;
  extern modbusBlockDef* pInputRegs;
  extern modbusBlockDef* pCoils;
// Exception code
  extern mbException eMbExceptionCode;
// Receiver & transmitter GAP set-points
  extern uint uintMbRxGapSetPt1;
  extern uint uintMbRxGapSetPt2;
  extern uint uintMbRxGapSetPt3;
  extern uint uintMbRxGapSetPt;
// Packet timeout in milliseconds
  extern uint uintPacketTimeout;
// Calculated CRC
  extern uint uintCRC;
// Receiver GAP counter
  extern uint uintMbRxGap;
// The modbus slave address
  extern byte bytMbSlaveAddress;
// Received packet buffer
  extern byte arybytMbBuffer[MAX_PACKET_LENGTH];
// The byte index
  extern byte bytMbRxphase;
  extern byte bytMbIndex;
#endif