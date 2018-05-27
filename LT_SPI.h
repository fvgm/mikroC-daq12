//! @todo Review this document.
/*!
LT_SPI: Routines to communicate with ATmega328P's hardware SPI port.

REVISION HISTORY
$Revision: 3659 $
$Date: 2015-07-01 10:19:20 -0700 (Wed, 01 Jul 2015) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.
*/

#ifndef LT_SPI_H
#define LT_SPI_H

#include <stdint.h>

void spi_transfer_byte(uint8_t ttx, uint8_t *rrx);

//! Reads and sends a word
//! @return void
void spi_transfer_word(uint16_t ttx,         //!< Byte to be transmitted
                       uint16_t *rrx         //!< Byte to be received
                      );

//! Reads and sends a byte array
//! @return void
void spi_transfer_block(uint8_t *ttx,        //!< Byte array to be transmitted
                        uint8_t *rrx,        //!< Byte array to be received
                        uint8_t length      //!< Length of array
                       );

#endif  // LT_SPI_H