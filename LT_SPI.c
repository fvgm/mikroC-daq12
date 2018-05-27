#include <stdint.h>
#include "LT_SPI.h"

// Reads and sends a byte
// Return 0 if successful, 1 if failed
void spi_transfer_byte(uint8_t ttx, uint8_t *rrx) {
  Chip_Select = 0;                         //! 1) Pull CS low

  *rrx = SPI1_Read(tx); //             //! 2) Read byte and send byte

  Chip_Select = 1;                //! 3) Pull CS high
}

// Reads and sends a word
// Return 0 if successful, 1 if failed
void spi_transfer_word(uint16_t ttx, uint16_t *rrx) {
  union  {
    uint8_t b[2];
    uint16_t w;
  } data_tx;

  union  {
    uint8_t b[2];
    uint16_t w;
  } data_rx;

  data_tx.w = tx;

  Chip_Select = 0;                              //! 1) Pull CS low

  data_rx.b[1] = SPI1_Read(data_tx.b[1]);  //! 2) Read MSB and send MSB
  data_rx.b[0] = SPI1_Read(data_tx.b[0]);  //! 3) Read LSB and send LSB

  *rrx = data_rx.w;

  Chip_Select = 1;                              //! 4) Pull CS high
}

// Reads and sends a byte array
void spi_transfer_block(uint8_t *ttx, uint8_t *rrx, uint8_t length) {
  int8_t i=0;
  Chip_Select = 0;                       //! 1) Pull CS low

  for (i=(length-1);  i >= 0; i--)
    rrx[i] = SPI1_Read(ttx[i]);       //! 2) Read and send byte array

  Chip_Select = 1;                       //! 3) Pull CS high
}