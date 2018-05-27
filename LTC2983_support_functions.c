/*!
LTC2983_support_functions.cpp:
This file contains all the support functions used in the main program.
http://www.linear.com/product/LTC2983
http://www.linear.com/product/LTC2983#demoboards
$Revision: 1.3.4 $
$Date: October 5, 2016 $
Copyright (c) 2014, Linear Technology Corp.(LTC)
All rights reserved.
*/

#include <stdint.h>
//#include "LT_SPI.h"
//#include "LT_SPI.c"
#include "LTC2983_configuration_constants.h"
#include "LTC2983_table_coeffs.h"
#include "LTC2983_support_functions.h"

#define hi(param) ((char *)&param)[1]
#define lo(param) ((char *)&param)[0]
#define Highest(param) ((char *)&param)[3]



// ***********************
// Program the part
// ***********************
void assign_channel(uint8_t channel_number, uint32_t channel_assignment_data) {
  uint16_t start_address = get_start_address(CH_ADDRESS_BASE, channel_number);
  transfer_four_bytes(WRITE_TO_RAM, start_address, channel_assignment_data);
}

void write_custom_table(struct table_coeffs coefficients[64], uint16_t start_address, uint8_t table_length) {
  int8_t i;
  uint32_t coeff;

  Chip_Select = 0; //output_low(chip_select);

  SPI1_Write(WRITE_TO_RAM);
  SPI1_Write(hi(start_address));
  SPI1_Write(lo(start_address));

  for (i=0; i< table_length; i++)  {
    coeff = coefficients[i].measurement;
    SPI1_Write((uint8_t)(coeff >> 16));
    SPI1_Write((uint8_t)(coeff >> 8));
    SPI1_Write((uint8_t)coeff);

    coeff = coefficients[i].temperature;
    SPI1_Write((uint8_t)(coeff >> 16));
    SPI1_Write((uint8_t)(coeff >> 8));
    SPI1_Write((uint8_t)coeff);
  }
  Chip_Select = 1; //output_high(chip_select);
}


void write_custom_steinhart_hart(uint32_t steinhart_hart_coeffs[6], uint16_t start_address) {
  int8_t i;
  uint32_t coeff;

  Chip_Select = 0; //output_low(chip_select);

  SPI1_Write(WRITE_TO_RAM);
  SPI1_Write(hi(start_address));
  SPI1_Write(lo(start_address));

  for (i = 0; i < 6; i++)  {
    coeff = steinhart_hart_coeffs[i];
    SPI1_Write((uint8_t)(coeff >> 24));
    SPI1_Write((uint8_t)(coeff >> 16));
    SPI1_Write((uint8_t)(coeff >> 8));
    SPI1_Write((uint8_t)coeff);
  }
  Chip_Select = 1; //output_high(chip_select);
}

// *****************
// Measure channel
// *****************
float measure_channel(uint8_t channel_number, uint8_t channel_output) {
    convert_channel(channel_number);
    return get_result(channel_number, channel_output);
}

void convert_channel(uint8_t channel_number) { // Start conversion
  transfer_byte(WRITE_TO_RAM, COMMAND_STATUS_REGISTER, CONVERSION_CONTROL_BYTE | channel_number);
  if(channel_number != 0) { // multiple channels
     wait_for_process_to_finish();
  }
}

bool check() {
  uint8_t process_finished = 0;
  uint8_t data_;
    data_ = transfer_byte(READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
    return process_finished  = data_ & 0x40;
}

void wait_for_process_to_finish() {
  uint8_t process_finished = 0;
  uint8_t data_;
  while (process_finished == 0)  {
    data_ = transfer_byte(READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0);
    process_finished  = data_ & 0x40;
  }
}

void wait_for_interrupt() {
  while (LTC_INT == 0)  {
  }
}

// *********************************
// Get results
// *********************************
float get_result(uint8_t channel_number, uint8_t channel_output) {
  uint32_t raw_data;
  uint8_t fault_data;
  
  uint16_t start_address = get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
  uint32_t raw_conversion_result;

  raw_data = transfer_four_bytes(READ_FROM_RAM, start_address, 0);

  //UART_Write_Text("\nChannel ");
  //UART_Write(channel_number+48);

  // 24 LSB's are conversion result
  raw_conversion_result = raw_data & 0xFFFFFF;
  return print_conversion_result(raw_conversion_result, channel_output);

  // If you're interested in the raw voltage or resistance, use the following
/*if (channel_output != VOLTAGE) {
    read_voltage_or_resistance_results(channel_number);
  }*/

  // 8 MSB's show the fault data
/*fault_data = raw_data >> 24;
  print_fault_data(fault_data);
  UART1_Write(fault_data);*/
}


float print_conversion_result(uint32_t raw_conversion_result, uint8_t channel_output) {
  int32_t signed_data = raw_conversion_result;
  float scaled_result;
  unsigned int txt[4];

  if(signed_data & 0x800000)
    signed_data = signed_data | 0xFF000000; // Convert the 24 LSB's into a signed 32-bit integer

  if (channel_output == TEMPERATURE)  {   // Translate and print result
    scaled_result = signed_data/1024.0;
  }
  else if (channel_output == VOLTAGE)  {
    scaled_result = signed_data/2097152.0;
  }
  return scaled_result;
}


/*void read_voltage_or_resistance_results(uint8_t channel_number) {
  int32_t raw_data;
  float voltage_or_resistance_result;
  uint16_t start_address = get_start_address(VOUT_CH_BASE, channel_number);
  char txt[15];

  raw_data = transfer_four_bytes(READ_FROM_RAM, start_address, 0);
  voltage_or_resistance_result = raw_data/1024.0;
  UART_Write_Text("  Voltage or resistance = ");
  FloatToStr_FixLen(voltage_or_resistance_result,txt,6);
  UART_Write_Text(txt);
  //UART_Write_Text(voltage_or_resistance_result);
} */

// Translate the fault byte into usable fault data and print it out
void print_fault_data(uint8_t fault_byte) {
  //
  UART_Write_Text("  FAULT DATA = ");
  UART_Write(fault_byte);

  if (fault_byte & SENSOR_HARD_FAILURE)
    UART_Write_Text("  - SENSOR HARD FALURE");
  if (fault_byte & ADC_HARD_FAILURE)
    UART_Write_Text("  - ADC_HARD_FAILURE");
  if (fault_byte & CJ_HARD_FAILURE)
    UART_Write_Text("  - CJ_HARD_FAILURE");
  if (fault_byte & CJ_SOFT_FAILURE)
    UART_Write_Text("  - CJ_SOFT_FAILURE");
  if (fault_byte & SENSOR_ABOVE)
    UART_Write_Text("  - SENSOR_ABOVE");
  if (fault_byte & SENSOR_BELOW)
    UART_Write_Text("  - SENSOR_BELOW");
  if (fault_byte & ADC_RANGE_ERROR)
    UART_Write_Text("  - ADC_RANGE_ERROR");
  if (!(fault_byte & VALID))
    UART_Write_Text("INVALID READING !!!!!!");
  if (fault_byte == 0b11111111)
    UART_Write_Text("CONFIGURATION ERROR !!!!!!");
}

// *********************
// SPI RAM data transfer
// *********************
// To write to the RAM, set ram_read_or_write = WRITE_TO_RAM.
// To read from the RAM, set ram_read_or_write = READ_FROM_RAM.
// input_data is the data to send into the RAM. If you are reading from the part, set input_data = 0.

uint32_t transfer_four_bytes(uint8_t ram_read_or_write, uint16_t start_address, uint32_t input_data) {
  uint32_t output_data;
  uint8_t ttx[7], rrx[7];

  ttx[6] = ram_read_or_write;
  ttx[5] = hi(start_address);
  ttx[4] = lo(start_address);
  ttx[3] = (uint8_t)(input_data >> 24);
  ttx[2] = (uint8_t)(input_data >> 16);
  ttx[1] = (uint8_t)(input_data >> 8);
  ttx[0] = (uint8_t) input_data;

  spi_transfer_block(ttx, rrx, 7);

  output_data = (uint32_t) rrx[3] << 24 |
                (uint32_t) rrx[2] << 16 |
                (uint32_t) rrx[1] << 8  |
                (uint32_t) rrx[0];

  return output_data;
}


uint8_t transfer_byte(uint8_t ram_read_or_write, uint16_t start_address, uint8_t input_data) {
  uint8_t ttx[4], rrx[4];

  ttx[3] = ram_read_or_write;
  ttx[2] = (uint8_t)(start_address >> 8);
  ttx[1] = (uint8_t)start_address;
  ttx[0] = input_data;
  spi_transfer_block(ttx, rrx, 4);
  return rrx[0];
}


// ******************************
// Misc support functions
// ******************************
uint16_t get_start_address(uint16_t base_address, uint8_t channel_number) {
  return base_address + 4 * (channel_number-1);
}


bool is_number_in_array(uint8_t number, uint8_t *array, uint8_t array_length) // Find out if a number is an element in an array
{
  uint8_t i=0;
  bool found = false;
  for (i=0; i< array_length; i++)
  {
    if (number == array[i])
    {
      found = true;
    }
  }
  return found;
}