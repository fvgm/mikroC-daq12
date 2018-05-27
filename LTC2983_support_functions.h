/*!
LTC2983_support_functions.h:
This file contains all the support function prototypes used in the main program.

http://www.linear.com/product/LTC2983

http://www.linear.com/product/LTC2983#demoboards

$Revision: 1.3.4 $
$Date: October 5, 2016 $
Copyright (c) 2014, Linear Technology Corp.(LTC)
All rights reserved.

*/

void assign_channel(uint8_t channel_number, uint32_t channel_assignment_data);
void write_custom_steinhart_hart(uint32_t steinhart_hart_coeffs[6], uint16_t start_address);

float measure_channel(uint8_t channel_number, uint8_t channel_output);
void convert_channel(uint8_t channel_number);
void wait_for_process_to_finish();
void wait_for_interrupt();

float get_result(uint8_t channel_number, uint8_t channel_output);
float print_conversion_result(uint32_t raw_conversion_result, uint8_t channel_output);
//void read_voltage_or_resistance_results(uint8_t channel_number);
void print_fault_data(uint8_t fault_byte);

uint32_t transfer_four_bytes(uint8_t read_or_write, uint16_t start_address, uint32_t input_data);
uint8_t transfer_byte(uint8_t read_or_write, uint16_t start_address, uint8_t input_data);

uint16_t get_start_address(uint16_t base_address, uint8_t channel_number);
bool is_number_in_array(uint8_t number, uint8_t *array, uint8_t array_length);