#define VBATT        0  // AN0
#define PRESSURE     1  // AN1
#define INT_TEMP     2  // AN2

#define LTC_INT      RB0_bit   //entrada
#define INPUT_STAT   RB1_bit   //entrada
#define CHRG_STAT    RB2_bit   //entrada
#define LTC_RESET    RB3_bit   //saída
#define DEBUG_LED    RB5_bit   //saída
#define BATT_CHECK   RB6_bit   //saída

#define HIGH 1
#define LOW 0

sbit Chip_Select at RB4_bit;   //LTC_CS
sbit Chip_Select_Direction at TRISB4_bit;

#include <headers.h>   // // protótipos de funções
#include <stdint.h>
#include <stdbool.h>
#include <built_in.h>
#include "modbus.h"
#include "LTC2983_configuration_constants.h"
#include "LT_SPI.h"
#include "LT_SPI.c"
#include "LTC2983_support_functions.h"
#include "LTC2983_support_functions.c"

//#include "LTC2983_table_coeffs.h"


// variáveis MODBUS
static volatile modbusBlockDef statusBitsBlock,
                               inputRegsBlock;

static volatile uint aryuintInputRegs[28];
static volatile byte arybytStatusBits[8];

float temperatureValue = 0.0;
bool updateInternal = false;

void interrupt() {
 decodePacket(); // usa o Timer0 e RCIF (USART) - modbus
 
 if (TMR1IF_bit){ // Timer1 @ 100mS
    TMR1IF_bit = 0;
    TMR1H         = 0x3C;
    TMR1L         = 0xB0;
    
    updateInternal = true;
    }
}

void MCHPtoIEEE(float *f) {
   char *p = (char *)f;
   char sign = p[2].B7;
   p[2].B7 = p[3].B0;
   p[3] >>= 1;
   p[3].B7 = sign;
}

void main() {
     Delay_ms(500);
     setup();

     while(1) {
        if(LTC_INT == HIGH) { // idle

            DEBUG_LED = ~DEBUG_LED;
            updateInputRegisters();
           
           convert_channel(0x00); // multiple channels conforme a máscara abaixo
           transfer_byte(WRITE_TO_RAM, 0x0F4, 0x00);
           transfer_byte(WRITE_TO_RAM, 0x0F5, 0x00);
           transfer_byte(WRITE_TO_RAM, 0x0F6, 0b00111111);
           transfer_byte(WRITE_TO_RAM, 0x0F7, 0b11111100);

        }
        
        if(updateInternal == true) {
           aryuintInputRegs[24] = ADC_Read(INT_TEMP);
           aryuintInputRegs[25] = ADC_Read(PRESSURE);
           
           BATT_CHECK = 1;
           aryuintInputRegs[26] = ADC_Read(VBATT);
           BATT_CHECK = 0;
           
           // FC-02
           //arybytStatusBits[0] = INPUT_STAT;
           //arybytStatusBits[1] = CHRG_STAT;

           arybytStatusBits[0] = 1;
           arybytStatusBits[2] = 1;
           
           updateInternal = false; // clear flag

           //DEBUG_LED = ~DEBUG_LED;
        }


        serviceIOBlocks();   // Any updates?
        //DEBUG_LED = ~DEBUG_LED;
     }

}

void setup() {
     TRISA = 0x07;        // AN0:2 entradas; resto é saída.
     PORTA = 0;
     ADCON1 = 0b00001100; // AN0:2 analógicas
     CMCON = 0x07;        // comparadores OFF
     TRISB = 0X07;        // RB0, RB2 - entradas;  resto é saída
     PORTB = 0;
     TRISC = 0;
     PORTC = 0;
      
     Chip_Select = 1;                       // Deselect DAC
     Chip_Select_Direction = 0;             // Set CS# pin as Output
     SPI1_Init_Advanced(_SPI_MASTER_OSC_DIV4, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_LOW, _SPI_LOW_2_HIGH);
     Delay_ms(300);
     
/*UART1_Init(9600);
     Delay_ms(300);*/
   
     LTC_RESET = 1;
     wait_for_interrupt();
     configure_channels();
     configure_global_parameters();
     
     InitTimer1();
     
     modbusSerialInit(BAUD_9600, 1, 1); // inicializa módulo MODBUS e Timer0
     // Make sure the data areas are all cleared
     memset(arybytStatusBits,   0, sizeof(arybytStatusBits));
     memset(aryuintInputRegs,   0, sizeof(aryuintInputRegs));
     
// Create the various data I/O blocks
     addModbusBlock(1, STATUS_INPUTS,     &statusBitsBlock,  1, 8,
                   (void*)arybytStatusBits, NULL);  // fc-02
     addModbusBlock(1, INPUT_REGISTERS,   &inputRegsBlock,   1, 27,
                   (void*)aryuintInputRegs, NULL);       // FC-04
}

void configure_channels() {
  uint8_t channel_number;
  uint32_t channel_assignment_data;

  // ----- Channel 2: Assign Sense Resistor -----
  channel_assignment_data =
    SENSOR_TYPE__SENSE_RESISTOR |
    (uint32_t) 0xFA000 << SENSE_RESISTOR_VALUE_LSB;                // sense resistor - value: 1000.
  assign_channel(2, channel_assignment_data);
  // ----- Channel 3: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(3, channel_assignment_data);
  // ----- Channel 4: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(4, channel_assignment_data);
  // ----- Channel 5: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(5, channel_assignment_data);
  // ----- Channel 6: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(6, channel_assignment_data);
  // ----- Channel 7: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(7, channel_assignment_data);
  // ----- Channel 8: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(8, channel_assignment_data);
  // ----- Channel 9: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(9, channel_assignment_data);
  // ----- Channel 10: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(10, channel_assignment_data);
  // ----- Channel 11: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(11, channel_assignment_data);
  // ----- Channel 12: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(12, channel_assignment_data);
  // ----- Channel 13: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(13, channel_assignment_data);
  // ----- Channel 14: Assign RTD PT-100 -----
  channel_assignment_data =
    SENSOR_TYPE__RTD_PT_100 |
    RTD_RSENSE_CHANNEL__2 |
    RTD_NUM_WIRES__2_WIRE |
    RTD_EXCITATION_MODE__NO_ROTATION_SHARING |
    RTD_EXCITATION_CURRENT__100UA |
    RTD_STANDARD__AMERICAN;
  assign_channel(14, channel_assignment_data);
}

void configure_global_parameters() {
  transfer_byte(WRITE_TO_RAM, 0xF0, TEMP_UNIT__C | REJECTION__50_60_HZ);   // -- Set global parameters
  transfer_byte(WRITE_TO_RAM, 0xFF, 2); // -- Set any extra delay between conversions (in this case, 2*100us)
}

void InitTimer1(){
//Timer1
//Prescaler 1:2; TMR1 Preload = 15536; Actual Interrupt Time : 100 ms
  T1CON         = 0x11;
  TMR1IF_bit         = 0;
  TMR1H         = 0x3C;
  TMR1L         = 0xB0;
  TMR1IE_bit         = 1;
  INTCON         = 0xC0;
}

void updateInputRegisters() {
   unsigned short i = 0;
   
   for (i=1; i<=12; i++) {
      temperatureValue = get_result((i+2), TEMPERATURE);
      MCHPtoIEEE(&temperatureValue);
      aryuintInputRegs[((2*i)-2)] = HiWord(temperatureValue);
      aryuintInputRegs[(2*i)-1]   = LoWord(temperatureValue);
   }
}