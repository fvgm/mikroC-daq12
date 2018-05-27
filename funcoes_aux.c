                       float readVBatt() {
   unsigned adc_reading = 0;
   float voltage=0;
   short i=0;
   BATT_CHECK = 1;
   Delay_ms(100);
   for(i=0; i<10; i++) {
      adc_reading += ADC_Read(VBATT);
   }
   adc_reading = adc_reading/10;

   voltage = 1.36*((adc_reading/1023.0)*3.3);
   BATT_CHECK = 0;

   return voltage;
}

float readInternalTemp() {
   unsigned adc_reading = 0;
   float temperature;
   short i=0;
   for(i=0; i<10; i++) {
      adc_reading += ADC_Read(AN2);
   }
   adc_reading = adc_reading/10;
   temperature = 100*(((adc_reading)/1023.0)*3.3);
   return temperature;

}

float readPressure() {
   unsigned int adc_reading = 0;
   float pressure = 0;
   short i=0;
   for(i=0; i<10; i++) {
       adc_reading += ADC_Read(AN1);
   }
   adc_reading /= 10;
   pressure = (adc_reading/1023.0)*3.3;
   //Delay_ms(10);

   return pressure*1000;

}


void interrupt() {
}

 unsigned char Checksum(unsigned short header, unsigned length, unsigned char *msg) { // 8-bit Checksum http://www.planetimming.com/checksum8.html  http://easyonlineconverter.com/converters/checksum_converter.html

     unsigned char count;
     unsigned int Sum = 0;

     Sum = Sum + header;
     Sum = Sum + length;
     for (count = 0; count < length; count++) {
         Sum = Sum + msg[count];
     }

     Sum = -Sum;
     return (Sum & 0xFF);
 }

void MchpToIEEE754(float data_in, char data_out[4]) {
   unsigned char f0;
   MemCpy(data_out, &data_in, 4);
   f0 = data_out[3] & 0x01;
    data_out[3] = (data_out[3] >> 1);
    if ((data_out[2] & 0x80) == 1)
     data_out[3] = (data_out[3] | 0x80);
    else
     data_out[3] = (data_out[3] & 0x7F);

    if (f0 == 1)
     data_out[2] = (data_out[2] | 0x80);
    else
     data_out[2] = (data_out[2] & 0x7F);
}

void spSend(unsigned short header, unsigned length, char *msg){
  char i=0;

  UART1_Write(header);
  UART1_Write(length);

  for (i=0; i<length; i++){
    UART1_Write(*(msg+i));
  }
  UART1_Write(Checksum(header,length,msg));
}

struct meuPacote {
       char d1;
       char d2;
       char d3;
       char d4;
} pacote;


union {
   float float_variable;
   char temp_array[4];
} u;

float floatvar;
unsigned short spi_rx;

  uint16_t dataTemp;