#include <avr/io.h>
#include <util/delay.h>

uint8_t segment_data[5];  // each element corresponds to digit (or colon)

//decimal to 7-segment LED display encodings, logic "0" turns on segment.
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x98, 0xFF};


//***********************************************************************
//                            spi_init
//**********************************************************************
void spi_init(void){
  DDRB =  0x17;                 //Turn on SS, MOSI, SCLK (SS is output)
  SPCR = (1<<SPE) | (1<<MSTR);  //SPI enabled, master, low polarity, MSB 1st
  SPSR = (1<<SPI2X);            //run at i/o clock/2
}//spi_init



/*********************************************************************/
//                            spi_read
//Reads the SPI port.
/*********************************************************************/
uint8_t spi_read(void){
  // DDRB &= (0<<3);
  SPDR = 0x00;                       //"dummy" write to SPDR
  while (bit_is_clear(SPSR,SPIF)){}  //wait till 8 clock cycles are done
  return(SPDR);                      //return incoming data from SPDR
}//read_spi

//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
/***************************************************************************************/
void segsum() {
  uint8_t d0, d1, d2, d3;
  uint8_t temp;


  DDRE |= (1<<6);  //set PE6 as output
  PORTB |= (1<<4);     // set clk_inh HIGH
  PORTE &= (0<<6);        // set PE6 LOW (SH/LD)
  PORTE |= (1<<6);
  PORTB &= (0<<4);     // // set clk_inh LOW
  temp = spi_read();

  //break up decimal sum into 4 digit-segments
  d3 = temp/1000;
  temp = temp - d3*1000;
  d2 = temp/100;
  temp = temp - d2*100;
  d1 = temp/10;
  temp = temp - d1*10;
  d0 = temp/1;

  //blank out leading zero digits
  // segment_data[10] = BCD for blank
  if( d3 == 0 )
  {
    d3 = 10;
      if( d2 == 0 )
      {
        d2 = 10;
        if( d1 == 0 )
        {
          d1=10;
        }
      }
  }

  //now move data to right place for misplaced colon position
  segment_data[0] = d0;
  segment_data[1] = d1;
  segment_data[3] = d2;
  segment_data[4] = d3;

}//segment_sum

/***************************************************************************************/
//MAIN
/***************************************************************************************/
int main(){
  uint8_t x;

  spi_init();

  while(1) {
  segsum();
  // x = spi_read();
  DDRF = 0xFF;    // set PORTD as output
  DDRA = 0xFF;   // set PORTA as output

  PORTA = dec_to_7seg[segment_data[0]]; // enable segments
  PORTF = 0x00; // Digit 3 ON
  _delay_ms(0.1);

  PORTA = dec_to_7seg[segment_data[1]]; // enable segments
  PORTF = 0x10;
  _delay_ms(0.1);

  PORTA = dec_to_7seg[segment_data[3]]; // enable segments
  PORTF = 0x30;
  _delay_ms(0.1);

  PORTA = dec_to_7seg[segment_data[4]]; // enable segments
  PORTF = 0x40;
  _delay_ms(0.1);

  DDRA = 0x00;
  //PORTA = 0xFF;
  }
}
