#include <avr/io.h>
#include <util/delay.h>

//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){

  DDRB = 0x02; //output mode for SCLK, input for MISO

  SPCR = (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1<<SPE);
  //slave mode, clk low on idle, leading edge sample

  SPSR   = (1<<SPI2X); //choose double speed operation
 }//spi_init

/*********************************************************************/
//                            spi_read
//Reads the SPI port.
/*********************************************************************/
uint8_t spi_read(void){
  spi_init();
  SPDR = 0x00;                       //"dummy" write to SPDR
  while (bit_is_clear(SPSR,SPIF)){}  //wait till 8 clock cycles are done
DDRB = 0xFF;
PORTB = 0xFF;
_delay_ms(100);
PORTB = 0x00;
_delay_ms(100);
  return(SPDR);                      //return incoming data from SPDR
}//read_spi

/*********************************************************************/
//                            main
//
/*********************************************************************/
int main() {
  DDRE = 0xFF;
  PORTE = 0xFF;
  uint8_t x = 0x00;
  while(1){
    x = spi_read();
    //DDRB = 0xFF;
    //PORTB = 0xFF;
//    if(x == 0x01) { PORTB = 0xFF; }
//    else {PORTB = 0x0F; }
  }	

}
