#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

uint8_t segment_data[5];  // each element corresponds to digit (or colon)

//data to 7-segment LED display encodings (0-F & blank). Logic "0" turns on segment.
uint8_t data_to_7seg[17] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x98, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E, 0xFF};
int16_t disp_value;       //value being incremented and decremeted
uint8_t last_encoder_val; // stores previos encoder value
uint16_t increment;       // stores increment setting
uint8_t buttons;          // holds buttons' toggle status
uint8_t base_mode;        // acts as bool. stores current base mode (hex or dec)

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

  // enable encoders to be read per timing diagram for 74hc165
  DDRE |= (1<<6);   //set PE6 as output
  PORTB |= (1<<4);  // set clk_inh HIGH (inhibits data from being shifted out)
  PORTE &= (0<<6);  // set PE6 LOW (SH/LD) (Load data to shift register)
  PORTE |= (1<<6);  // set SH/LD HIGH
  PORTB &= (0<<4);  // set clk_inh LOW  (enable clock for shift register)

  SPDR = 0x00;                       //"dummy" write to SPDR
  while (bit_is_clear(SPSR,SPIF)){}  //wait till 8 clock cycles are done

  return(SPDR);                      //return incoming data from SPDR
}//read_spi

//******************************************************************************
//                            chk_buttons
//Checks the state of the button number passed to it. It shifts in ones till
//the button is pushed. Function returns a 1 only once per debounced button
//push so a debounce and toggle function can be implemented at the same time.
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"
//Expects active low pushbuttons on PINA port.  Debounce time is determined by
//external loop delay times 12.
//
uint8_t chk_buttons(uint8_t button) {
    static uint16_t state[8] = {0,0,0,0,0,0,0,0};  //holds present state for each button

    //make PORTA an input port with pullups
    DDRA = 0x00;  // Set portA as input
    PORTA = 0xFF; // Pull HIGH

    //enable tristate buffer for pushbutton switches
    PORTF = 0x70; // enable COM_EN

    state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) { return 1; }
    return 0;
}//chk_buttons()

//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
/***************************************************************************************/
void segsum() {
  uint8_t d0, d1, d2, d3;
  uint16_t temp;

  if(base_mode){
    temp = disp_value;
    // temp = spi_read();

    //break up decimal sum into 4 digit-segments
    d3 = temp/1000;
    temp = temp - d3*1000;
    d2 = temp/100;
    temp = temp - d2*100;
    d1 = temp/10;
    temp = temp - d1*10;
    d0 = temp/1;

  }
  else{

    d0 = (disp_value & 0x000F);
    d1 = ( (disp_value>>4) & 0x000F);
    d2 = ( (disp_value>>8) & 0x000F);
    d3 = ( (disp_value>>12) & 0x000F);

  }

  //blank out leading zero digits
  // segment_data[10] = BCD for blank
  if( d3 == 0 )
  {
    d3 = 16;
      if( d2 == 0 )
      {
        d2 = 16;
        if( d1 == 0 )
        {
          d1=16;
        }
      }
  }

  //now move data to right place for misplaced colon position
  segment_data[0] = d0;
  segment_data[1] = d1;
  segment_data[3] = d2;
  segment_data[4] = d3;


}//segment_sum()


//***********************************************************************************
// Check if mode encoder changed in such a way that it should be toggled.
// Next, update current toggle data
/***************************************************************************************/
void base_mode_check(uint8_t encoder){

  // if base mode encoder is in resting position, and last time it was checked it was not,
  // then toggle base mode
  if( !( !(encoder&(1<<1)) | !(encoder&(1<<0)) ) ){
      if( !(last_encoder_val & (1<<1)) || !(last_encoder_val & (1<<0)) ){
        base_mode = !base_mode;   // toggle base mode
      }
    }

    //Enable buttons to be read
    DDRA = 0x00;  // Set portA as input
    PORTA = 0xFF; // Pull HIGH
    PORTF = 0x70; // enable COM_EN

    // check each button and toggle if it is pressed
    for (int i = 0; i < 2; i++)
    {
      if(chk_buttons(i)){
        buttons ^= 1 << i;  //toggle button value
      }
    }

    // toggle LED

}//base_mode_check()

//***********************************************************************************
// send data to be displayed to bargraph
/***************************************************************************************/
void bar_graph_display(uint8_t data){
  // button mode display on bargraph
  SPDR = data;  //send encoder value to the display
  while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent
  PORTB |= 0x01;            //send rising edge to regclk on HC595
  PORTB &= 0xFE;           //send falling edge to regclk on HC595
}//bar_graph_display()

//***********************************************************************************
// check increment/ decrement encoder for input
// Next, update data drom buttons
/***************************************************************************************/
void inc_dec_check(uint8_t encoder){
  // if increment encoder is in resting position, the previous position is used
  // to determine if CW or CCW rotation, then increment/decrement accordingly
  if( !( !(encoder&(1<<3)) | !(encoder&(1<<2)) ) ){
      if( !(last_encoder_val & (1<<3)) ){disp_value = disp_value + increment;}
      if( !(last_encoder_val & (1<<2)) ){disp_value = disp_value - increment;}
  }

      // if neither mode bits are selected
      if( !(buttons&(1<<0)) && !(buttons&(1<<1)) ){
        increment = 1;
      }

      // if both mode bits are selected
      else if( (buttons&(1<<0)) && (buttons&(1<<1)) ){
        increment = 0;
      }

      // if increment by 2 (only) bit is selected
      else if(((buttons&(1<<0)) && !(buttons&(1<<1)))){
        increment = 2;
      }

      // if increment by 4 (only) bit is selected
      else if((!(buttons&(1<<0)) && (buttons&(1<<1)))){
        increment = 4;
      }
}


ISR(TIMER0_OVF_vect){

  uint8_t encoder;

  encoder = spi_read();         // get data from encoder
  base_mode_check(encoder);     // check base mode and toggle if necessary
  inc_dec_check(encoder);       // increment/decrement if necessary
  last_encoder_val = encoder;   // record previous encoder value;
  bar_graph_display(buttons);   // display base mode toggle status on bar graph

  segsum();

}//TIMER0_OVF_vect

/***************************************************************************************/
// initialization upon startup
/***************************************************************************************/
void init() {

  // set global variables to known values
  disp_value = 0;
  spi_init();
  last_encoder_val = 0x11;
  increment = 1;
  buttons = 0x00;
  base_mode = 0;

  // Set PB7 (!OE from bar graph) LOW
  DDRB |= (1<<7);
  PORTB |= (0<<7);

  TCCR0 |= (1<<CS02) | (0<<CS01) | (0<<CS00); //normal mode, clk prescaler = 64
  TIMSK |= (1<<TOIE0);                        //enable timer/coutner0 overflow interrupt
  sei();                                      // enable interrupts
}//init()

/***************************************************************************************/
//MAIN
/***************************************************************************************/
int main(){

  init();     // initializations

  while(1) {

  DDRF = 0xFF;    // set PORTD as output
  DDRA = 0xFF;   // set PORTA as output


  PORTF = 0x00; // Digit 3 ON
  PORTA = data_to_7seg[segment_data[0]]; // enable segments
  _delay_ms(0.1);

  PORTA = data_to_7seg[segment_data[1]]; // enable segments
  PORTF = 0x10; // Digit 2 ON
  _delay_ms(0.1);

  PORTA = data_to_7seg[segment_data[3]]; // enable segments
  PORTF = 0x30; // Digit 3 ON
  _delay_ms(0.1);

  PORTA = data_to_7seg[segment_data[4]]; // enable segments
  PORTF = 0x40; // Digit 4 ON
  _delay_ms(0.1);

  // If hex mode is on, turn on Led seg L3
  if(!base_mode){
    PORTA = 0xFB; // enable segments
    PORTF = 0x20; // Digit 3 ON
  }
  _delay_ms(0.4);

  DDRA = 0x00;
  // PORTA = 0xFF;

  //bound the count to 0 - 1023
  if(disp_value >= 1024) {disp_value = 0;}
  else if(disp_value < 0) {disp_value = 1023;}

  }
}
