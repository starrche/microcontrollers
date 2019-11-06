//Chelsea Starr

// lab2_skel.c
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138. QUESTION: a,b,c, not SEL0, SEL1, SEL2???
//  PORTB bit 7 goes to the PWM transistor base.

// NEXT: Debounce

#define F_CPU 16000000 // cpu speed in hertz
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];  // each element corresponds to digit (or colon)

//decimal to 7-segment LED display encodings, logic "0" turns on segment.
// index 1 has BCD 1, index 2 has BCD 2 ... BUT index 10 has BCD for blank digit
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x98, 0xFF};

// Value being incremented & displayed
uint16_t disp_value;


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

    state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) { return 1; }
    return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit
//BCD segment code in the array segment_data for display.
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//
void segsum() {
  uint16_t d0, d1, d2, d3;
  uint16_t temp = disp_value;


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
//***********************************************************************************


//***********************************************************************************
int main()
{
//set port bits 4-7 B as outputs
DDRF = 0xF0;

while(1){
  //insert loop delay for debounce
  /////////////////?

  //make PORTA an input port with pullups
  DDRA = 0x00;  // Set portA as input
  PORTA = 0xFF; // Pull HIGH

  //enable tristate buffer for pushbutton switches
  PORTF = 0x70; // enable COM_EN

  //now check each button and increment the count as needed
  for (int i = 0; i < 8; i++)
  {
    if(chk_buttons(i))
    {
      disp_value = disp_value + (0x01 << i);
    }
  }

  //disable tristate buffer for pushbutton switches
  // PORTB = 0x00; //enable Least Significant Digit

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

  //bound the count to 0 - 1023
  if(disp_value >= 1024) {disp_value = 0;}

  //break up the disp_value to 4, BCD digits in the array: call (segsum)
  segsum();

  //bound a counter (0-4) to keep track of digit to display
  //make PORTA an output
  //send 7 segment code to LED segments
  //send PORTB the digit to display
  //update digit to display
  }//while
}//main
