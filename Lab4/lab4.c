/*
Chelsea Starr
ECE 473
Lab 4
*/

/*
NEXT:
- enable volume pwm
*/

/*
BUGS:
- if mintue updates while snooze is on, alarm does not go off when snooze is over
- speaker short
*/

/*
EXTRAS
- A mode that approximates a sine wave would sound better than the current pwm square wave
*/


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "kellen_music.c"

uint8_t segment_data[5];  // each element corresponds to digit (or colon)
uint16_t adc_result;

//data to 7-segment LED display encodings
uint8_t data_to_7seg[18] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x98, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E, 0xFF, 0xFC}; //0-F, off, colon
int16_t time_val;         // encoded time is a number that stores 0-1339. each digit corresponds to a minute of the day
int16_t alarm_val;
int16_t disp_time;
uint8_t last_encoder_val; // stores previos encoder value
uint16_t increment;       // stores increment setting
uint8_t buttons;          // holds buttons' toggle status
uint8_t snooze_tracker;



static uint8_t seconds=0; //hold value of seconds between interrupts
static uint8_t count=0; //hold value of count between interrupts

// macros to clarify bits in buttons
#define clk_edit 0
#define alrm_edit 1
#define mode 2
#define alrm_set 3
#define snooze 4

//***********************************************************************
//                            spi_init
//**********************************************************************
void spi_init(void){
  DDRB |=  0x17;                 //Turn on SS, MOSI, SCLK (SS is output)
  SPCR = (1<<SPE) | (1<<MSTR);  //SPI enabled, master, low polarity, MSB 1st
  SPSR = (1<<SPI2X);            //run at i/o clock/2

  /* Run this code before attempting to write to the LCD.*/
  DDRF  |= 0x08;  //port F bit 3 is enable for LCD
  PORTF &= 0xF7;  //port F bit 3 is initially low

  DDRB  |= 0x07;  //Turn on SS, MOSI, SCLK
  PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
 //see: /$install_path/avr/include/avr/iom128.h for bit definitions

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
//******************************************************************************
uint8_t chk_buttons(uint8_t button) {
    static uint16_t state[8] = {0,0,0,0,0,0,0,0};  //holds present state for each button

    //make PORTA an input port with pullups
    DDRA = 0x00;  // Set portA as input
    PORTA = 0xFF; // Pull HIGH

    //enable tristate buffer for pushbutton switches
    PORTC = 0x70; // enable COM_EN

    state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
    if (state[button] == 0xF000) { return 1; }
    return 0;
}//chk_buttons()

//***********************************************************************************
//                                   segment_sum
// takes a 16 bit binary number that represents minutes in a day an converts that to time
// in either 12 or 24 hour format. Results corresponding to each digit are stored in array.
/***************************************************************************************/
void segsum(int16_t value) {
  uint8_t d0, d1, d2, d3;
  uint8_t mins, hrs;

  mins = value % 60;
  hrs = value/60;

  // if in 12hr format and it is afternoon, reformat 24hr time accordingly (by subracting 12)
  // if( (buttons&(1<<0)) & (hrs > 12) ) { hrs = hrs - 12; }
  if( (buttons&(1<<mode)) && (hrs > 12) ) { hrs = hrs - 12; }

  //break mins and hrs into 2 sets of 2 digit representations
  d3 = hrs/10;
  d2 = hrs - d3*10;
  d1 = mins/10;
  d0 = mins - d1*10;

  //now move data to right place for misplaced colon position
  segment_data[0] = d0;
  segment_data[1] = d1;
  segment_data[3] = d2;
  segment_data[4] = d3;

  //bound the count to 0 - 1439 (1439 is the max amount of minutes in a day, corresponding to 11:59pm)
  if(time_val >= 1440) {time_val = 0;}
  else if(time_val < 0) {time_val = 1439;}
  if(alarm_val >= 1440) {alarm_val = 0;}
  else if(alarm_val < 0) {alarm_val = 1439;}

}//segsum()


//***********************************************************************************
// check button data
/***************************************************************************************/
void mode_check(){

    //Enable buttons to be read
    DDRA = 0x00;  // Set portA as input
    PORTA = 0xFF; // Pull HIGH
    PORTC = 0x70; // enable COM_EN

    // check each button and toggle if it is pressed
    for (int i = 0; i < 5; i++)
    {
      if(chk_buttons(i)){
        buttons ^= 1 << i;  //toggle button value
        if (i == snooze && !(buttons&(1<<alrm_set)) ) { buttons ^= 1 << snooze; } //ignore snooze if alarm is not set
      }

      // if both alarm and time select modes get selected, deselect both
      if( (buttons&(1<<clk_edit)) && (buttons&(1<<alrm_edit)) ) { buttons &= (0b11111100); }

      // if snooze is set but alarm is not, deselect snooze
      if( (buttons&(1<<snooze)) && (!(buttons&(1<<alrm_set)) || (alarm_val != time_val)) ) { buttons &= (0b11101111); }
    }

}//mode_check()

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
// Next, update data from buttons
/***************************************************************************************/
void inc_dec_check(uint8_t encoder, int16_t *value){
  // if increment encoder is in resting position, the previous position is used
  // to determine if CW or CCW rotation, then increment/decrement accordingly

  // this block checks minute encoder
  if( !( !(encoder&(1<<3)) | !(encoder&(1<<2)) ) ){
      if( !(last_encoder_val & (1<<3)) ){*value = *value + 1;}
      if( !(last_encoder_val & (1<<2)) ){*value = *value - 1;}
  }

  // this block checks hour encoder
  if( !( !(encoder&(1<<1)) | !(encoder&(1<<0)) ) ){
      if( !(last_encoder_val & (1<<1)) ){*value = *value + 60;}
      if( !(last_encoder_val & (1<<0)) ){*value = *value - 60;}
  }

  last_encoder_val = encoder;   // record previous encoder value;
}

//***********************************************************************************
// get data from encoders
/***************************************************************************************/
void read_encoders(int16_t *value){
    uint8_t encoder;

    encoder = spi_read();         // get data from encoder
    inc_dec_check(encoder, value);       // increment/decrement if necessary

}

//***********************************************************************************
// ISR to keep time. Runs every 1 second
/***************************************************************************************/
ISR(TIMER0_OVF_vect){

  // static uint8_t count=0; //hold value of count between interrupts

  count++; //extend counter

  if((count) == 128){
    seconds++;
    count = 0;
    //toggle colon each time this happens
    if (seconds >= 60){
      seconds = 0;
      time_val = time_val + 1;
    }
    if ((buttons&(1<<snooze) && 0b00010000 && (snooze_tracker > 0))){ //if snooze is enabled
      snooze_tracker--;
    }
  }
  if(count % 8 == 0) {
    //for note duration (64th notes)
    beat++;
  }


}//TIMER0_OVF_vect

/***************************************************************************************/
// initialization upon startup
/***************************************************************************************/
void init() {

  // set global variables to known values
  time_val = 0;
  alarm_val = 0;
  spi_init();
  last_encoder_val = 0xFF;
  increment = 1;
  buttons = 0x00;

  //initialize lcd
  lcd_init();

  //Initalize ADC and its ports
  DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input
  PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
  ADMUX |= (1<<REFS0) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0);//single-ended, input PORTF bit 7, right adjusted, 10 bits
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //ADC enabled, don't start yet, single shot mode, division factor is 128 (125khz)

  // initialize timer/couter 0 for time keeping isr
  TCCR0 |= (1<<CS00);  //normal mode, no prescaling
  ASSR  |= (1<<AS0);
  TIMSK |= (1<<TOIE0);

  sei();                                  // enable interrupts
}//init()

/***************************************************************************************/
//write data to 7 seg display
/***************************************************************************************/
// void write_to_7seg(int16_t *value){
void write_to_7seg(){

  // int16_t *value = &alarm_val;

  segsum(disp_time);

  // DDRC = 0xFF;    // set PORTD as output
  DDRA = 0xFF;   // set PORTA as output


  PORTC = 0x00; // Digit 3 ON
  PORTA = data_to_7seg[segment_data[0]]; // enable segments
  _delay_ms(0.5);

  PORTA = data_to_7seg[segment_data[1]]; // enable segments
  PORTC = 0x10; // Digit 2 ON
  _delay_ms(0.5);


  if ( !((buttons &(1<<alrm_edit)) || (buttons &(1<<clk_edit))) && seconds%2 == 0) {PORTA = data_to_7seg[16];}
  else {PORTA = data_to_7seg[17];}

  PORTC = 0x20; // Digit 2 ON
  _delay_ms(0.5);

  PORTA = data_to_7seg[segment_data[3]]; // enable segments
  PORTC = 0x30; // Digit 3 ON
  _delay_ms(0.5);

  PORTA = data_to_7seg[segment_data[4]]; // enable segments
  PORTC = 0x40; // Digit 4 ON
  _delay_ms(0.5);

  DDRA = 0x00;

}


/***************************************************************************************/
// PWM for 7seg led display brightness via pb7 (oc2) using TCNT2
//
/***************************************************************************************/
void dimmer(){
  float dim_val = adc_result*0.25;

  if (dim_val < 32) { dim_val = 32; }     // sets minimum on how dim LEDs can get
  if (dim_val > 255 ) { dim_val = 255; }  // sets max so OCR2 does not overflow

  DDRB |= (1 << DDB7); // PB7 as output
  OCR2 = dim_val; // set PWM duty cycle (smaller number -> dimmer leds)

  TCCR2 |= (1 << COM21) | (1 << COM20); // set non-inverting mode
  TCCR2 |= (1 << WGM21) | (1 << WGM20); // set fast PWM Mode
  TCCR2 |= (1 << CS21); // set prescaler to 8 and starts PWM

}

void get_photocell_val()
{

  ADCSRA |= (1<<ADSC);//poke ADSC and start conversion
  while(bit_is_clear(ADCSRA, ADIF)){} //spin while interrupt flag not set
  ADCSRA |= (1<<ADIF);//its done, clear flag by writing a one

  adc_result = ADC;    //read the ADC output as 16 bits

}

/***************************************************************************************/
//write to LCD. FIX
/***************************************************************************************/
void lcd_write(char message[16]){
  int i;
  static char last_mssg[16];


  if(!strcmp(last_mssg, message)) {return;}
  // clear_display();
  set_cursor(1,0);
  for ( i = 0; i<16; i++){
    if (message[i] == '\0'){break;}
    char2lcd(message[i]);
  }
  strncpy(last_mssg, message, 15);

}

/***************************************************************************************/
//MAIN
/***************************************************************************************/
int main(){
  int music_flag = 0;

  music_init();
  init();     // initializations

  snooze_tracker = 10;

  DDRD = 0xFF;
  DDRC = 0xFF;

   while(1) {

     get_photocell_val();
     dimmer();
     mode_check();                   // check mode (buttons)
     bar_graph_display(buttons);     // display mode on bar graph
     write_to_7seg();

     set_cursor(2,14);
     if(buttons&(1<<mode)){int2lcd(12);}
     else{int2lcd(24);}

     if(buttons &(1<<alrm_edit)) {lcd_write("ALARM EDIT     "); read_encoders(&alarm_val); disp_time = alarm_val;}
     else if(buttons &(1<<clk_edit)) {lcd_write("CLOCK EDIT     "); read_encoders(&time_val); disp_time = time_val; }
     else if(buttons & (1<<alrm_set)) {
       disp_time = time_val;
       if( (alarm_val == time_val) && !(buttons &(1<<snooze)) ) {
         lcd_write("WAKE UP!       ");
         snooze_tracker = 10;
         if (!music_flag) {music_on(); music_flag = 1;}
         }
       else if( (alarm_val == time_val) && (buttons &(1<<snooze)) ) {
       lcd_write("SNOOZING       ");
       music_off();
       music_flag = 0;
       set_cursor(1,15);
       int2lcd(snooze_tracker);
       if(snooze_tracker == 0) {snooze_tracker = 10; alarm_val = time_val; buttons ^= 1 << snooze;}
      }
       else if( snooze_tracker != 10 ) {
         lcd_write("SNOOZING       ");
         set_cursor(1,15);
         int2lcd(snooze_tracker);
         if(snooze_tracker == 0) {snooze_tracker = 10; alarm_val = time_val; }//buttons ^= 1 << snooze;}
       }
       else{lcd_write("ALARM SET      "); music_off(); music_flag = 0;}
     }
     else{lcd_write("ALARM OFF      "); music_off(); music_flag = 0;disp_time = time_val; music_off();}
  }

}
