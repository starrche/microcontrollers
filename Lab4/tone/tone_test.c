// tcnt1_ctc.c
// setup TCNT1 in ctc mode
// set OC1A (PB5) to toggle on compare
// blink frequency ~= (16,000,000)/(2^15 * 64 * 2) = 3.8 cycles/sec //
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ALARM_PIN 0x03

void music_init(void) {
  //initially turned off (use music_on() to turn on)
  TIMSK |= (1<<OCIE1A); //enable timer interrupt 1 on compare
  TCCR1A = 0x00;        //TCNT1, normal port operation
  TCCR1B |= (1<<WGM12); //CTC, OCR1A = top, clk/64 (250kHz), compare match
  TCCR1C = 0x00;        //no forced compare
  OCR1A=0x11C0;         //(use to vary alarm frequency (tone))
  sei();

  TCCR1B |= (1<<CS11)|(1<<CS10); // enables ....
}

/*********************************************************************/
/*                             TIMER1_COMPA                          */
/*Oscillates pin7, PORTD for alarm tone output                       */
/*********************************************************************/
ISR(TIMER1_COMPA_vect) {
  PORTD ^= ALARM_PIN;      //flips the bit, creating a tone



  DDRC = 0xFF;    // set PORTD as output
  DDRA = 0xFF;   // set PORTA as output
  PORTC = 0x00; // Digit 3 ON
  PORTA = 0x92; // enable segments
  _delay_ms(0.1);
  DDRA = 0x00;

}


int main()
{
  DDRB |= 0x20; //set port B bit five to output
  //ctc mode, toggle on compare match
  TCCR1A |= (1<<COM1A0);
 //use OCR1A as source for TOP, use clk/64
  TCCR1B = (1<< WGM12) | (1<<CS10);
  TCCR1C = 0x00;    //no forced compare
  OCR1A = 0x7FFF;  //compare at half of 2^16
  while(1) {
  if (TIFR & (1<<OCF1A)) //if output compare flag is set
  TIFR |= (1<<OCF1A); //clear it by writing a one to OCF1A
  } //while
 // initialize timer/couter 0 for time keeping isr
 TCCR0 |= (1<<CS00);  //normal mode, no prescaling
 ASSR  |= (1<<AS0);
 TIMSK |= (1<<TOIE0);
 // music_init();
 // while(1){}
} // main
