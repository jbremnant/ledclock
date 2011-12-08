#ifndef F_CPU
#define F_CPU 8000000
#endif

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <inttypes.h>
#include <util/delay.h>
#include <stdlib.h>
#include "uart.h"
#include "ht1632.h"

// #define BUTTON1 PD1
// #define BUTTON1DRAIN PD2
#define BUTTON2 PD3
#define BUTTON2DRAIN PD4

#define BT_PRESS 0x02
#define BT_HOLD  0x04

// define some notes
// Frequencies from http://www.phy.mtu.edu/~suits/notefreqs.html
// converted to half-periods (us) by calculating
//  1000000/2/frequency
// where frequency is in Hz
#define D5 851
#define E5 758
#define Fsh5 675
#define G5 637
#define A5 568
#define B5 506
#define C6 477
#define D6 425
#define DUR 40


volatile uint8_t seconds,minutes,hours;
volatile uint8_t days,months,years,leap_year;
volatile unsigned char flag = 0;
volatile uint8_t last_buttonstate = 0;
volatile int intcount = 0;

void update_datetime(uint8_t);

void rtc_init(void)
{
  TCCR2A = 0x02;//clear timer on compare match
  // TCCR2B = (1<<CS21) | (1<<CS20); // prescale by 32: gives 1024 ticks. pg.158
  TCCR2B = 0x07; // prescale by 1024, gives 32 ticks 
  TIFR2  = 0x02;//clear the interrupt flag
  TIMSK2 = 0x02;//enable timer2A output compare match interrupt
  OCR2A  = 31;  //Set the output compare to 31 since the count starts at 0 - this gives 32 interrups per sec
  ASSR   = 0x20;//enable asynchronous mode, disable this line if you are using the 32khz crystal as a the system clock source
}


void key_init()
{
  //Enable PIN Change Interrupt 1 - This enables interrupts on pins
  //  //PCINT14...8 see p70 of datasheet
  PCICR |= (1<<PCIE2);

  //Set the mask on Pin change interrupt 1 so that only PCINT12 (PC4) triggers
  //the interrupt. see p71 of datasheet
  // PCINT17 (PD1) for hours, PCINT19 (PD3) for minutes
  PCMSK2 |= (1<<PCINT17) | (1<<PCINT19);

  DDRD &= ~(1<<BUTTON2);
  DDRD |=  (1<<BUTTON2DRAIN);

  PORTD |=  (1<<BUTTON2);       // high
  PORTD &= ~(1<<BUTTON2DRAIN);  // low
}

void extra_init(void)
{
  // led and piezo
  DDRD  |= (1<<PD6) | (1<<PD7);
  PORTD &= ~(1<<PD7); // provides ground for led & piezo
}


void play_tone(uint16_t delay, uint8_t duration)
{
  // delay is half-period in microseconds
  // duration is in 10ms increments
  
  // example: 440Hz --> delay=1136
  
  // duration = 2*delay * cycles (all in same units)
  // cycles = 10000 * duration / delay / 2
  // cycles = 100 * duration / (delay/50)
  uint16_t tmp = 100 * duration;
  uint16_t delaysm = delay / 50;
  uint16_t cycles = tmp / delaysm;
  
  while(cycles > 0) {
    PORTD |= (1<<PD6);
    _delay_us(delay);
    PORTD &= ~(1<<PD6);
    _delay_us(delay);
    cycles--;
  }
}

ISR(TIMER2_COMPA_vect)
{
  // sei();
  // when Timer0 gets to its Output Compare value,
  // one one-hundredth of a second has elapsed (0.01 seconds).
  update_datetime(0);
  // } 
}


ISR(PCINT2_vect)
{
  // if the button went low, the button was pressed
  if(!(PIND & (1<<BUTTON2)))
  {
    intcount++;

    // was not pressed before
    if(! (last_buttonstate & BT_PRESS))
    { 
      _delay_ms(5);                  // debounce
      if( PIND & (1<<BUTTON2) )      // filter out bounces
        return;

      last_buttonstate |= BT_PRESS;

      minutes++;
      update_datetime(1);
      play_tone(D5, 2);
      // uart_write('a');

      // check if it's still tied to ground
      while(!(PIND & (1<<BUTTON2)))
      {
        if(++intcount > 0xAA00)
        {
          last_buttonstate |= BT_HOLD;
          play_tone(A5, 2);
          return;
        }
      }
    }
  }
  else
  {
    intcount = 0;
    last_buttonstate &= ~(BT_PRESS | BT_HOLD);
  }
}


// SIGNAL(SIG_OUTPUT_COMPARE0A)
// ISR(TIMER2_COMPA_vect)
void update_datetime(uint8_t noincrement)
{
  if(!noincrement)
    seconds++;

  if(seconds >= 60)
  {
    seconds = 0;
    minutes++;
  }
  if(minutes >= 60)
  {
    minutes = 0;
    hours++;
  }
  if(hours >= 24)
  {
    hours = 0;
    days++;
    if( (days == 32) ||
      (days == 31 && months == (4|6|9|11)) ||
      (days == 30 && months == 2 && leap_year != 0) ||
      (days == 29 && months == 2 && leap_year == 0) )
    {
      days = 1;
      months++;
      if(months == 13)
      {
        months = 1;
        years++;
        if( (years % 400 == 0) ||
          (years % 4 == 0 && years % 100 != 0) )
        {
          leap_year = 1;
        } else {
          leap_year = 0;
        }
      }
    }
  }
}

void set_time(char* str)
{
  str[0] = (hours / 10) + '0';
  str[1] = (hours % 10) + '0';
  str[2] = (minutes / 10) + '0';
  str[3] = (minutes % 10) + '0';
  str[4] = (seconds / 10) + '0';
  str[5] = (seconds % 10) + '0';
  str[6] = 0;
}

void disp_bindots(void)
{
  uint8_t ytop = 1;
  // let's do binary clock thing here
  ht1632_sendbyte(28,ytop,   hours>>4,   1);    // upper word
  ht1632_sendbyte(28,ytop+4, hours<<0,   1);    // lower word
  ht1632_sendbyte(29,ytop,   minutes>>4, 1);    // upper word
  ht1632_sendbyte(29,ytop+4, minutes<<0, 1);    // lower word
  ht1632_sendbyte(30,ytop,   seconds>>4, 1);    // upper word
  ht1632_sendbyte(30,ytop+4, seconds<<0, 1);    // lower word
}

void disp_time(char *timebuf)
{
  static int8_t ppos = 0;
  int8_t pos = 0, ytop=1;
  ht1632_clear();

  ht1632_putchar(0,ytop,timebuf[0], 1);
  ht1632_putchar(6,ytop,timebuf[1], 1);
  ht1632_do_plot(12,ytop+2,1,0,1);
  ht1632_do_plot(12,ytop+4,1,0,1);
  ht1632_putchar(14,ytop,timebuf[2], 1);
  ht1632_putchar(20,ytop,timebuf[3], 1);
  ht1632_syncram();
  ppos = pos;
}

void disp_time_seconds(char *timebuf)
{
  static int8_t ppos = 0;
  int8_t pos = 0;
  ht1632_clear();

  // the last arg is 'ramonly' which updates just the shadow_ram
  // Once all the digits are written in ram, we then sync the
  // entire thing in one shot - should be faster.
  ht1632_putchar(0,0,timebuf[0], 1);
  ht1632_putchar(5,0,timebuf[1], 1);
  ht1632_putchar(11,0,timebuf[2], 1);
  ht1632_putchar(16,0,timebuf[3], 1);
  ht1632_putchar(22,0,timebuf[4], 1);
  ht1632_putchar(27,0,timebuf[5], 1);

  pos = seconds%32; 
  ht1632_do_plot(pos, 7,1,0,1);
  ht1632_do_plot(ppos,7,0,0,1);

  ht1632_syncram();
  ppos = pos;
}

void sleep_and_wake()
{
  sleep_enable();    // set the sleep enable bit in the SMCR register
  sleep_cpu();       // use the sleep command
  sleep_disable();  // unset th esleep enable bit, for safety
  OCR2B = flag;
}



int main()
{
  uint8_t i = 0;
  uint8_t pos;
  char strbuf[64];
  char timebuf[16];
  char ch, tok;
  uint8_t seconds_last = seconds;
  uint8_t minutes_last = minutes;
  uint8_t j, timegood = 0;
  uint8_t digits[4] = {0,0,0,0};
  

  uart_init();
  FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
  stdin = stdout = &uart_stream;

  rtc_init();
  key_init();
  extra_init();
  sei(); // turn on interrupt handler

  ht1632_setup();
  pos = 1;
  ht1632_putchar(pos, 0, 'r', 0); pos+=6;
  ht1632_putchar(pos, 0, 'e', 0); pos+=6;
  ht1632_putchar(pos, 0, 'a', 0); pos+=6;
  ht1632_putchar(pos, 0, 'd', 0); pos+=6;
  ht1632_putchar(pos, 0, 'y', 0);
  // ht1632_syncram();
  _delay_ms(1000);        

  set_time(timebuf);
  disp_time(timebuf);

  while(1)
  {
    // sleep_and_wake();
    // main clock is a lot faster than uart baud
    if(uart_char_is_waiting())
    {
      ch = uart_read();

      if(ch != '\n')
        strbuf[i++] = ch;

      uart_write(ch); // echo

      if(i>64)
      {
        uart_write('n'); uart_write('g'); uart_write('\n');
        i = 64;
      }

      // display scrolling text
      if(ch == '\n')
      {
        // if the first byte was command indicator ':'
        if(strbuf[0] == ':')
        {
          // should contain 4 digits: hhmm
          if(i>6 && strbuf[1] == 't')
          {
            play_tone(A5, 10); _delay_ms(10);
            play_tone(B5, 10); _delay_ms(10);

            timegood = 1; 
            for(j=0; j<4; j++)
            {
              tok = strbuf[j+2];
              if(tok >= '0' && tok <= '9')
              {
                digits[j] = tok - '0';      
              }
              else
              {
                timegood = 0;
              }
            }

            if(timegood)
            {
              play_tone(B5, 5); _delay_ms(2);
              play_tone(D6, 5);
              hours   = 10*digits[0] + digits[1];
              minutes = 10*digits[2] + digits[3];
              // report time no good (:tng)
              uart_write(':'); uart_write('t');
              for(j=0; j<4; j++)
                uart_write(digits[j]+'0');
              uart_write('\n');
            }
            else
            {
              play_tone(G5, 20); _delay_ms(5);
              play_tone(G5, 20);
              // report time no good (:tng)
              uart_write(':'); uart_write('t');
              uart_write('n'); uart_write('g');
              uart_write('\n');
            }
          }
        }
        else
        {
          play_tone(A5, 4); _delay_ms(5);
          play_tone(A5, 4); _delay_ms(10);
          play_tone(B5, 5); _delay_ms(5);
          play_tone(E5, 8);

          // i-1 because it's sending \r\n as newline
          ht1632_scrollstr(strbuf, i-1, 30);

          uart_write('0'); uart_write('x');
          uart_write( (i >> 4) + '0' );
          uart_write( (0x0F & i) + '0' );
          uart_write('\n');
          // uart_write('0'+i); uart_write('\n');
        }

        i  = 0;
        ch = 0;

        _delay_ms(500);
        set_time(timebuf);
        disp_time(timebuf);
  
        continue;
      }
    }

    if(seconds_last != seconds || last_buttonstate & BT_PRESS) 
    {
      seconds_last = seconds;

      if(minutes_last != minutes)
      {
        minutes_last = minutes;
        set_time(timebuf);
        disp_time(timebuf);
      }

      disp_bindots();
    }

    // fast cycle
    if(last_buttonstate & BT_HOLD)
    {
      while(!(PIND & (1<<BUTTON2)))
      {
        minutes++;
        update_datetime(1);
        set_time(timebuf);
        disp_time(timebuf);
        minutes_last = minutes;
        _delay_ms(20);
      }
      intcount = 0;
    }

    while ((ASSR & (1<<OCR2BUB)) != 0) {} 
  }
}
