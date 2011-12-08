// uart.c
// for NerdKits with ATmega168, 14.7456 MHz clock
// mrobbins@mit.edu

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <inttypes.h>
#include "uart.h"

#ifndef F_CPU
#define F_CPU 8000000
#endif

#define BAUD 38400


void uart_init_old() {
  // set baud rate
  UBRR0H = 0;
  UBRR0L = 12;	// for 38400 bps :  8000000 / (16*38400) - 1 = 12.02083

  // enable uart RX and TX
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  // set 8N1 frame format
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);

  // set up STDIO handlers so you can use printf, etc
  fdevopen(&uart_putchar, &uart_getchar);
}

void uart_init()
{
  UBRR0H = (((F_CPU/BAUD/16)-1) >> 8);
  UBRR0L = ((F_CPU/BAUD/16)-1);
  UCSR0C = (3 << UCSZ00);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  fdevopen(&uart_putchar, &uart_getchar);
}


void uart_write(char x) {
  // wait for empty receive buffer
  while ((UCSR0A & (1<<UDRE0))==0);
  // send
  UDR0 = x;
}

uint8_t uart_char_is_waiting() {
  // returns 1 if a character is waiting
  // returns 0 if not
  return (UCSR0A & (1<<RXC0));
}

char uart_read() {
  // wait
  while(!uart_char_is_waiting());
  char x = UDR0;
  return x;
}

int uart_putchar(char c, FILE *stream) {
  uart_write(c);
  return 0;
}
    
int uart_getchar(FILE *stream) {
  int x = uart_read();
  return x;
}


