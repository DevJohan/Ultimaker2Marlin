/*
  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include "MarlinSerial.h"

#include "Marlin.h"

#ifndef AT90USB
// this next line disables the entire HardwareSerial.cpp,
// this is so I can support Attiny series and any other chip without a uart
#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

#if UART_PRESENT(SERIAL_PORT)
  ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
  tx_ring_buffer tx_buffer  =  { { 0 }, 0, 0 };
#endif


FORCE_INLINE void store_char(unsigned char c)
{
  int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != rx_buffer.tail) {
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}


//#elif defined(SIG_USART_RECV)
#if defined(M_USARTx_RX_vect)
  // fixed by Mark Sproul this is on the 644/644p
  //SIGNAL(SIG_USART_RECV)
  SIGNAL(M_USARTx_RX_vect)
  {
    unsigned char c  =  M_UDRx;
    store_char(c);
  }
#endif

  ISR( M_USARTx_UDRE_vect ){
	  if(tx_buffer.tail != tx_buffer.head){
		  M_UDRx = tx_buffer.buffer[tx_buffer.tail];
		  tx_buffer.tail = ( tx_buffer.tail + 1 ) % TX_BUFFER_SIZE;
	  }else
		  cbi(M_UCSRxB, M_UDRIEx);
  }

// Constructors ////////////////////////////////////////////////////////////////

MarlinBinarySerial::MarlinBinarySerial()
{

}

// Public Methods //////////////////////////////////////////////////////////////

void MarlinBinarySerial::begin(long baud)
{
  uint16_t baud_setting;
  bool useU2X = true;

#if F_CPU == 16000000UL && SERIAL_PORT == 0
  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    useU2X = false;
  }
#endif

  if (useU2X) {
    M_UCSRxA = 1 << M_U2Xx;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    M_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  M_UBRRxH = baud_setting >> 8;
  M_UBRRxL = baud_setting;

  sbi(M_UCSRxB, M_RXENx);
  sbi(M_UCSRxB, M_TXENx);
  sbi(M_UCSRxB, M_RXCIEx);
  sbi(M_UCSRxB, M_UDRIEx);
}

void MarlinBinarySerial::end()
{
  cbi(M_UCSRxB, M_RXENx);
  cbi(M_UCSRxB, M_TXENx);
  cbi(M_UCSRxB, M_RXCIEx);
  cbi(M_UCSRxB, M_UDRIEx);
}



int MarlinBinarySerial::peek(void)
{
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    return rx_buffer.buffer[rx_buffer.tail];
  }
}

int MarlinBinarySerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}

void MarlinBinarySerial::flush()
{
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  rx_buffer.head = rx_buffer.tail;
}




// Private Methods /////////////////////////////////////////////////////////////

// Preinstantiate Objects //////////////////////////////////////////////////////


MarlinBinarySerial MSerial;

#endif // whole file
#endif // !AT90USB
