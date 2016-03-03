/*
  HardwareSerial.h - Hardware serial library for Wiring
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

  Modified 28 September 2010 by Mark Sproul
*/

#ifndef MarlinSerial_h
#define MarlinSerial_h
#include "Marlin.h"
#include "../CommunicationsBridge/printer_data_types.h"

#if !defined(SERIAL_PORT)
#define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
						(port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
						(port == 3 && defined(UBRR3H)))

// These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
// requires two levels of indirection to expand macro values properly)
#define SERIAL_REGNAME(registerbase,number,suffix) SERIAL_REGNAME_INTERNAL(registerbase,number,suffix)
#if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##suffix
#else
#define SERIAL_REGNAME_INTERNAL(registerbase,number,suffix) registerbase##number##suffix
#endif

// Registers used by MarlinSerial class (these are expanded
// depending on selected serial port
#define M_UCSRxA SERIAL_REGNAME(UCSR,SERIAL_PORT,A) // defines M_UCSRxA to be UCSRnA where n is the serial port number
#define M_UCSRxB SERIAL_REGNAME(UCSR,SERIAL_PORT,B)
#define M_RXENx SERIAL_REGNAME(RXEN,SERIAL_PORT,)
#define M_TXENx SERIAL_REGNAME(TXEN,SERIAL_PORT,)
#define M_RXCIEx SERIAL_REGNAME(RXCIE,SERIAL_PORT,)
#define M_UDRIEx SERIAL_REGNAME(UDRIE,SERIAL_PORT,)
#define M_UDREx SERIAL_REGNAME(UDRE,SERIAL_PORT,)
#define M_UDRx SERIAL_REGNAME(UDR,SERIAL_PORT,)
#define M_UBRRxH SERIAL_REGNAME(UBRR,SERIAL_PORT,H)
#define M_UBRRxL SERIAL_REGNAME(UBRR,SERIAL_PORT,L)
#define M_RXCx SERIAL_REGNAME(RXC,SERIAL_PORT,)
#define M_USARTx_RX_vect SERIAL_REGNAME(USART,SERIAL_PORT,_RX_vect)
#define M_USARTx_UDRE_vect SERIAL_REGNAME(USART,SERIAL_PORT,_UDRE_vect)
#define M_U2Xx SERIAL_REGNAME(U2X,SERIAL_PORT,)



#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0


#ifndef AT90USB
// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 64

struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
};

struct tx_ring_buffer
{
  unsigned char buffer[TX_BUFFER_SIZE];
  int head;
  int tail;
};

#if UART_PRESENT(SERIAL_PORT)
  extern ring_buffer rx_buffer;
  extern tx_ring_buffer tx_buffer;
#endif

  template<typename... args>
  struct send_impl;
  template<typename arg_t>
  struct send_type;
  template<typename arg_t,size_t data_pos,size_t data_size>
  struct send_type_default_impl;

  template <typename arg_t>
  struct sized_array{
  	uint8_t data_size;
  	arg_t* data;
  };
  template <typename arg_t,uint8_t data_size>
  struct fixed_sized_array{
  	arg_t* data;
  };


class MarlinBinarySerial //: public Stream
{

  public:
    MarlinBinarySerial();
    void begin(long);
    void end();
    int peek(void);
    int read(void);
    void flush(void);

    FORCE_INLINE int available(void)
    {
      return (unsigned int)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
    }

    FORCE_INLINE void write_blocking(){
    	critical_section_guard lock;
    	if(tx_buffer.tail != tx_buffer.head){
    		while(!((M_UCSRxA) & (1 << M_UDREx)));
    		M_UDRx = tx_buffer.buffer[tx_buffer.tail];
    		tx_buffer.tail = ( tx_buffer.tail + 1 ) % TX_BUFFER_SIZE;
    	}
    }

    FORCE_INLINE void write(uint8_t c)
    {
    	int i = (unsigned int)(tx_buffer.head + 1) % TX_BUFFER_SIZE;
    	if( i == tx_buffer.tail )
    		write_blocking();
    	tx_buffer.buffer[tx_buffer.head] = c;
    	tx_buffer.head = i;
    	sbi(M_UCSRxB, M_UDRIEx);
    }

    FORCE_INLINE void checkTx(){
    	if(tx_buffer.tail != tx_buffer.head){
    		critical_section_guard lock;
    		if(((M_UCSRxA) & (1 << M_UDREx))){
    			M_UDRx = tx_buffer.buffer[tx_buffer.tail];
    			tx_buffer.tail = ( tx_buffer.tail + 1 ) % TX_BUFFER_SIZE;
    		}
    	}else
    		cbi(M_UCSRxB, M_UDRIEx);
    }

    FORCE_INLINE void checkRx(void)
    {
      if((M_UCSRxA & (1<<M_RXCx)) != 0) {
        unsigned char c  =  M_UDRx;
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
    }


    private:


  public:

    template<printer_message message_type, typename... arg_ts>
    void send( arg_ts... args){
    	send_impl<printer_message,arg_ts...>::send(*this,message_type, args...);
    }

    template<printer_message message_type, typename sub_t, sub_t sub_message, typename... arg_ts>
    void send_sub_message( arg_ts... args ){
    	send_impl<printer_message,sub_t,arg_ts...>::send(*this,message_type,sub_message, args...);
    }

//    FORCE_INLINE void write(const char *str)
//    {
//      while (*str)
//        write(*str++);
//    }
//
//
//    FORCE_INLINE void write(const uint8_t *buffer, size_t size)
//    {
//      while (size--)
//        write(*buffer++);
//    }

};


template<typename arg_t, size_t data_pos, size_t data_size>
struct send_type_default_impl{
	static void send(MarlinBinarySerial& serial_port, arg_t data){
		serial_port.write(reinterpret_cast<const uint8_t*>(&data)[data_pos]);
		send_type_default_impl<arg_t,data_pos+1,data_size>::send(serial_port,data);
	}
};
template<typename arg_t,size_t data_size>
struct send_type_default_impl<arg_t,data_size,data_size>{
	static void send(MarlinBinarySerial& serial_port, arg_t data){}
};

template<typename arg_t>
struct send_type{
	static void send(MarlinBinarySerial& serial_port, arg_t data){
		send_type_default_impl<arg_t,0,sizeof(arg_t)>::send(serial_port, data);
	}};

template<typename arg_t>
struct send_type<arg_t*>{
	static void send(MarlinBinarySerial& serial_port, arg_t* data){
		send_type_default_impl<arg_t,0,sizeof(arg_t)>::send(serial_port, data);
	}};

template<typename arg_t>
struct send_type<sized_array<arg_t>>{
	static void send(MarlinBinarySerial& serial_port, sized_array<arg_t> data){
		serial_port.write( static_cast<uint8_t>(data.data_size) );
		arg_t* d = data.data;
		for(uint8_t i = data.data_size;i!=0;--i){
			send_type_default_impl<arg_t,0,sizeof(arg_t)>::send(serial_port, *(d++));
		}
	}};
template<typename arg_t,uint8_t data_size>
struct send_type<fixed_sized_array<arg_t,data_size>>{
	static void send(MarlinBinarySerial& serial_port, fixed_sized_array<arg_t,data_size> data){
		arg_t* d = data.data;
		for(uint8_t i = data_size;i!=0;--i){
			send_type_default_impl<arg_t,0,sizeof(arg_t)>::send(serial_port, *(d++));
		}
	}};

template<>
struct send_type<const char*>{
	static void send(MarlinBinarySerial& serial_port, const char* data){
		int data_len = strlen(data);
		if(data_len < 255){
			serial_port.write( static_cast<uint8_t>(data_len) );
			for( uint8_t i=data_len; i != 0; --i )
				serial_port.write( *data++ );
		}
	}};

template<typename arg_t, typename... rest_t>
struct send_impl<arg_t,rest_t...>{
	static void send(MarlinBinarySerial& serial_port, arg_t arg, rest_t... rest_arg){
		send_type<arg_t>::send(serial_port, arg);
		send_impl<rest_t...>::send(serial_port, rest_arg...);
	}
};

template<typename arg_t>
struct send_impl<arg_t>{
	static void send(MarlinBinarySerial& serial_port, arg_t arg){
		send_type<arg_t>::send(serial_port, arg);
	}
};

extern MarlinBinarySerial MSerial;
#endif // !AT90USB

#endif
