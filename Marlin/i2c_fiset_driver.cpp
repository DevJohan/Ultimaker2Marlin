#include "i2c_fiset_driver.h"
#include "i2c_driver.h"

static const uint8_t fiset_address       = 0x40; // This can change
static const uint8_t fiset_priority      = 75;

static const uint8_t fiset_gain_addr     = 0xfa;
static const uint8_t fiset_diagnose_addr = 0xfb;
static const uint8_t fiset_cond_hi_addr  = 0xfc;
static const uint8_t fiset_cond_lo_addr  = 0xfd;
static const uint8_t fiset_value_hi_addr = 0xfe;
static const uint8_t fiset_value_lo_addr = 0xff;


static i2cCombinedCommand<2> fiset_read_all_command;
static uint8_t issue_fiset_read_all_buffer[1];
static uint8_t fiset_read_all_buffer[ fiset_value_lo_addr + 1 - fiset_gain_addr ];

static bool fiset_data_read;
static int16_t fiset_data_count = 0;

void fiset_callback( i2cCommand* cmd ){
	++fiset_data_count;
}

void fiset_init(){
	fiset_data_read = true;

	issue_fiset_read_all_buffer[0] = fiset_gain_addr;
	i2cDriverCommandSetup(fiset_read_all_command, fiset_priority, fiset_callback );
	i2cDriverCommandAddSequence(fiset_read_all_command, (fiset_address << 1 ) | i2cWriteBit, issue_fiset_read_all_buffer, sizeof(issue_fiset_read_all_buffer));
	i2cDriverCommandAddSequence(fiset_read_all_command, (fiset_address << 1) | i2cReadBit, fiset_read_all_buffer, sizeof(fiset_read_all_buffer));

}

void plan_read_fiset(){
	fiset_data_read = false;
	i2cDriverPlan( &fiset_read_all_command );
}

bool fiset_data_ready(){
	return  !fiset_data_read &&
			fiset_read_all_command.finished;
}

int16_t get_fiset_data_count(){
	return fiset_data_count;
}

int16_t get_fiset_data(){
	fiset_data_read = true;
	return
			(0xff & fiset_read_all_buffer[4])<<8 |
			(0xff & fiset_read_all_buffer[5])<<2;
}

uint8_t get_fiset_gain(){
	return
			fiset_read_all_buffer[0];
}

uint16_t get_fiset_magnitude(){
	return
			(0xff & fiset_read_all_buffer[2])<<8 |
			(0xff & fiset_read_all_buffer[3])<<2;
}
