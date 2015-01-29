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

static i2cCommand issue_read_fiset_cond_command;
static uint8_t issue_read_fiset_cond_buffer[1];
static i2cCommand read_fiset_cond_command;
static uint8_t read_fiset_cond_buffer[2];

static i2cCommand issue_read_fiset_value_command;
static uint8_t issue_read_fiset_value_buffer[1];
static i2cCommand read_fiset_value_command;
static uint8_t read_fiset_value_buffer[2];


void InitFiset(){

	issue_read_fiset_cond_buffer[0] = fiset_cond_hi_addr;
	i2cDriverCommandSetup(issue_read_fiset_cond_command, fiset_address | i2cWriteBit, fiset_priority, issue_read_fiset_cond_buffer, sizeof(issue_read_fiset_cond_buffer));
	i2cDriverCommandSetup(read_fiset_cond_command, fiset_address | i2cReadBit, fiset_priority, read_fiset_cond_buffer, sizeof(read_fiset_cond_buffer));

	issue_read_fiset_cond_buffer[0] = fiset_value_hi_addr;
	i2cDriverCommandSetup(issue_read_fiset_value_command, fiset_address | i2cWriteBit, fiset_priority, issue_read_fiset_value_buffer, sizeof(issue_read_fiset_value_buffer));
	i2cDriverCommandSetup(read_fiset_value_command, fiset_address | i2cReadBit, fiset_priority, read_fiset_value_buffer, sizeof(read_fiset_value_buffer));

}

