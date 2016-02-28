/* -*- c++ -*- */

/*
    Reprap firmware based on Sprinter and grbl.
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)

 It has preliminary support for Matthew Roberts advance algorithm
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "Marlin.h"

#include "ultralcd.h"
#include "UltiLCD2.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "lifetime_stats.h"
#include "electronics_test.h"
#include "language.h"
#include "pins_arduino.h"
#include "i2c_driver.h"
#include "i2c_capacitance.h"
#include "i2c_fiset_driver.h"
#include "fan_driver.h"

#include "../CommunicationsBridge/printer_to_remote.h"

#if NUM_SERVOS > 0
#include "Servo.h"
#endif

#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif


// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G10 - retract filament according to settings of M207
// G11 - retract recover filament according to settings of M208
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to coordinates given

//RepRap M Codes
// M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
// M1   - Same as M0
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M17  - Enable/Power all stepper motors
// M18  - Disable all stepper motors; same as M84
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M30  - Delete file from SD (M30 filename.g)
// M31  - Output time since last M109 or SD card start to serial
// M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Output current position to serial port
// M115 - Capabilities string
// M117 - display message
// M119 - Output Endstop status to serial port
// M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
// M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
// M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M200 - Set filament diameter
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
// M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
// M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
// M206 - set additional homeing offset
// M207 - set retract length S[positive mm] F[feedrate mm/sec] Z[additional zlift/hop]
// M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
// M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
// M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
// M220 S<factor in percent>- set speed factor override percentage
// M221 S<factor in percent>- set extrude factor override percentage
// M240 - Trigger a camera to take a photograph
// M280 - set servo position absolute. P: servo index, S: angle or microseconds
// M300 - Play beepsound S<frequency Hz> P<duration ms>
// M301 - Set PID parameters P I and D
// M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
// M304 - Set bed PID parameters P I and D
// M400 - Finish all moves
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// M503 - print the current settings (from memory not from eeprom)
// M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
// M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
// M907 - Set digital trimpot motor current using axis codes.
// M908 - Control digital trimpot directly.
// M350 - Set microstepping mode.
// M351 - Toggle MS1 MS2 pins directly.
// M923 - Select file and start printing directly (can be used from other SD file)
// M928 - Start SD logging (M928 filename.g) - ended by M29
// M999 - Restart after being stopped by error

//Stepper Movement Variables

//===========================================================================
//=============================imported variables============================
//===========================================================================


//===========================================================================
//=============================public variables=============================
//===========================================================================
#ifdef SDSUPPORT
CardReader card;
#endif
float homing_feedrate[] = HOMING_FEEDRATE;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply=100; //100->1 200->2
int saved_feedmultiply;
int extrudemultiply[EXTRUDERS]=ARRAY_BY_EXTRUDERS(100, 100, 100); //100->1 200->2
float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
float add_homeing[3]={0,0,0};
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
// Extruder offset, only in XY plane
#if EXTRUDERS > 1
float extruder_offset[2][EXTRUDERS] = {
#if defined(EXTRUDER_OFFSET_X) && defined(EXTRUDER_OFFSET_Y)
  EXTRUDER_OFFSET_X, EXTRUDER_OFFSET_Y
#endif
};
#endif
uint8_t active_extruder = 0;
uint8_t fanSpeed=0;
uint8_t fanSpeedPercent=100;

struct machinesettings {
  machinesettings() : has_saved_settings(0) {}
  int feedmultiply;
  int HotendTemperature[EXTRUDERS];
  int BedTemperature;
  uint8_t fanSpeed;
  int extrudemultiply[EXTRUDERS];
  long max_acceleration_units_per_sq_second[NUM_AXIS];
  float max_feedrate[NUM_AXIS];
  float acceleration;
  float minimumfeedrate;
  float mintravelfeedrate;
  long minsegmenttime;
  float max_xy_jerk;
  float max_z_jerk;
  float max_e_jerk;
  uint8_t has_saved_settings;
};
machinesettings machinesettings_tempsave[10];

#ifdef SERVO_ENDSTOPS
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif
#ifdef BARICUDA
int ValvePressure=0;
int EtoPPressure=0;
#endif

#ifdef FWRETRACT
  bool autoretract_enabled=false;
  bool retracted=false;
  float retract_length=4.5, retract_feedrate=25*60, retract_zlift=0.8;
#if EXTRUDERS > 1
  float extruder_swap_retract_length=16.0;
#endif
  float retract_recover_length=0, retract_recover_feedrate=25*60;
#endif

PRINT_STATE printing_state;

//===========================================================================
//=============================private variables=============================
//===========================================================================
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};
#ifdef DELTA
static float delta[3] = {0.0, 0.0, 0.0};
#endif
static float offset[3] = {0.0, 0.0, 0.0};
static bool home_all_axis = true;
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates

static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static bool fromsd[BUFSIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
//static int i = 0;
static char serial_char;
static int serial_count = 0;
static boolean comment_mode = false;
static char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

const int sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

//static float tt = 0;
//static float bt = 0;

//Inactivity shutdown variables
static unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;

unsigned long starttime=0;
unsigned long stoptime=0;

static uint8_t tmp_extruder;


uint8_t Stopped = false;

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

//===========================================================================
//=============================ROUTINES=============================
//===========================================================================

void get_arc_coordinates();
bool setTargetedHotend(int code);

/*void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
*/

extern "C"{
  extern unsigned int __bss_end;
  extern unsigned int __heap_start;
  extern void *__brkval;

  int freeMemory() {
    int free_memory;

    if((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);

    return free_memory;
  }
}

//Clear all the commands in the ASCII command buffer, to make sure we have room for abort commands.
void clear_command_queue()
{
    if (buflen > 0)
    {
        bufindw = (bufindr + 1)%BUFSIZE;
        buflen = 1;
    }
}

//adds an command to the main command buffer
//thats really done in a non-safe way.
//needs overworking someday
void enquecommand(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy(&(cmdbuffer[bufindw][0]),cmd);
    report_enqueing_command(cmdbuffer[bufindw]);
    bufindw = (bufindw + 1) % BUFSIZE;
    buflen += 1;
  }
}

void enquecommand_P(const char *cmd)
{
  if(buflen < BUFSIZE)
  {
    //this is dangerous if a mixing of serial and this happsens
    strcpy_P(&(cmdbuffer[bufindw][0]),cmd);
    report_enqueing_command(cmdbuffer[bufindw]);
    bufindw= (bufindw + 1)%BUFSIZE;
    buflen += 1;
  }
}

bool is_command_queued()
{
    return buflen > 0;
}

uint8_t commands_queued()
{
    return buflen;
}

void setup_killpin()
{
  #if defined(KILL_PIN) && KILL_PIN > -1
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN,HIGH);
  #endif
}

void setup_photpin()
{
  #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
    SET_OUTPUT(PHOTOGRAPH_PIN);
    WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if defined(PS_ON_PIN) && PS_ON_PIN > -1
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif
}

void suicide()
{
  #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
    SET_OUTPUT(SUICIDE_PIN);
    WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if (NUM_SERVOS >= 1) && defined(SERVO0_PIN) && (SERVO0_PIN > -1)
    servos[0].attach(SERVO0_PIN);
  #endif
  #if (NUM_SERVOS >= 2) && defined(SERVO1_PIN) && (SERVO1_PIN > -1)
    servos[1].attach(SERVO1_PIN);
  #endif
  #if (NUM_SERVOS >= 3) && defined(SERVO2_PIN) && (SERVO2_PIN > -1)
    servos[2].attach(SERVO2_PIN);
  #endif
  #if (NUM_SERVOS >= 4) && defined(SERVO3_PIN) && (SERVO3_PIN > -1)
    servos[3].attach(SERVO3_PIN);
  #endif
  #if (NUM_SERVOS >= 5)
    #error "TODO: enter initalisation code for more servos"
  #endif

  // Set position of Servo Endstops that are defined
  #ifdef SERVO_ENDSTOPS
  for(int8_t i = 0; i < 3; i++)
  {
    if(servo_endstops[i] > -1) {
      servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
    }
  }
  #endif
}


void setup()
{
  setup_killpin();
  setup_powerhold();
  MSerial.begin(BAUDRATE);
  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  report_startup_status(mcu);
  MCUSR=0;

  //Read ADC14, this is connected to the main power and helps in detecting which board we have
  // Connected to 24V - 100k -|- 4k7 - GND = ADC ~221 on Ultimaker 2.0 board with 8 microsteps on the Z
  // Connected to 24V - 100k -|- 10k - GND = ADC ~447 ADC on Ultimaker 2.x board with 16 microsteps on the Z
  int main_board_power = analogRead(14);

  report_firmware_information( freeMemory(), (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  for(int8_t i = 0; i < BUFSIZE; i++)
  {
    fromsd[i] = false;
  }

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();
  if (main_board_power > 300)//HACKERDYHACK, set the steps per unit for Z to 400 for the 16 microstep board.
    axis_steps_per_unit[to_index(Axes::Z)] = 400;
  lifetime_stats_init();
  i2cDriverInit();
  initFans();   // Initialize the fan driver
  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  servo_init();
  lcd_init();
#ifdef ENABLE_BED_LEVELING_PROBE
  i2cCapacitanceInit();
#endif
  fiset_init();
  
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif
}

void loop()
{
  if(fiset_data_ready()){
	  report_fiset_full_data( get_fiset_data_count(), get_fiset_gain(), get_fiset_magnitude(), get_fiset_data() );
  }

  if(buflen < (BUFSIZE-1))
    get_command();
  #ifdef SDSUPPORT
  card.checkautostart(false);
  #endif
  if(buflen)
  {
    #ifdef SDSUPPORT
      if(card.saving)
      {
        if(strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL)
        {
          card.write_command(cmdbuffer[bufindr]);
          if(card.logging)
          {
            process_commands();
          }
          else
          {
        	  report_ok();
          }
        }
        else
        {
          card.closefile();
          report_file_saved();
        }
      }
      else
      {
        process_commands();
      }
    #else
      process_commands();
    #endif //SDSUPPORT
    if (buflen > 0)
    {
      buflen = (buflen-1);
      bufindr = (bufindr + 1)%BUFSIZE;
    }
  }
  //check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  lcd_update();
  lifetime_stats_tick();
}

void get_command()
{
	while( MSerial.available() > 0  && buflen < BUFSIZE) {
		serial_char = MSerial.read();
		if(serial_char == '\n' ||
				serial_char == '\r' ||
				(serial_char == ':' && comment_mode == false) ||
				serial_count >= (MAX_CMD_SIZE - 1) )
		{
			if(!serial_count) { //if empty line
				comment_mode = false; //for new command
				return;
			}
			cmdbuffer[bufindw][serial_count] = 0; //terminate string
			if(!comment_mode){
				comment_mode = false; //for new command
				fromsd[bufindw] = false;
				if(strchr(cmdbuffer[bufindw], 'N') != NULL)
				{
					strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
					gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
					if(gcode_N != gcode_LastN+1 && (strstr_P(cmdbuffer[bufindw], PSTR("M110")) == NULL) ) {
						report_error_in_line_number(gcode_LastN);//Serial.println(gcode_N);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}

					if(strchr(cmdbuffer[bufindw], '*') != NULL)
					{
						byte checksum = 0;
						byte count = 0;
						while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
						strchr_pointer = strchr(cmdbuffer[bufindw], '*');

						if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
							report_error_in_checksum(gcode_LastN);
							FlushSerialRequestResend();
							serial_count = 0;
							return;
						}
						//if no errors, continue parsing
					}
					else
					{
						report_missing_checksum(gcode_LastN);
						FlushSerialRequestResend();
						serial_count = 0;
						return;
					}

					gcode_LastN = gcode_N;
					//if no errors, continue parsing
				}
				else  // if we don't receive 'N' but still see '*'
				{
					if((strchr(cmdbuffer[bufindw], '*') != NULL))
					{
						report_no_line_number_with_checksum(gcode_LastN);
						serial_count = 0;
						return;
					}
				}
				if((strchr(cmdbuffer[bufindw], 'G') != NULL)){
					strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
					switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
					case 0:
					case 1:
					case 2:
					case 3:
						if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
#ifdef SDSUPPORT
							if(card.saving)
								break;
#endif //SDSUPPORT
							report_ok();
						}
						else {
							report_printer_stopped();
							LCD_MESSAGEPGM(MSG_STOPPED);
						}
						break;
					default:
						break;
					}

				}
#ifdef ENABLE_ULTILCD2
				strchr_pointer = strchr(cmdbuffer[bufindw], 'M');
				if (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10) != 105)
					lastSerialCommandTime = millis();
#endif
				bufindw = (bufindw + 1)%BUFSIZE;
				buflen += 1;
			}
			serial_count = 0; //clear buffer
		}
		else
		{
			if(serial_char == ';') comment_mode = true;
			if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
		}
	}
#ifdef SDSUPPORT
	if(!card.sdprinting)
		return;
	if (serial_count!=0)
	{
		if (millis() - lastSerialCommandTime < 5000)
			return;
		serial_count = 0;
	}
	if (card.pause)
	{

		return;
	}
	static uint32_t endOfLineFilePosition = 0;
	while( !card.eof()  && buflen < BUFSIZE) {
		int16_t n=card.get();
		if (card.errorCode())
		{
			if (!card.sdInserted)
			{
				card.release();
				serial_count = 0;
				return;
			}

			//On an error, reset the error, reset the file position and try again.
			card.clearError();
			serial_count = 0;
			//Screw it, if we are near the end of a file with an error, act if the file is finished. Hopefully preventing the hang at the end.
			if (endOfLineFilePosition > card.getFileSize() - 512)
				card.sdprinting = false;
			else
				card.setIndex(endOfLineFilePosition);
			return;
		}

		serial_char = (char)n;
		if(serial_char == '\n' ||
				serial_char == '\r' ||
				(serial_char == ':' && comment_mode == false) ||
				serial_count >= (MAX_CMD_SIZE - 1)||n==-1)
		{
			if(card.eof() || n==-1){
				stoptime=millis();
				char time[30];
				unsigned long t=(stoptime-starttime)/1000;
				report_file_printed(t);
				int hours, minutes;
				minutes=(t/60)%60;
				hours=t/60/60;
				sprintf_P(time, PSTR("%i hours %i minutes"),hours, minutes);
				lcd_setstatus(time);
				card.printingHasFinished();
				card.checkautostart(true);

			}
			if(!serial_count)
			{
				comment_mode = false; //for new command
				return; //if empty line
			}
			cmdbuffer[bufindw][serial_count] = 0; //terminate string
			//      if(!comment_mode){
			fromsd[bufindw] = true;
			buflen += 1;
			bufindw = (bufindw + 1)%BUFSIZE;
			//      }
			comment_mode = false; //for new command
			serial_count = 0; //clear buffer
			endOfLineFilePosition = card.getFilePos();
		}
		else
		{
			if(serial_char == ';') comment_mode = true;
			if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
		}
	}

#endif //SDSUPPORT

}


float code_value()
{
  return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL));
}

long code_value_long()
{
  return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10));
}

bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(Axes axis)          \
    { return pgm_read_any(&array##_P[to_index(axis)]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_retract_mm, HOME_RETRACT_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

static void axis_is_at_home(Axes axis) {
  current_position[to_index(axis)] = base_home_pos(axis) + add_homeing[to_index(axis)];
  min_pos[to_index(axis)] =          base_min_pos(axis);// + add_homeing[axis];
  max_pos[to_index(axis)] =          base_max_pos(axis);// + add_homeing[axis];
}


static void homeaxis( Axes axis ) {
#define HOMEAXIS_DO(LETTER) \
  ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))
  if (axis==Axes::X ? HOMEAXIS_DO(X) :
      axis==Axes::Y ? HOMEAXIS_DO(Y) :
      axis==Axes::Z ? HOMEAXIS_DO(Z) :
      0) {

    // Engage Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
    #endif

    current_position[to_index(axis)] = 0;
    plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
    destination[to_index(axis)] = 1.5 * max_length(axis) * home_dir(axis);
    feedrate = homing_feedrate[to_index(axis)];
    plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
    st_synchronize();
    if (!isEndstopHit())
    {
        if (axis == Axes::Z)
        {
            current_position[to_index(axis)] = 0;
            plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
            destination[to_index(axis)] = -home_retract_mm(axis) * home_dir(axis) * 10.0;
            plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
            st_synchronize();

            report_endstop_not_pressed_after_homing();
            Stop(STOP_REASON_Z_ENDSTOP_BROKEN_ERROR);
        }else{
        	report_endstop_not_pressed_after_homing();
            Stop(STOP_REASON_XY_ENDSTOP_BROKEN_ERROR);
        }
        return;
    }

    current_position[to_index(axis)] = 0;
    plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
    destination[to_index(axis)] = -home_retract_mm(axis) * home_dir(axis);
    plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
    st_synchronize();

    bool endstop_pressed = false;
    switch(axis)
    {
    case Axes::X:
        #if defined(X_MIN_PIN) && X_MIN_PIN > -1 && X_HOME_DIR == -1
        endstop_pressed = (READ(X_MIN_PIN) != X_ENDSTOPS_INVERTING);
        #endif
        #if defined(X_MAX_PIN) && X_MAX_PIN > -1 && X_HOME_DIR == 1
        endstop_pressed = (READ(X_MAX_PIN) != X_ENDSTOPS_INVERTING);
        #endif
        break;
    case Axes::Y:
        #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1 && Y_HOME_DIR == -1
        endstop_pressed = (READ(Y_MIN_PIN) != Y_ENDSTOPS_INVERTING);
        #endif
        #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1 && Y_HOME_DIR == 1
        endstop_pressed = (READ(Y_MAX_PIN) != Y_ENDSTOPS_INVERTING);
        #endif
        break;
    case Axes::Z:
        #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1 && Z_HOME_DIR == -1
        endstop_pressed = (READ(Z_MIN_PIN) != Z_ENDSTOPS_INVERTING);
        #endif
        #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1 && Z_HOME_DIR == 1
        endstop_pressed = (READ(Z_MAX_PIN) != Z_ENDSTOPS_INVERTING);
        #endif
        break;
    }
    if (endstop_pressed)
    {
			report_error_endstop_still_pressed();
			if (axis == Axes::Z)
            Stop(STOP_REASON_Z_ENDSTOP_STUCK_ERROR);
        else
            Stop(STOP_REASON_XY_ENDSTOP_STUCK_ERROR);
        endstops_hit_on_purpose();
        return;
    }

    destination[to_index(axis)] = 2*home_retract_mm(axis) * home_dir(axis);
    feedrate = homing_feedrate[to_index(axis)]/3;
    plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
    st_synchronize();

    axis_is_at_home(axis);
    destination[to_index(axis)] = current_position[to_index(axis)];
    feedrate = 0.0;
    endstops_hit_on_purpose();

    // Retract Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
      if (servo_endstops[axis] > -1) servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
    #endif
  }
}
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

#ifdef ENABLE_BED_LEVELING_PROBE
float probeWithCapacitiveSensor()
{
    float total_z_height = 0.0;
    float z_target = 0.0;
    float z_distance = 5.0;

    feedrate = 1;
    for(uint8_t loop_counter = 0; loop_counter < CONFIG_BED_LEVEL_PROBE_REPEAT; loop_counter++)
    {
        destination[to_index(Axes::Z)] = z_target + z_distance;
        plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)], active_extruder);
        st_synchronize();
        destination[to_index(Axes::Z)] = z_target - z_distance;
        plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate, active_extruder);
        uint16_t cnt = 0;
        long sensor_value_total = 0;
        float z_position_total = 0.0;
        
        int16_t max_diff = 0;
        int16_t max_sensor_value = 0;
        int16_t previous_sensor_value = 0;
        uint8_t steady_counter = 0;

        float sensor_value_list[6];
        float z_position_list[6];
        for(uint8_t n=0; n<6; n++)
        {
            sensor_value_list[n] = 0;
            z_position_list[n] = 0;
        }

        i2cCapacitanceStart();
        while(blocks_queued())
        {
            manage_heater();
            manage_inactivity();
            //lcd_update(); //Not updating the LCD during this loop improves the performance of the i2cCapacitance sensor, allowing for a much higher sample right, as the display no longer keeps the i2c bus busy.
            lifetime_stats_tick();
            uint16_t value = 0;
            if (i2cCapacitanceDone(value))
            {
                z_position_total += float(st_get_position(Axes::Z))/axis_steps_per_unit[to_index(Axes::Z)];
                sensor_value_total += value;
                cnt++;
                if (cnt == 1000)
                {
                    z_position_total /= 1000;
                    sensor_value_total /= 1000;
                    
                    if (previous_sensor_value != 0)
                    {
                        for(uint8_t n=1; n<6; n++)
                        {
                            sensor_value_list[n - 1] = sensor_value_list[n];
                            z_position_list[n - 1] = z_position_list[n];
                        }
                        sensor_value_list[5] = sensor_value_total;
                        z_position_list[5] = z_position_total;

                        if (sensor_value_total > previous_sensor_value + max_diff / 3)
                        {
                            max_sensor_value = sensor_value_total;
                            steady_counter = 0;
                        }else{
                            steady_counter++;
                            if (steady_counter > 2)
                            {
                                quickStop();
                                current_position[to_index(Axes::X)] = destination[to_index(Axes::X)];
                                current_position[to_index(Axes::Y)] = destination[to_index(Axes::Y)];
                                current_position[to_index(Axes::Z)] = float(st_get_position(Axes::Z))/axis_steps_per_unit[to_index(Axes::Z)];
                                plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
                            }
                        }

                        int16_t diff = sensor_value_total - previous_sensor_value;
                        if (diff > max_diff)
                            max_diff = diff;
                    }
                    previous_sensor_value = sensor_value_total;
    /*
                    MSerial.print(z_position_total);
                    MSerial.print(' ');
                    MSerial.print(float(sensor_value_total) / 100.0f);
                    MSerial.print(' ');
                    MSerial.print(float(max_sensor_value) / 100.0f);
                    MSerial.print(' ');
                    MSerial.print(float(max_diff) / 100.0f);
                    MSerial.println();
    */
                    cnt = 0;
                    sensor_value_total = 0;
                    z_position_total = 0;
                }
                i2cCapacitanceStart();
            }
        }
    /*
        MSerial.println();
        for(uint8_t n=0; n<6; n++)
        {
            MSerial.print(z_position_list[n]);
            MSerial.print(' ');
            MSerial.print(float(sensor_value_list[n]) / 100.0f);
            MSerial.println();
        }
    */
        //Solve the line-line intersection to get the proper position where the bed was hit. At index [1] we are sure to be above the bed. At index[2] we are already on the bed. So the intersection point is somewhere between [1] and [2].
        float f = float((sensor_value_list[2]-(sensor_value_list[3]-sensor_value_list[2]))-sensor_value_list[1])/float((sensor_value_list[1]-sensor_value_list[0])-(sensor_value_list[3]-sensor_value_list[2]));
        float z_height = z_position_list[1] + (z_position_list[2] - z_position_list[1]) * f;
        z_target = z_height;
        z_distance = 1.0;
        total_z_height += z_height;
    }
    return total_z_height / CONFIG_BED_LEVEL_PROBE_REPEAT;
}
#endif//ENABLE_BED_LEVELING_PROBE


void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  printing_state = PRINT_STATE::NORMAL;
  if(code_seen('G'))
  {
    switch((int)code_value())
    {
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {
        get_coordinates(); // For X Y Z E F
        prepare_move();
        //ClearToSend();
        return;
      }
      //break;
    case 2: // G2  - CW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(true);
        return;
      }
    case 3: // G3  - CCW ARC
      if(Stopped == false) {
        get_arc_coordinates();
        prepare_arc_move(false);
        return;
      }
    case 4: // G4 dwell
      LCD_MESSAGEPGM(MSG_DWELL);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd = millis();
      printing_state = PRINT_STATE::DWELL;
      while(millis()  < codenum ){
        manage_heater();
        manage_inactivity();
        lcd_update();
        lifetime_stats_tick();
      }
      break;
      #ifdef FWRETRACT
      case 10: // G10 retract
      if(!retracted)
      {
        destination[to_index(Axes::X)]=current_position[to_index(Axes::X)];
        destination[to_index(Axes::Y)]=current_position[to_index(Axes::Y)];
        destination[to_index(Axes::Z)]=current_position[to_index(Axes::Z)];
        #if EXTRUDERS > 1
        if (code_seen('S') && code_value_long() == 1)
            destination[to_index(Axes::E)]=current_position[to_index(Axes::E)]-extruder_swap_retract_length/volume_to_filament_length[active_extruder];
        else
            destination[to_index(Axes::E)]=current_position[to_index(Axes::E)]-retract_length/volume_to_filament_length[active_extruder];
        #else
        destination[to_index(Axes::E)]=current_position[to_index(Axes::E)]-retract_length/volume_to_filament_length[active_extruder];
        #endif
        float oldFeedrate = feedrate;
        feedrate=retract_feedrate;
        retract_recover_length = current_position[to_index(Axes::E)]-destination[to_index(Axes::E)];//Set the recover length to whatever distance we retracted so we recover properly.
        retracted=true;
        prepare_move();
        feedrate = oldFeedrate;
      }

      break;
      case 11: // G11 retract_recover
      if(retracted)
      {
        destination[to_index(Axes::X)]=current_position[to_index(Axes::X)];
        destination[to_index(Axes::Y)]=current_position[to_index(Axes::Y)];
        destination[to_index(Axes::Z)]=current_position[to_index(Axes::Z)];
        destination[to_index(Axes::E)]=current_position[to_index(Axes::E)]+retract_recover_length;
        float oldFeedrate = feedrate;
        feedrate=retract_recover_feedrate;
        retracted=false;
        prepare_move();
        feedrate = oldFeedrate;
      }
      break;
      #endif //FWRETRACT
    case 28: //G28 Home all Axis one at a time
      printing_state = PRINT_STATE::HOMING;
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      previous_millis_cmd = millis();

      enable_endstops(true);

      for(int8_t i=0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i];
      }
          feedrate = 0.0;

#ifdef DELTA
          // A delta can only safely home all axis at the same time
          // all axis have to home at the same time

          // Move all carriages up together until the first endstop is hit.
          current_position[to_index(Axes::X)] = 0;
          current_position[to_index(Axes::Y)] = 0;
          current_position[to_index(Axes::Z)] = 0;
          plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);

          destination[to_index(Axes::X)] = 3 * Z_MAX_LENGTH;
          destination[to_index(Axes::Y)] = 3 * Z_MAX_LENGTH;
          destination[to_index(Axes::Z)] = 3 * Z_MAX_LENGTH;
          feedrate = 1.732 * homing_feedrate[to_index(Axes::X)];
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
          st_synchronize();
          endstops_hit_on_purpose();

          current_position[to_index(Axes::X)] = destination[to_index(Axes::X)];
          current_position[to_index(Axes::Y)] = destination[to_index(Axes::Y)];
          current_position[to_index(Axes::Z)] = destination[to_index(Axes::Z)];

          // take care of back off and rehome now we are all at the top
          HOMEAXIS(X);
          HOMEAXIS(Y);
          HOMEAXIS(Z);

          calculate_delta(current_position);
          plan_set_position(delta[to_index(Axes::X)], delta[to_index(Axes::Y)], delta[to_index(Axes::Z)], current_position[to_index(Axes::E)]);

#else // NOT DELTA

          home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      #if defined(QUICK_HOME)
      if(home_all_axis)
      {
        current_position[to_index(Axes::X)] = 0; current_position[to_index(Axes::Y)] = 0; current_position[to_index(Axes::Z)] = 0;

        plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);

        destination[to_index(Axes::X)] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;
        destination[to_index(Axes::Y)] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
        destination[to_index(Axes::Z)] = 1.5 * Z_MAX_LENGTH * Z_HOME_DIR;
        feedrate = homing_feedrate[to_index(Axes::X)];
        plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
        st_synchronize();
        endstops_hit_on_purpose();

        axis_is_at_home(Axes::X);
        axis_is_at_home(Axes::Y);
        axis_is_at_home(Axes::Z);
        plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
        destination[to_index(Axes::X)] = current_position[to_index(Axes::X)];
        destination[to_index(Axes::Y)] = current_position[to_index(Axes::Y)];
        destination[to_index(Axes::Z)] = current_position[to_index(Axes::Z)];
        plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();

        current_position[to_index(Axes::X)] = destination[to_index(Axes::X)];
        current_position[to_index(Axes::Y)] = destination[to_index(Axes::Y)];
        current_position[to_index(Axes::Z)] = destination[to_index(Axes::Z)];
      }
      #endif
      if((home_all_axis) || (code_seen(axis_codes[to_index(Axes::Z)]))) {
        homeaxis(Axes::Z);
      }
      #endif

      #if defined(QUICK_HOME)
      if((home_all_axis)||( code_seen(axis_codes[to_index(Axes::X)]) && code_seen(axis_codes[to_index(Axes::Y)])) )  //first diagonal move
      {
        current_position[to_index(Axes::X)] = 0;current_position[to_index(Axes::Y)] = 0;

        plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
        destination[to_index(Axes::X)] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;destination[to_index(Axes::Y)] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
        feedrate = homing_feedrate[to_index(Axes::X)];
        if(homing_feedrate[to_index(Axes::Y)]<feedrate)
          feedrate =homing_feedrate[to_index(Axes::Y)];
        plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
        st_synchronize();

        axis_is_at_home(Axes::X);
        axis_is_at_home(Axes::Y);
        plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
        destination[to_index(Axes::X)] = current_position[to_index(Axes::X)];
        destination[to_index(Axes::Y)] = current_position[to_index(Axes::Y)];
        plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();

        current_position[to_index(Axes::X)] = destination[to_index(Axes::X)];
        current_position[to_index(Axes::Y)] = destination[to_index(Axes::Y)];
        current_position[to_index(Axes::Z)] = destination[to_index(Axes::Z)];
      }
      #endif

      if((home_all_axis) || (code_seen(axis_codes[to_index(Axes::X)])))
      {
        homeaxis(Axes::X);
      }

      if((home_all_axis) || (code_seen(axis_codes[to_index(Axes::Y)]))) {
        homeaxis(Axes::Y);
      }

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
      if((home_all_axis) || (code_seen(axis_codes[to_index(Axes::Z)]))) {
        HOMEAXIS(Z);
      }
      #endif

      if(code_seen(axis_codes[to_index(Axes::X)]))
      {
        if(code_value_long() != 0) {
          current_position[to_index(Axes::X)]=code_value()+add_homeing[0];
        }
      }

      if(code_seen(axis_codes[to_index(Axes::Y)])) {
        if(code_value_long() != 0) {
          current_position[to_index(Axes::Y)]=code_value()+add_homeing[1];
        }
      }

      if(code_seen(axis_codes[to_index(Axes::Z)])) {
        if(code_value_long() != 0) {
          current_position[to_index(Axes::Z)]=code_value()+add_homeing[2];
        }
      }
      plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
#endif // DELTA

      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif

      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      previous_millis_cmd = millis();
      endstops_hit_on_purpose();
      break;
#ifdef ENABLE_BED_LEVELING_PROBE
    case 29://G29 - Automatic bed leveling with probing.
      {
          planner_bed_leveling_factor[to_index(Axes::X)] = 0.0;
          planner_bed_leveling_factor[to_index(Axes::Y)] = 0.0;
          
          destination[to_index(Axes::Z)] = CONFIG_BED_LEVELING_Z_HEIGHT;
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)], active_extruder);
          destination[to_index(Axes::X)] = CONFIG_BED_LEVELING_POINT1_X;
          destination[to_index(Axes::Y)] = CONFIG_BED_LEVELING_POINT1_Y;
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::X)], active_extruder);
          st_synchronize();
          float height_1 = probeWithCapacitiveSensor();

          destination[to_index(Axes::Z)] = CONFIG_BED_LEVELING_Z_HEIGHT;
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)], active_extruder);
          destination[to_index(Axes::X)] = CONFIG_BED_LEVELING_POINT2_X;
          destination[to_index(Axes::Y)] = CONFIG_BED_LEVELING_POINT2_Y;
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::X)], active_extruder);
          st_synchronize();
          float height_2 = probeWithCapacitiveSensor();

          destination[to_index(Axes::Z)] = CONFIG_BED_LEVELING_Z_HEIGHT;
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)], active_extruder);
          destination[to_index(Axes::X)] = CONFIG_BED_LEVELING_POINT3_X;
          destination[to_index(Axes::Y)] = CONFIG_BED_LEVELING_POINT3_Y;
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::X)], active_extruder);
          st_synchronize();
          float height_3 = probeWithCapacitiveSensor();
          destination[to_index(Axes::Z)] = height_3;
          //Position the head at exactly the height, so we can use this as Z0 later.
          plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::X)], active_extruder);
          st_synchronize();

          
          //Set the X and Y skew factors for how skewed the bed is (this assumes the leveling points are 1 in the back, and 2 at the front)
          planner_bed_leveling_factor[to_index(Axes::X)] = (height_3 - height_2) / (CONFIG_BED_LEVELING_POINT3_X - CONFIG_BED_LEVELING_POINT2_X);
          planner_bed_leveling_factor[to_index(Axes::Y)] = ((height_2 + height_3) / 2.0 - height_1) / (CONFIG_BED_LEVELING_POINT3_Y - CONFIG_BED_LEVELING_POINT1_Y);
          
			report_bed_leveling_probe_sequence(height_1, height_2, height_3,
					planner_bed_leveling_factor[to_index(Axes::X)],
					planner_bed_leveling_factor[to_index(Axes::Y)]);
          //Correct the Z position. So Z0 is always on top of the bed. We are currently positioned at point 3, on top of the bed.
          destination[to_index(Axes::Z)] = 0.0;
          plan_set_position(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)]);
      }
      break;
    case 30: // G30 Probe Z at current position and report result.
      destination[to_index(Axes::Z)] = probeWithCapacitiveSensor();
      plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)], active_extruder);
      report_bed_leveling_probe_point(destination[to_index(Axes::Z)]);
      break;
#endif
    case 90: // G90
      relative_mode = false;
      break;
    case 91: // G91
      relative_mode = true;
      break;
    case 92: // G92
      if(!code_seen(axis_codes[to_index(Axes::E)]))
        st_synchronize();
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) {
           if(i == to_index(Axes::E)) {
             current_position[i] = code_value();
             plan_set_e_position(current_position[to_index(Axes::E)]);
           }
           else {
             current_position[i] = code_value();
             plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
           }
        }
      }
      break;
    }
  }

  else if(code_seen('M'))
  {
    switch( (int)code_value() )
    {
#ifdef ULTIPANEL
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
      printing_state = PRINT_STATE::WAIT_USER;
      LCD_MESSAGEPGM(MSG_USERWAIT);
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait

      st_synchronize();
      previous_millis_cmd = millis();
      if (codenum > 0){
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum && !lcd_clicked()){
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
      }else{
        while(!lcd_clicked()){
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
      }
      LCD_MESSAGEPGM(MSG_RESUMING);
    }
    break;
#endif
#ifdef ENABLE_ULTILCD2
    case 0: // M0 - Unconditional stop - Wait for user button press on LCD
    case 1: // M1 - Conditional stop - Wait for user button press on LCD
    {
        card.pause = true;
        while(card.pause)
        {
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
    }
    break;
#endif
    case 17:
        LCD_MESSAGEPGM(MSG_NO_MOVE);
        enable_x();
        enable_y();
        enable_z();
        enable_e0();
        enable_e1();
        enable_e2();
      break;

#ifdef SDSUPPORT
    case 20: // M20 - list SD card
    	serial_sd_card_begin_file_list();
      card.ls();
      serial_sd_card_end_file_list();
      break;
    case 21: // M21 - init SD card

      card.initsd();

      break;
    case 22: //M22 - release SD card
      card.release();

      break;
    case 23: //M23 - Select file
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      break;
    case 24: //M24 - Start SD print
      card.startFileprint();
      starttime=millis();
      break;
    case 25: //M25 - Pause SD print
      //card.pauseSDPrint();
      card.closefile();
      break;
    case 26: //M26 - Set SD index
      if(card.isOk() && code_seen('S')) {
        card.setIndex(code_value_long());
      }
      break;
    case 27: //M27 - Get SD status
      card.getStatus();
      break;
    case 28: //M28 - Start SD write
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openFile(strchr_pointer+4,false);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //card,saving = false;
      break;
    case 30: //M30 <filename> Delete File
      if (card.isOk()){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(cmdbuffer[bufindr], 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos-1) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;
    case 923: //M923 - Select file and start printing
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      card.openFile(strchr_pointer + 4,true);
      card.startFileprint();
      starttime=millis();
      break;
    case 928: //M928 - Start SD write
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos-1) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    case 31: //M31 take time since the start of the SD print or an M109 command
    {
    	stoptime=millis();
    	char time[30];
    	unsigned long t=(stoptime-starttime)/1000;
    	report_time_elapsed(t);
    	int sec,min;
    	min=t/60;
    	sec=t%60;
    	sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
    	lcd_setstatus(time);
    	autotempShutdown();
    }
    break;
    case 42: //M42 -Change pin status via gcode
      if (code_seen('S'))
      {
        int pin_status = code_value();
        int pin_number = LED_PIN;
        if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          pin_number = code_value();
        for(int8_t i = 0; i < (int8_t)sizeof(sensitive_pins); i++)
        {
          if (sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          pinMode(pin_number, OUTPUT);
          digitalWrite(pin_number, pin_status);
          analogWrite(pin_number, pin_status);
        }
      }
     break;
    case 104: // M104
      if(setTargetedHotend(104)){
        break;
      }
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      setWatch();
      break;
    case 140: // M140 set bed temp
      if (code_seen('S')) setTargetBed(code_value());
      break;
    case 105 : // M105
    	if(setTargetedHotend(105)){
    		break;
    	}
    	{
#if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
    	const float deg_hotend = degHotend(tmp_extruder);
    	const float target_hotend = degTargetHotend(tmp_extruder);
#else
    	const float deg_hotend = 0;
    	const float target_hotend = 0;
#endif // TEMP_0_PIN
#if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
    	const float deg_bed = degBed();
    	const float target_bed = degTargetBed();
#else
    	const float deg_bed = 0;
    	const float target_bed = 0;
#endif // TEMP_BED_PIN
    	report_temperature_and_power(
    			tmp_extruder,
				deg_hotend,
				target_hotend,
				deg_bed,
				target_bed,
				getHeaterPower(tmp_extruder),
				getHeaterPower(-1));
    	}
    	return;
    	break;
    case 109:
    {// M109 - Wait for extruder heater to reach target.
      if(setTargetedHotend(109)){
        break;
      }
      printing_state = PRINT_STATE::HEATING;
      LCD_MESSAGEPGM(MSG_HEATING);
      #ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
      if (code_seen('S')) setTargetHotend(code_value(), tmp_extruder);
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor=code_value();
          autotemp_enabled=true;
        }
      #endif

      setWatch();
      codenum = millis();

      /* See if we are heating up or cooling down */
      bool target_direction = isHeatingHotend(tmp_extruder); // true if heating, false if cooling

      #ifdef TEMP_RESIDENCY_TIME
        long residencyStart;
        residencyStart = -1;
        /* continue to loop until we have reached the target temp
          _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
        while((residencyStart == -1) ||
              (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL))) ) {
      #else
        while ( target_direction ? (isHeatingHotend(tmp_extruder)) : (isCoolingHotend(tmp_extruder)&&(CooldownNoWait==false)) ) {
      #endif //TEMP_RESIDENCY_TIME
          if( (millis() - codenum) > 1000UL )
          { //Print Temp Reading and remaining time every 1 second while heating up/cooling down
#ifdef TEMP_RESIDENCY_TIME
        	  const char residency_char = residencyStart > -1 ? 'W': '?';
        	  const int16_t residency_time = residency_char == 'W' ? ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL : 0;
#else
        	  const char residency_char = 0;
        	  const int16_t residency_time = 0;
#endif
        	  report_wait_for_temperature( tmp_extruder, degHotend(tmp_extruder), residency_char, residency_time );
        	  codenum = millis();
          }
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        #ifdef TEMP_RESIDENCY_TIME
            /* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
              or when current temp falls outside the hysteresis after target temp was reached */
          if ((residencyStart == -1 &&  target_direction && (degHotend(tmp_extruder) >= (degTargetHotend(tmp_extruder)-TEMP_WINDOW))) ||
              (residencyStart == -1 && !target_direction && (degHotend(tmp_extruder) <= (degTargetHotend(tmp_extruder)+TEMP_WINDOW))) ||
              (residencyStart > -1 && labs(degHotend(tmp_extruder) - degTargetHotend(tmp_extruder)) > TEMP_HYSTERESIS && (!target_direction || !CooldownNoWait)) )
          {
            residencyStart = millis();
          }
        #endif //TEMP_RESIDENCY_TIME
        }
        LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
        starttime=millis();
        previous_millis_cmd = millis();
      }
      break;
    case 190: // M190 - Wait for bed heater to reach target.
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        printing_state = PRINT_STATE::HEATING_BED;
        LCD_MESSAGEPGM(MSG_BED_HEATING);
        if (code_seen('S')) setTargetBed(code_value());
        codenum = millis();
        while(current_temperature_bed < target_temperature_bed - TEMP_WINDOW)
        {
          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            report_wait_for_temperature_bed(active_extruder, degHotend(active_extruder), degBed() );
            codenum = millis();
          }
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }
        LCD_MESSAGEPGM(MSG_BED_DONE);
        previous_millis_cmd = millis();
    #endif // defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        break;

    #if defined(FAN_PIN) && FAN_PIN > -1
      case 106: //M106 Fan On
        if (code_seen('S')){
           fanSpeed=constrain(code_value() * fanSpeedPercent / 100,0,255);
        }
        else {
          fanSpeed = 255 * int(fanSpeedPercent) / 100;
        }
        break;
      case 107: //M107 Fan Off
        fanSpeed = 0;
        break;
    #endif //FAN_PIN
    #ifdef BARICUDA
      // PWM for HEATER_1_PIN
      #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
        case 126: //M126 valve open
          if (code_seen('S')){
             ValvePressure=constrain(code_value(),0,255);
          }
          else {
            ValvePressure=255;
          }
          break;
        case 127: //M127 valve closed
          ValvePressure = 0;
          break;
      #endif //HEATER_1_PIN

      // PWM for HEATER_2_PIN
      #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
        case 128: //M128 valve open
          if (code_seen('S')){
             EtoPPressure=constrain(code_value(),0,255);
          }
          else {
            EtoPPressure=255;
          }
          break;
        case 129: //M129 valve closed
          EtoPPressure = 0;
          break;
      #endif //HEATER_2_PIN
    #endif

    #if defined(PS_ON_PIN) && PS_ON_PIN > -1
      case 80: // M80 - ATX Power On
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);
        break;
      #endif

      case 81: // M81 - ATX Power Off

      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
        break;

    case 82:
      axis_relative_modes[3] = false;
      break;
    case 83:
      axis_relative_modes[3] = true;
      break;
    case 18: //compatibility
    case 84: // M84
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2]))|| (code_seen(axis_codes[3])));
        if(all_axis)
        {
          st_synchronize();
          disable_e0();
          disable_e1();
          disable_e2();
          finishAndDisableSteppers();
        }
        else
        {
          st_synchronize();
          if(code_seen('X')) disable_x();
          if(code_seen('Y')) disable_y();
          if(code_seen('Z')) disable_z();
          #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
            if(code_seen('E')) {
              disable_e0();
              disable_e1();
              disable_e2();
            }
          #endif
        }
      }
      break;
    case 85: // M85
      if (code_seen('S')) max_inactive_time = code_value() * 1000;
      break;
    case 92: // M92
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == 3) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              max_e_jerk *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            axis_steps_per_unit[i] = value;
          }
          else {
            axis_steps_per_unit[i] = code_value();
          }
        }
      }
      break;
    case 115: // M115
      report_firmware_capabilities_string( MSG_FIRMWARE_CAPABILITIES );
      break;
    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos-1)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;
    case 114: // M114
    	report_current_position(
    			current_position[to_index(Axes::X)],
				current_position[to_index(Axes::Y)],
				current_position[to_index(Axes::Z)],
				current_position[to_index(Axes::E)],
				float(st_get_position(Axes::X))/axis_steps_per_unit[to_index(Axes::X)],
				float(st_get_position(Axes::Y))/axis_steps_per_unit[to_index(Axes::Y)],
				float(st_get_position(Axes::Z))/axis_steps_per_unit[to_index(Axes::Z)]);
    	break;
    case 120: // M120
      enable_endstops(false) ;
      break;
    case 121: // M121
      enable_endstops(true) ;
      break;
    case 119: // M119
    {

#if defined(X_MIN_PIN) && X_MIN_PIN > -1
    	endstop_status endstop_X_min = (READ(X_MIN_PIN)^X_ENDSTOPS_INVERTING)?endstop_status::TRIGGERED:endstop_status::UNTRIGGERD;
#else
    	endstop_status endstop_X_min = endstop_status::UNUSED;
#endif

#if defined(X_MAX_PIN) && X_MAX_PIN > -1
    	endstop_status endstop_X_max = (READ(X_MAX_PIN)^X_ENDSTOPS_INVERTING)?endstop_status::TRIGGERED:endstop_status::UNTRIGGERD;
#else
    	endstop_status endstop_X_max = endstop_status::UNUSED;
#endif

#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    	endstop_status endstop_Y_min =  (READ(Y_MIN_PIN)^Y_ENDSTOPS_INVERTING)?endstop_status::TRIGGERED:endstop_status::UNTRIGGERD;
#else
    	endstop_status endstop_Y_min =  endstop_status::UNUSED;
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
		endstop_status endstop_Y_max =  (READ(Y_MAX_PIN)^Y_ENDSTOPS_INVERTING) ? endstop_status::TRIGGERED:endstop_status::UNTRIGGERD;
#else
		endstop_status endstop_Y_max =  endstop_status::UNUSED;
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
		endstop_status endstop_Z_min =  (READ(Z_MIN_PIN)^Z_ENDSTOPS_INVERTING) ? endstop_status::TRIGGERED:endstop_status::UNTRIGGERD;
#else
		endstop_status endstop_Z_min =  endstop_status::UNUSED;
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
		endstop_status endstop_Z_max =  (READ(Z_MAX_PIN)^Z_ENDSTOPS_INVERTING) ? endstop_status::TRIGGERED:endstop_status::UNTRIGGERD;
#else
		endstop_status endstop_Z_max =  endstop_status::UNUSED;
#endif
    	report_endstop_status( endstop_X_min,endstop_X_max,endstop_Y_min,endstop_Y_max, endstop_Z_min, endstop_Z_max);
    }
    break;
      //TODO: update for all axis, use for loop
    case 201: // M201
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          max_acceleration_units_per_sq_second[i] = code_value();
        }
      }
      // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
      reset_acceleration_rates();
      break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
      }
      break;
    #endif
    case 203: // M203 max feedrate mm/sec
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
      }
      break;
    case 204: // M204 acceleration: S - normal moves;  T - filament only moves
      {
        if(code_seen('S')) acceleration = code_value() ;
        if(code_seen('T')) retract_acceleration = code_value() ;
      }
      break;
    case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
    {
      if(code_seen('S')) minimumfeedrate = code_value();
      if(code_seen('T')) mintravelfeedrate = code_value();
      if(code_seen('B')) minsegmenttime = code_value() ;
      if(code_seen('X')) max_xy_jerk = code_value() ;
      if(code_seen('Z')) max_z_jerk = code_value() ;
      if(code_seen('E')) max_e_jerk = code_value() ;
    }
    break;
    case 206: // M206 additional homing offset
      for(int8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
      }
      break;
    #ifdef FWRETRACT
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        retract_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_feedrate = code_value() ;
      }
      if(code_seen('Z'))
      {
        retract_zlift = code_value() ;
      }
    }break;
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
    {
      if(code_seen('S'))
      {
        retract_recover_length = code_value() ;
      }
      if(code_seen('F'))
      {
        retract_recover_feedrate = code_value() ;
      }
    }break;
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        int t= code_value() ;
        switch(t)
        {
          case 0: autoretract_enabled=false;retracted=false;break;
          case 1: autoretract_enabled=true;retracted=false;break;
          default:
        	  report_unknown_command( cmdbuffer[bufindr] );
        }
      }

    }break;
    #endif // FWRETRACT
    #if EXTRUDERS > 1
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      if(setTargetedHotend(218)){
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[to_index(Axes::X)][tmp_extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[to_index(Axes::Y)][tmp_extruder] = code_value();
      }
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
      for(tmp_extruder = 0; tmp_extruder < EXTRUDERS; tmp_extruder++)
      {
         SERIAL_ECHO(" ");
         SERIAL_ECHO(extruder_offset[to_index(Axes::X)][tmp_extruder]);
         SERIAL_ECHO(",");
         SERIAL_ECHO(extruder_offset[to_index(Axes::Y)][tmp_extruder]);
      }
      SERIAL_ECHOLN("");
    }break;
    #endif
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
      if(code_seen('S'))
      {
        feedmultiply = code_value() ;
      }
    }
    break;
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
      if(code_seen('S'))
      {
        extrudemultiply[active_extruder] = code_value() ;
      }
    }
    break;

    #if NUM_SERVOS > 0
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
            servos[servo_index].write(servo_position);
          }
          else {
            SERIAL_ECHO_START;
            SERIAL_ECHO("Servo ");
            SERIAL_ECHO(servo_index);
            SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          SERIAL_PROTOCOL(MSG_OK);
          SERIAL_PROTOCOL(" Servo ");
          SERIAL_PROTOCOL(servo_index);
          SERIAL_PROTOCOL(": ");
          SERIAL_PROTOCOL(servos[servo_index].read());
          SERIAL_PROTOCOLLN("");
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    #if LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) )
    case 300: // M300
    {
      int beepS = code_seen('S') ? code_value() : 110;
      int beepP = code_seen('P') ? code_value() : 1000;
      if (beepS > 0)
      {
        #if BEEPER > 0
          tone(BEEPER, beepS);
          delay(beepP);
          noTone(BEEPER);
        #elif defined(ULTRALCD)
          lcd_buzz(beepS, beepP);
        #endif
      }
      else
      {
        delay(beepP);
      }
    }
    break;
    #endif // M300

    #ifdef PIDTEMP
    case 301: // M301
      {
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = scalePID_i(code_value());
        if(code_seen('D')) Kd = scalePID_d(code_value());

        #ifdef PID_ADD_EXTRUSION_RATE
        if(code_seen('C')) Kc = code_value();
        #endif

        updatePID();
#ifdef PID_ADD_EXTRUSION_RATE
        float kc = Kc;
#else
        float kc = 0;
#endif
        report_PID_extruder_parameters(Kp, unscalePID_i(Ki), unscalePID_d(Kd), kc);
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED
    case 304: // M304
      {
        if(code_seen('P')) bedKp = code_value();
        if(code_seen('I')) bedKi = scalePID_i(code_value());
        if(code_seen('D')) bedKd = scalePID_d(code_value());

        updatePID();
        SERIAL_PROTOCOL(MSG_OK);
        SERIAL_PROTOCOL(" p:");
        SERIAL_PROTOCOL(bedKp);
        SERIAL_PROTOCOL(" i:");
        SERIAL_PROTOCOL(unscalePID_i(bedKi));
        SERIAL_PROTOCOL(" d:");
        SERIAL_PROTOCOL(unscalePID_d(bedKd));
        SERIAL_PROTOCOLLN("");
      }
      break;
    #endif //PIDTEMP
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
      #if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
        const uint8_t NUM_PULSES=16;
        const float PULSE_LENGTH=0.01524;
        for(int i=0; i < NUM_PULSES; i++) {
          WRITE(PHOTOGRAPH_PIN, HIGH);
          _delay_ms(PULSE_LENGTH);
          WRITE(PHOTOGRAPH_PIN, LOW);
          _delay_ms(PULSE_LENGTH);
        }
        delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
          WRITE(PHOTOGRAPH_PIN, HIGH);
          _delay_ms(PULSE_LENGTH);
          WRITE(PHOTOGRAPH_PIN, LOW);
          _delay_ms(PULSE_LENGTH);
        }
      #endif
     }
    break;
    #ifdef PREVENT_DANGEROUS_EXTRUDE
    case 302: // allow cold extrudes, or set the minimum extrude temperature
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      set_extrude_min_temp(temp);
    }
    break;
	#endif
    case 303: // M303 PID autotune
    {
      float temp = 150.0;
      int e=0;
      int c=5;
      if (code_seen('E')) e=code_value();
        if (e<0)
          temp=70;
      if (code_seen('S')) temp=code_value();
      if (code_seen('C')) c=code_value();
      PID_autotune(temp, e, c);
    }
    break;
#ifdef ENABLE_BED_LEVELING_PROBE
    case 310://M310, single cap sensor read.
      {
        i2cCapacitanceStart();
        uint16_t value;
        while(!i2cCapacitanceDone(value))
        {
            manage_heater();
            manage_inactivity();
            lcd_update();
            lifetime_stats_tick();
        }
        report_single_cap_probe_reading(value);
      }
    break;
#endif//ENABLE_BED_LEVELING_PROBE
    case 400: // M400 finish all moves
    {
      st_synchronize();
    }
    break;
    case 500: // M500 Store settings in EEPROM
    {
        Config_StoreSettings();
    }
    break;
    case 501: // M501 Read settings from EEPROM
    {
        Config_RetrieveSettings();
    }
    break;
    case 502: // M502 Revert to default settings
    {
        Config_ResetDefault();
    }
    break;
    case 503: // M503 print settings currently in memory
    {
        Config_PrintSettings();
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif
    #ifdef FILAMENTCHANGEENABLE
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
    {
        float target[4];
        float lastpos[4];
        target[to_index(Axes::X)]=current_position[to_index(Axes::X)];
        target[to_index(Axes::Y)]=current_position[to_index(Axes::Y)];
        target[to_index(Axes::Z)]=current_position[to_index(Axes::Z)];
        target[to_index(Axes::E)]=current_position[to_index(Axes::E)];
        lastpos[to_index(Axes::X)]=current_position[to_index(Axes::X)];
        lastpos[to_index(Axes::Y)]=current_position[to_index(Axes::Y)];
        lastpos[to_index(Axes::Z)]=current_position[to_index(Axes::Z)];
        lastpos[to_index(Axes::E)]=current_position[to_index(Axes::E)];
        //retract by E
        if(code_seen('E'))
        {
          target[to_index(Axes::E)]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FIRSTRETRACT
            target[to_index(Axes::E)]+= FILAMENTCHANGE_FIRSTRETRACT ;
          #endif
        }
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder);

        //lift Z
        if(code_seen('Z'))
        {
          target[to_index(Axes::Z)]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_ZADD
            target[to_index(Axes::Z)]+= FILAMENTCHANGE_ZADD ;
          #endif
        }
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder);

        //move xy
        if(code_seen('X'))
        {
          target[to_index(Axes::X)]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
            target[to_index(Axes::X)]= FILAMENTCHANGE_XPOS ;
          #endif
        }
        if(code_seen('Y'))
        {
          target[to_index(Axes::Y)]= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            target[to_index(Axes::Y)]= FILAMENTCHANGE_YPOS ;
          #endif
        }

        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder);

        if(code_seen('L'))
        {
          target[to_index(Axes::E)]+= code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[to_index(Axes::E)]+= FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }

        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder);

        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        delay(100);
        LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
        uint8_t cnt=0;
        while(!lcd_clicked()){
          cnt++;
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
          if(cnt==0)
          {
          #if BEEPER > 0
            SET_OUTPUT(BEEPER);

            WRITE(BEEPER,HIGH);
            delay(3);
            WRITE(BEEPER,LOW);
            delay(3);
          #else
            lcd_buzz(1000/6,100);
          #endif
          }
        }

        //return to normal
        if(code_seen('L'))
        {
          target[to_index(Axes::E)]+= -code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FINALRETRACT
            target[to_index(Axes::E)]+=(-1)*FILAMENTCHANGE_FINALRETRACT ;
          #endif
        }
        current_position[to_index(Axes::E)]=target[to_index(Axes::E)]; //the long retract of L is compensated by manual filament feeding
        plan_set_e_position(current_position[to_index(Axes::E)]);
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder); //should do nothing
        plan_buffer_line(lastpos[to_index(Axes::X)], lastpos[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder); //move xy back
        plan_buffer_line(lastpos[to_index(Axes::X)], lastpos[to_index(Axes::Y)], lastpos[to_index(Axes::Z)], target[to_index(Axes::E)], feedrate/60, active_extruder); //move z back
        plan_buffer_line(lastpos[to_index(Axes::X)], lastpos[to_index(Axes::Y)], lastpos[to_index(Axes::Z)], lastpos[to_index(Axes::E)], feedrate/60, active_extruder); //final untretract
    }
    break;
    #endif //FILAMENTCHANGEENABLE
    #ifdef ENABLE_ULTILCD2
    case 601: //Pause in UltiLCD2, X[pos] Y[pos] Z[relative lift] L[later retract distance]
    {
        st_synchronize();
        float target[4];
        float lastpos[4];
        target[to_index(Axes::X)]=current_position[to_index(Axes::X)];
        target[to_index(Axes::Y)]=current_position[to_index(Axes::Y)];
        target[to_index(Axes::Z)]=current_position[to_index(Axes::Z)];
        target[to_index(Axes::E)]=current_position[to_index(Axes::E)];
        lastpos[to_index(Axes::X)]=current_position[to_index(Axes::X)];
        lastpos[to_index(Axes::Y)]=current_position[to_index(Axes::Y)];
        lastpos[to_index(Axes::Z)]=current_position[to_index(Axes::Z)];
        lastpos[to_index(Axes::E)]=current_position[to_index(Axes::E)];

        target[to_index(Axes::E)] -= retract_length/volume_to_filament_length[active_extruder];
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], retract_feedrate/60, active_extruder);

        //lift Z
        if(code_seen('Z'))
        {
          target[to_index(Axes::Z)]+= code_value();
        }
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)]/60, active_extruder);

        //move xy
        if(code_seen('X'))
        {
          target[to_index(Axes::X)] = code_value();
        }
        if(code_seen('Y'))
        {
          target[to_index(Axes::Y)] = code_value();
        }
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], homing_feedrate[to_index(Axes::X)]/60, active_extruder);

        if(code_seen('L'))
        {
          target[to_index(Axes::E)] -= code_value()/volume_to_filament_length[active_extruder];
        }
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], retract_feedrate/60, active_extruder);

        current_position[to_index(Axes::X)] = target[to_index(Axes::X)];
        current_position[to_index(Axes::Y)] = target[to_index(Axes::Y)];
        current_position[to_index(Axes::Z)] = target[to_index(Axes::Z)];
        current_position[to_index(Axes::E)] = target[to_index(Axes::E)];
        //finish moves
        st_synchronize();
        //disable extruder steppers so filament can be removed
        disable_e0();
        disable_e1();
        disable_e2();
        while(card.pause){
          manage_heater();
          manage_inactivity();
          lcd_update();
          lifetime_stats_tick();
        }

        //return to normal
        if(code_seen('L'))
        {
          target[to_index(Axes::E)] += code_value()/volume_to_filament_length[active_extruder];
        }
        plan_buffer_line(target[to_index(Axes::X)], target[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], retract_feedrate/60, active_extruder); //Move back the L feed.
        plan_buffer_line(lastpos[to_index(Axes::X)], lastpos[to_index(Axes::Y)], target[to_index(Axes::Z)], target[to_index(Axes::E)], homing_feedrate[to_index(Axes::X)]/60, active_extruder); //move xy back
        plan_buffer_line(lastpos[to_index(Axes::X)], lastpos[to_index(Axes::Y)], lastpos[to_index(Axes::Z)], target[to_index(Axes::E)], homing_feedrate[to_index(Axes::Z)]/60, active_extruder); //move z back
        plan_buffer_line(lastpos[to_index(Axes::X)], lastpos[to_index(Axes::Y)], lastpos[to_index(Axes::Z)], lastpos[to_index(Axes::E)], retract_feedrate/60, active_extruder); //final untretract
        current_position[to_index(Axes::X)] = lastpos[to_index(Axes::X)];
        current_position[to_index(Axes::Y)] = lastpos[to_index(Axes::Y)];
        current_position[to_index(Axes::Z)] = lastpos[to_index(Axes::Z)];
        current_position[to_index(Axes::E)] = lastpos[to_index(Axes::E)];
    }
    break;

    case 605: // M605 store current set values
    {
      uint8_t tmp_select;
      if (code_seen('S')) 
      {
        tmp_select = code_value();
        if (tmp_select>9) tmp_select=9;
      }
      else
        tmp_select = 0;
      machinesettings_tempsave[tmp_select].feedmultiply = feedmultiply;
      machinesettings_tempsave[tmp_select].BedTemperature = target_temperature_bed;
      machinesettings_tempsave[tmp_select].fanSpeed = fanSpeed;
      for (int i=0; i<EXTRUDERS; i++)
      {
        machinesettings_tempsave[tmp_select].HotendTemperature[i] = target_temperature[i];
        machinesettings_tempsave[tmp_select].extrudemultiply[i] = extrudemultiply[i];
      }
      for (int i=0; i<NUM_AXIS; i++)
      {
        machinesettings_tempsave[tmp_select].max_acceleration_units_per_sq_second[i] = max_acceleration_units_per_sq_second[i];
        machinesettings_tempsave[tmp_select].max_feedrate[i] = max_feedrate[i];
      }
      machinesettings_tempsave[tmp_select].acceleration = acceleration;
      machinesettings_tempsave[tmp_select].minimumfeedrate = minimumfeedrate;
      machinesettings_tempsave[tmp_select].mintravelfeedrate = mintravelfeedrate;
      machinesettings_tempsave[tmp_select].minsegmenttime = minsegmenttime;
      machinesettings_tempsave[tmp_select].max_xy_jerk = max_xy_jerk;
      machinesettings_tempsave[tmp_select].max_z_jerk = max_z_jerk;
      machinesettings_tempsave[tmp_select].max_e_jerk = max_e_jerk;
      machinesettings_tempsave[tmp_select].has_saved_settings = 1;
    }
    break;

    case 606: // M606 recall saved values
    {
      uint8_t tmp_select;
      if (code_seen('S')) 
      {
        tmp_select = code_value();
        if (tmp_select>9) tmp_select=9;
      }
      else
        tmp_select = 0;
      if (machinesettings_tempsave[tmp_select].has_saved_settings > 0)
      {
        feedmultiply = machinesettings_tempsave[tmp_select].feedmultiply;
        target_temperature_bed = machinesettings_tempsave[tmp_select].BedTemperature;
        fanSpeed = machinesettings_tempsave[tmp_select].fanSpeed;
        for (int i=0; i<EXTRUDERS; i++)
        {
          target_temperature[i] = machinesettings_tempsave[tmp_select].HotendTemperature[i];
          extrudemultiply[i] = machinesettings_tempsave[tmp_select].extrudemultiply[i];
        }
        for (int i=0; i<NUM_AXIS; i++)
        {
          max_acceleration_units_per_sq_second[i] = machinesettings_tempsave[tmp_select].max_acceleration_units_per_sq_second[i];
          max_feedrate[i] = machinesettings_tempsave[tmp_select].max_feedrate[i];
        }
        acceleration = machinesettings_tempsave[tmp_select].acceleration;
        minimumfeedrate = machinesettings_tempsave[tmp_select].minimumfeedrate;
        mintravelfeedrate = machinesettings_tempsave[tmp_select].mintravelfeedrate;
        minsegmenttime = machinesettings_tempsave[tmp_select].minsegmenttime;
        max_xy_jerk = machinesettings_tempsave[tmp_select].max_xy_jerk;
        max_z_jerk = machinesettings_tempsave[tmp_select].max_z_jerk;
        max_e_jerk = machinesettings_tempsave[tmp_select].max_e_jerk;
      }
    }
    break;
    #endif//ENABLE_ULTILCD2

    case 907: // M907 Set digital trimpot motor current using axis codes.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_current(i,code_value());
        if(code_seen('B')) digipot_current(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) digipot_current(i,code_value());
      #endif
      #if defined(MOTOR_CURRENT_PWM_XY_PIN) && MOTOR_CURRENT_PWM_XY_PIN > -1
        if(code_seen('X')) digipot_current(0, code_value());
      #endif
      #if defined(MOTOR_CURRENT_PWM_Z_PIN) && MOTOR_CURRENT_PWM_Z_PIN > -1
        if(code_seen('Z')) digipot_current(1, code_value());
      #endif
      #if defined(MOTOR_CURRENT_PWM_E_PIN) && MOTOR_CURRENT_PWM_E_PIN > -1
        if(code_seen('E')) digipot_current(2, code_value());
      #endif
    }
    break;
    case 908: // M908 Control digital trimpot directly.
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
      #endif
    }
    break;
    case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
    }
    break;
    case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
      #endif
    }
    break;
    case 999: // M999: Restart after being stopped
      Stopped = false;
      lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      FlushSerialRequestResend();
    break;
#ifdef ENABLE_ULTILCD2
    case 10000://M10000 - Clear the whole LCD
        lcd_lib_clear();
        break;
    case 10001://M10001 - Draw text on LCD, M10002 X0 Y0 SText
        {
        uint8_t x = 0, y = 0;
        if (code_seen('X')) x = code_value_long();
        if (code_seen('Y')) y = code_value_long();
        if (code_seen('S')) lcd_lib_draw_string(x, y, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1]);
        }
        break;
    case 10002://M10002 - Draw inverted text on LCD, M10002 X0 Y0 SText
        {
        uint8_t x = 0, y = 0;
        if (code_seen('X')) x = code_value_long();
        if (code_seen('Y')) y = code_value_long();
        if (code_seen('S')) lcd_lib_clear_string(x, y, &cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1]);
        }
        break;
    case 10003://M10003 - Draw square on LCD, M10003 X1 Y1 W10 H10
        {
        uint8_t x = 0, y = 0, w = 1, h = 1;
        if (code_seen('X')) x = code_value_long();
        if (code_seen('Y')) y = code_value_long();
        if (code_seen('W')) w = code_value_long();
        if (code_seen('H')) h = code_value_long();
        lcd_lib_set(x, y, x + w, y + h);
        }
        break;
    case 10004://M10004 - Draw shaded square on LCD, M10004 X1 Y1 W10 H10
        {
        uint8_t x = 0, y = 0, w = 1, h = 1;
        if (code_seen('X')) x = code_value_long();
        if (code_seen('Y')) y = code_value_long();
        if (code_seen('W')) w = code_value_long();
        if (code_seen('H')) h = code_value_long();
        lcd_lib_draw_shade(x, y, x + w, y + h);
        }
        break;
    case 10005://M10005 - Draw shaded square on LCD, M10004 X1 Y1 W10 H10
        {
        uint8_t x = 0, y = 0, w = 1, h = 1;
        if (code_seen('X')) x = code_value_long();
        if (code_seen('Y')) y = code_value_long();
        if (code_seen('W')) w = code_value_long();
        if (code_seen('H')) h = code_value_long();
        lcd_lib_draw_shade(x, y, x + w, y + h);
        }
        break;
    case 10010://M10010 - Request LCD screen button info (R:[rotation difference compared to previous request] B:[button down])
        {
			report_lcd_button_info(lcd_lib_encoder_pos, lcd_lib_button_down);
			lcd_lib_encoder_pos = 0;
            return;
        }
        break;
#endif//ENABLE_ULTILCD2
    }
  }

  else if(code_seen('T'))
  {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
    	report_invalid_extruder_specified(tmp_extruder);
		}
    else {
      boolean make_move = false;
      if(code_seen('F')) {
        make_move = true;
        next_feedrate = code_value();
        if(next_feedrate > 0.0) {
          feedrate = next_feedrate;
        }
      }
      #if EXTRUDERS > 1
      if(tmp_extruder != active_extruder) {
        // Save current position to return to after applying extruder offset
        memcpy(destination, current_position, sizeof(destination));
        // Offset extruder (only by XY)
        int i;
        for(i = 0; i < 2; i++) {
           current_position[i] = current_position[i] -
                                 extruder_offset[i][active_extruder] +
                                 extruder_offset[i][tmp_extruder];
        }
        // Set the new active extruder and position
        active_extruder = tmp_extruder;
        plan_set_position(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)], current_position[to_index(Axes::E)]);
        // Move to the old position if 'F' was in the parameters
        if(make_move && Stopped == false) {
           prepare_move();
        }
      }
      #endif
			report_active_extruder( active_extruder );
		}
  }
  else if (strcmp_P(cmdbuffer[bufindr], PSTR("Electronics_test")) == 0)
  {
    run_electronics_test();
  }
  else
  {
		report_unknown_command(cmdbuffer[bufindr]);
	}
  printing_state = PRINT_STATE::NORMAL;

  ClearToSend();
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
	issue_resend_request(gcode_LastN);
	ClearToSend();
}


void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif //SDSUPPORT
	report_ok();
}

void get_coordinates()
{
    bool seen[4]={false,false,false,false};
    for(int8_t i=0; i < NUM_AXIS; i++)
    {
        if(code_seen(axis_codes[i]))
        {
            destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
            seen[i]=true;
        }
        else
        {
            destination[i] = current_position[i]; //Are these else lines really needed?
        }
    }
    if(code_seen('F'))
    {
        next_feedrate = code_value();
        if(next_feedrate > 0.0) feedrate = next_feedrate;
    }
    #ifdef FWRETRACT
    if(autoretract_enabled)
    {
        if( !(seen[to_index(Axes::X)] || seen[to_index(Axes::Y)] || seen[to_index(Axes::Z)]) && seen[to_index(Axes::E)])
        {
            float echange=destination[to_index(Axes::E)]-current_position[to_index(Axes::E)];
            if(echange<-MIN_RETRACT) //retract
            {
                if(!retracted)
                {
                    destination[to_index(Axes::Z)]+=retract_zlift; //not sure why chaninging current_position negatively does not work.
                    //if slicer retracted by echange=-1mm and you want to retract 3mm, corrrectede=-2mm additionally
                    float correctede=-echange-retract_length;
                    //to generate the additional steps, not the destination is changed, but inversely the current position
                    current_position[to_index(Axes::E)]+=-correctede;
                    feedrate=retract_feedrate;
                    retracted=true;
                }
            }
            else if(echange>MIN_RETRACT) //retract_recover
            {
                if(retracted)
                {
                    //current_position[Z_AXIS]+=-retract_zlift;
                    //if slicer retracted_recovered by echange=+1mm and you want to retract_recover 3mm, corrrectede=2mm additionally
                    float correctede=-echange+1*retract_length+retract_recover_length; //total unretract=retract_length+retract_recover_length[surplus]
                    current_position[to_index(Axes::E)]+=correctede; //to generate the additional steps, not the destination is changed, but inversely the current position
                    feedrate=retract_recover_feedrate;
                    retracted=false;
                }
            }
        }
    }
    #endif //FWRETRACT
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void clamp_to_software_endstops(float target[3])
{
  if (min_software_endstops) {
    if (target[to_index(Axes::X)] < min_pos[to_index(Axes::X)]) target[to_index(Axes::X)] = min_pos[to_index(Axes::X)];
    if (target[to_index(Axes::Y)] < min_pos[to_index(Axes::Y)]) target[to_index(Axes::Y)] = min_pos[to_index(Axes::Y)];
    if (target[to_index(Axes::Z)] < min_pos[to_index(Axes::Z)]) target[to_index(Axes::Z)] = min_pos[to_index(Axes::Z)];
  }

  if (max_software_endstops) {
    if (target[to_index(Axes::X)] > max_pos[to_index(Axes::X)]) target[to_index(Axes::X)] = max_pos[to_index(Axes::X)];
    if (target[to_index(Axes::Y)] > max_pos[to_index(Axes::Y)]) target[to_index(Axes::Y)] = max_pos[to_index(Axes::Y)];
    if (target[to_index(Axes::Z)] > max_pos[to_index(Axes::Z)]) target[to_index(Axes::Z)] = max_pos[to_index(Axes::Z)];
  }
}

#ifdef DELTA
void calculate_delta(float cartesian[3])
{
  delta[to_index(Axes::X)] = sqrt(sq(DELTA_DIAGONAL_ROD)
                       - sq(DELTA_TOWER1_X-cartesian[to_index(Axes::X)])
                       - sq(DELTA_TOWER1_Y-cartesian[to_index(Axes::Y)])
                       ) + cartesian[to_index(Axes::Z)];
  delta[to_index(Axes::Y)] = sqrt(sq(DELTA_DIAGONAL_ROD)
                       - sq(DELTA_TOWER2_X-cartesian[to_index(Axes::X)])
                       - sq(DELTA_TOWER2_Y-cartesian[to_index(Axes::Y)])
                       ) + cartesian[to_index(Axes::Z)];
  delta[to_index(Axes::Z)] = sqrt(sq(DELTA_DIAGONAL_ROD)
                       - sq(DELTA_TOWER3_X-cartesian[to_index(Axes::X)])
                       - sq(DELTA_TOWER3_Y-cartesian[to_index(Axes::Y)])
                       ) + cartesian[to_index(Axes::Z)];
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);

  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  */
}
#endif

void prepare_move()
{
  clamp_to_software_endstops(destination);

  previous_millis_cmd = millis();
#ifdef DELTA
  float difference[NUM_AXIS];
  for (int8_t i=0; i < NUM_AXIS; i++) {
    difference[i] = destination[i] - current_position[i];
  }
  float cartesian_mm = sqrt(sq(difference[to_index(Axes::X)]) +
                            sq(difference[to_index(Axes::Y)]) +
                            sq(difference[to_index(Axes::Z)]));
  if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[to_index(Axes::E)]); }
  if (cartesian_mm < 0.000001) { return; }
  float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
  int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));
  // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
  // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
  // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);
  for (int s = 1; s <= steps; s++) {
    float fraction = float(s) / float(steps);
    for(int8_t i=0; i < NUM_AXIS; i++) {
      destination[i] = current_position[i] + difference[i] * fraction;
    }
    calculate_delta(destination);
    plan_buffer_line(delta[to_index(Axes::X)], delta[to_index(Axes::Y)], delta[to_index(Axes::Z)],
                     destination[to_index(Axes::E)], feedrate*feedmultiply/60/100.0,
                     active_extruder);
  }
#else
  // Do not use feedmultiply for E or Z only moves
  if( (current_position[to_index(Axes::X)] == destination [to_index(Axes::X)]) && (current_position[to_index(Axes::Y)] == destination [to_index(Axes::Y)])) {
      plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate/60, active_extruder);
  }
  else {
    plan_buffer_line(destination[to_index(Axes::X)], destination[to_index(Axes::Y)], destination[to_index(Axes::Z)], destination[to_index(Axes::E)], feedrate*feedmultiply/60/100.0, active_extruder);
  }
#endif
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[to_index(Axes::X)], offset[to_index(Axes::Y)]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, to_index(Axes::X), to_index(Axes::Y), to_index(Axes::Z), feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  }
  previous_millis_cmd = millis();
}

#if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1

#if defined(FAN_PIN)
  #if CONTROLLERFAN_PIN == FAN_PIN
    #error "You cannot set CONTROLLERFAN_PIN equal to FAN_PIN"
  #endif
#endif

unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();

    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN)
    #if EXTRUDERS > 2
       || !READ(E2_ENABLE_PIN)
    #endif
    #if EXTRUDER > 1
       || !READ(E1_ENABLE_PIN)
    #endif
       || !READ(E0_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }

    if ((millis() - lastMotor) >= (CONTROLLERFAN_SECS*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
        digitalWrite(CONTROLLERFAN_PIN, 0);
        analogWrite(CONTROLLERFAN_PIN, 0);
    }
    else
    {
        // allows digital or PWM fan output to be used (see M42 handling)
        digitalWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
        analogWrite(CONTROLLERFAN_PIN, CONTROLLERFAN_SPEED);
    }
  }
}
#endif

void manage_inactivity()
{
  if( (millis() - previous_millis_cmd) >  max_inactive_time )
    if(max_inactive_time)
      kill();
  if(stepper_inactive_time)  {
    if( (millis() - previous_millis_cmd) >  stepper_inactive_time )
    {
      if(blocks_queued() == false) {
        if(DISABLE_X) disable_x();
        if(DISABLE_Y) disable_y();
        if(DISABLE_Z) disable_z();
        if(DISABLE_E) {
            disable_e0();
            disable_e1();
            disable_e2();
        }
      }
    }
  }
  #if defined(KILL_PIN) && KILL_PIN > -1
    if( 0 == READ(KILL_PIN) )
      kill();
  #endif
  #if defined(SAFETY_TRIGGERED_PIN) && SAFETY_TRIGGERED_PIN > -1
  if (READ(SAFETY_TRIGGERED_PIN))
    Stop(STOP_REASON_SAFETY_TRIGGER);
  #endif
  #if defined(CONTROLLERFAN_PIN) && CONTROLLERFAN_PIN > -1
    controllerFan(); //Check if fan should be turned on to cool stepper drivers down
  #endif
  #ifdef EXTRUDER_RUNOUT_PREVENT
    if( (millis() - previous_millis_cmd) >  EXTRUDER_RUNOUT_SECONDS*1000 )
    if(degHotend(active_extruder)>EXTRUDER_RUNOUT_MINTEMP)
    {
     bool oldstatus=READ(E0_ENABLE_PIN);
     enable_e0();
     float oldepos=current_position[to_index(Axes::E)];
     float oldedes=destination[to_index(Axes::E)];
     plan_buffer_line(current_position[to_index(Axes::X)], current_position[to_index(Axes::Y)], current_position[to_index(Axes::Z)],
                      current_position[to_index(Axes::E)]+EXTRUDER_RUNOUT_EXTRUDE*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[to_index(Axes::E)],
                      EXTRUDER_RUNOUT_SPEED/60.*EXTRUDER_RUNOUT_ESTEPS/axis_steps_per_unit[to_index(Axes::E)], active_extruder);
     current_position[to_index(Axes::E)]=oldepos;
     destination[to_index(Axes::E)]=oldedes;
     plan_set_e_position(oldepos);
     previous_millis_cmd=millis();
     st_synchronize();
     WRITE(E0_ENABLE_PIN,oldstatus);
    }
  #endif
  check_axes_activity();
}


void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();

#if defined(PS_ON_PIN) && PS_ON_PIN > -1
  pinMode(PS_ON_PIN,INPUT);
#endif
	report_printer_killed();
	LCD_ALERTMESSAGEPGM(MSG_KILLED);
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}


void Stop(uint8_t reasonNr)
{
  disable_heater();
  if(Stopped == false) {
    Stopped = reasonNr;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    report_printer_stopped();
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };
uint8_t StoppedReason() { return Stopped; };

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val)
{
  val &= 0x07;
  switch(digitalPinToTimer(pin))
  {

    #if defined(TCCR0A)
    case TIMER0A:
    case TIMER0B:
//         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
//         TCCR0B |= val;
         break;
    #endif

    #if defined(TCCR1A)
    case TIMER1A:
    case TIMER1B:
//         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
//         TCCR1B |= val;
         break;
    #endif

    #if defined(TCCR2)
    case TIMER2:
    case TIMER2:
         TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
         TCCR2 |= val;
         break;
    #endif

    #if defined(TCCR2A)
    case TIMER2A:
    case TIMER2B:
         TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
         TCCR2B |= val;
         break;
    #endif

    #if defined(TCCR3A)
    case TIMER3A:
    case TIMER3B:
    case TIMER3C:
         TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
         TCCR3B |= val;
         break;
    #endif

    #if defined(TCCR4A)
    case TIMER4A:
    case TIMER4B:
    case TIMER4C:
         TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
         TCCR4B |= val;
         break;
   #endif

    #if defined(TCCR5A)
    case TIMER5A:
    case TIMER5B:
    case TIMER5C:
         TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
         TCCR5B |= val;
         break;
   #endif

  }
}
#endif //FAST_PWM_FAN


bool setTargetedHotend(int code){
  tmp_extruder = active_extruder;
  if(code_seen('T')) {
    tmp_extruder = code_value();
    if(tmp_extruder >= EXTRUDERS) {
    	report_wrong_extruder_specified(code,tmp_extruder);
    	return true;
    }
  }
  return false;
}

