/*
 * printer_to_remote.h
 *
 *  Created on: Sep 10, 2015
 *      Author: johan
 */

#ifndef PRINTER_TO_REMOTE_H_
#define PRINTER_TO_REMOTE_H_

#include "../Marlin/MarlinSerial.h"
#include "../Marlin/language.h"

#define MYSERIAL MSerial

#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
#define SERIAL_PROTOCOLLN(x) do {MYSERIAL.print(x);MYSERIAL.write('\n');} while(0)
#define SERIAL_PROTOCOLLNPGM(x) do{serialprintPGM(PSTR(x));MYSERIAL.write('\n');} while(0)


const char errormagic[] PROGMEM ="Error:";
const char echomagic[] PROGMEM ="echo:";
#define SERIAL_ERROR_START serialprintPGM(errormagic);
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic);
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


//things to write to serial from Programmemory. saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str)
{
  char ch=pgm_read_byte(str);
  while(ch)
  {
    MYSERIAL.write(ch);
    ch=pgm_read_byte(++str);
  }
}


inline void report_wrong_extruder_specified(int command_code, uint8_t extruder) {
	SERIAL_ECHO_START;
	switch (command_code) {
	case 104:
		SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
		break;
	case 105:
		SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
		break;
	case 109:
		SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
		break;
	case 218:
		SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
		break;
	}
	SERIAL_ECHOLN(extruder);
}

inline void report_ok() {
	SERIAL_PROTOCOLLNPGM(MSG_OK);
}

inline void report_printer_killed() {
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
}

inline void report_printer_stopped() {
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
}


inline void report_cold_extrusion_prevented() {
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM(MSG_ERR_COLD_EXTRUDE_STOP);
}

inline void report_too_long_extrusion_prevented() {
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM(MSG_ERR_LONG_EXTRUDE_STOP);
}

inline void report_current_printer_settings(
		float axis_steps_per_unit[],
		float max_feedrate[],
		unsigned long max_acceleration_units_per_sq_second[],
		float acceleration,
		float retract_acceleration,
		float minimumfeedrate,
		float mintravelfeedrate,
		unsigned long minsegmenttime,
		float max_xy_jerk,
		float max_z_jerk,
		float max_e_jerk,
		float add_homeing[],
		float Kp,
		float unscalePID_i,
		float unscalePID_d
) {
	// Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Steps per unit:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M92 X", axis_steps_per_unit[0]);
	SERIAL_ECHOPAIR(" Y", axis_steps_per_unit[1]);
	SERIAL_ECHOPAIR(" Z", axis_steps_per_unit[2]);
	SERIAL_ECHOPAIR(" E", axis_steps_per_unit[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Maximum feedrates (mm/s):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M203 X", max_feedrate[0]);
	SERIAL_ECHOPAIR(" Y", max_feedrate[1]);
	SERIAL_ECHOPAIR(" Z", max_feedrate[2]);
	SERIAL_ECHOPAIR(" E", max_feedrate[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Maximum Acceleration (mm/s2):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M201 X", max_acceleration_units_per_sq_second[0]);
	SERIAL_ECHOPAIR(" Y", max_acceleration_units_per_sq_second[1]);
	SERIAL_ECHOPAIR(" Z", max_acceleration_units_per_sq_second[2]);
	SERIAL_ECHOPAIR(" E", max_acceleration_units_per_sq_second[3]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Acceleration: S=acceleration, T=retract acceleration");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M204 S", acceleration);
	SERIAL_ECHOPAIR(" T", retract_acceleration);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM(
			"Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M205 S", minimumfeedrate);
	SERIAL_ECHOPAIR(" T", mintravelfeedrate);
	SERIAL_ECHOPAIR(" B", minsegmenttime);
	SERIAL_ECHOPAIR(" X", max_xy_jerk);
	SERIAL_ECHOPAIR(" Z", max_z_jerk);
	SERIAL_ECHOPAIR(" E", max_e_jerk);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Home offset (mm):");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("  M206 X", add_homeing[0]);
	SERIAL_ECHOPAIR(" Y", add_homeing[1]);
	SERIAL_ECHOPAIR(" Z", add_homeing[2]);
	SERIAL_ECHOLN("");
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("PID settings:");
	SERIAL_ECHO_START;
	SERIAL_ECHOPAIR("   M301 P", Kp);
	SERIAL_ECHOPAIR(" I", unscalePID_i);
	SERIAL_ECHOPAIR(" D", unscalePID_d);
	SERIAL_ECHOLN("");
}

inline void report_settings_stored() {
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Settings Stored");
}

inline void report_settings_retrieved() {
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Stored settings retrieved");
}

inline void report_factory_settings_restored() {
	SERIAL_ECHO_START;
	SERIAL_ECHOLNPGM("Hardcoded Default Settings Loaded");
}

inline void report_enqueing_command( const char* enqued_command) {
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM("enqueing \"");
	SERIAL_ECHO( enqued_command );
	SERIAL_ECHOLNPGM("\"");
}

inline void report_startup_status(byte mcu) {
	if (mcu & 1)SERIAL_ECHOLNPGM(MSG_POWERUP);
	if (mcu & 2)SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
	if (mcu & 4)SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
	if (mcu & 8)SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
	if (mcu & 32)SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
}

inline void report_firmware_information(int free_memory, int planner_buffer_bytes){
	  SERIAL_ECHOPGM(MSG_MARLIN);
	  SERIAL_ECHOLNPGM(VERSION_STRING);
	  #ifdef STRING_VERSION_CONFIG_H
	    #ifdef STRING_CONFIG_H_AUTHOR
	      SERIAL_ECHO_START;
	      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
	      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
	      SERIAL_ECHOPGM(MSG_AUTHOR);
	      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
	      SERIAL_ECHOPGM("Compiled: ");
	      SERIAL_ECHOLNPGM(__DATE__);
	    #endif
	  #endif
	  SERIAL_ECHO_START;
	  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
	  SERIAL_ECHO(free_memory);
	  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
	  SERIAL_ECHOLN(planner_buffer_bytes);
}

inline void report_file_saved() {
	SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
}

inline void report_active_extruder(uint8_t extruder) {
	SERIAL_ECHO_START;
	SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
	SERIAL_PROTOCOLLN(extruder);
}

inline void report_unknown_command( const char* unknown_command) {
	SERIAL_ECHO_START;
	SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
	SERIAL_ECHO( unknown_command );
	SERIAL_ECHOLNPGM("\"");
}

inline void issue_resend_request(long LastN) {
	MYSERIAL.flush();
	SERIAL_PROTOCOLPGM(MSG_RESEND);
	SERIAL_PROTOCOLLN(LastN + 1);
}

inline void report_error_in_line_number(long LastN) {
	SERIAL_ERROR_START;
	SERIAL_ERRORPGM(MSG_ERR_LINE_NO);
	SERIAL_ERRORLN(LastN);
}

inline void report_error_in_checksum(long LastN) {
	SERIAL_ERROR_START;
	SERIAL_ERRORPGM(MSG_ERR_CHECKSUM_MISMATCH);
	SERIAL_ERRORLN(LastN);
}

inline void report_missing_checksum(long LastN) {
	SERIAL_ERROR_START;
	SERIAL_ERRORPGM(MSG_ERR_NO_CHECKSUM);
	SERIAL_ERRORLN(LastN);
}

inline void report_no_line_number_with_checksum(long LastN) {
	SERIAL_ERROR_START;
	SERIAL_ERRORPGM(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM);
	SERIAL_ERRORLN(LastN);
}

inline void report_file_printed(char time[30]) {
	SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
	SERIAL_ECHO_START;
	SERIAL_ECHOLN(time);
}

inline void report_endstop_not_pressed_after_homing() {
	SERIAL_ERROR_START;
	SERIAL_ERRORLNPGM("Endstop not pressed after homing down. Endstop broken?");
}


#endif /* PRINTER_TO_REMOTE_H_ */
