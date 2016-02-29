/*
 * printer_to_remote.h
 *
 *  Created on: Sep 10, 2015
 *      Author: johan
 */

#ifndef PRINTER_TO_REMOTE_H_
#define PRINTER_TO_REMOTE_H_

#include "language_serial_communication.h"
#include "printer_data_types.h"
#include "../Marlin/MarlinSerial.h"

#ifdef REMOTE_SERIAL
#include <iostream>
#else
namespace std{
	class ostream;
	template<typename T>
	ostream& operator<<(ostream&,T);
}
#endif


//#define MYSERIAL MSerial
//
//#define SERIAL_PROTOCOL(x) MYSERIAL.print(x);
//#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y);
//#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x));
//#define SERIAL_PROTOCOLLN(x) do {MYSERIAL.print(x);MYSERIAL.write('\n');} while(0)
//#define SERIAL_PROTOCOLLNPGM(x) do{serialprintPGM(PSTR(x));MYSERIAL.write('\n');} while(0)


const char errormagic[] ="Error:";
const char echomagic[] ="echo:";
//#define SERIAL_ERROR_START serialprintPGM(errormagic);
//#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
//#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
//#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
//#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)
//
//#define SERIAL_ECHO_START serialprintPGM(echomagic);
//#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
//#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
//#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
//#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)
//
//#define SERIAL_ECHOPAIR(name,value) (serial_echopair_P(PSTR(name),(value)))
//
//void serial_echopair_P(const char *s_P, float v);
//void serial_echopair_P(const char *s_P, double v);
//void serial_echopair_P(const char *s_P, unsigned long v);



////things to write to serial from Programmemory. saves 400 to 2k of RAM.
//FORCE_INLINE void serialprintPGM(const char *str)
//{
//  char ch=pgm_read_byte(str);
//  while(ch)
//  {
//    MYSERIAL.write(ch);
//    ch=pgm_read_byte(++str);
//  }
//}



inline void report_startup_status( byte mcu ){
	MSerial.send<printer_message::REPORT_STARTUP_STATUS>( mcu );
}

inline std::ostream& remote_report_startup_status( std::ostream& os, byte mcu) {
	os << "start\n" << "echo:";
	if (mcu & 1) os << (MSG_POWERUP);
	if (mcu & 2) os << (MSG_EXTERNAL_RESET);
	if (mcu & 4) os << (MSG_BROWNOUT_RESET);
	if (mcu & 8) os << (MSG_WATCHDOG_RESET);
	if (mcu & 32) os << (MSG_SOFTWARE_RESET);
	return os << "\n";
}



inline void report_firmware_capabilities_string( const char* capabilities_string ){
	MSerial.send<printer_message::REPORT_FIRMWARE_CAPABILITIES_STRING>(capabilities_string);
}

inline std::ostream& remote_report_firmware_capabilities_string(std::ostream& os, const char* capabilities_string ){
	return os << capabilities_string;
}



inline void report_ok() {
	MSerial.send<printer_message::REPORT_OK>();
}

inline std::ostream& remote_report_ok( std::ostream& os ) {
	return os << MSG_OK << "\n";
}



inline void report_printer_killed() {
	MSerial.send<printer_message::REPORT_PRINTER_KILLED>();
}

inline std::ostream& remote_report_printer_killed( std::ostream& os ) {
	return os << errormagic << (MSG_ERR_KILLED) << "\n";
}



inline void report_printer_stopped() {
	MSerial.send<printer_message::REPORT_PRINTER_STOPPED>();
}

inline std::ostream& remote_report_printer_stopped( std::ostream& os ) {
	return os << errormagic << (MSG_ERR_STOPPED) << "\n";
}


inline void report_step_rate_too_high(unsigned short step_rate) {
	MSerial.send<printer_message::REPORT_STEP_RATE_TOO_HIGH>(step_rate);
}

inline std::ostream& remote_report_step_rate_to_high(std::ostream& os, unsigned short step_rate) {
	return os << (MSG_STEPPER_TOO_HIGH) << step_rate << "\n";
}


inline void report_cold_extrusion_prevented() {
	MSerial.send<printer_message::REPORT_COLD_EXTRUSION_PREVENTED>();
}

inline std::ostream& remote_report_cold_extrusion_prevented( std::ostream& os) {
	return os << echomagic << (MSG_ERR_COLD_EXTRUDE_STOP) << "\n";
}



inline void report_too_long_extrusion_prevented() {
	MSerial.send<printer_message::REPORT_TOO_LONG_EXTRUSION_PREVENTED>();
}

inline std::ostream& remote_report_too_long_extrusion_prevented(std::ostream& os) {
	return os << echomagic << MSG_ERR_LONG_EXTRUDE_STOP << "\n";
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
	MSerial.send<printer_message::REPORT_CURRENT_PRINTER_SETTINGS>(
			axis_steps_per_unit,
			max_feedrate,
			max_acceleration_units_per_sq_second,
			acceleration,
			retract_acceleration,
			minimumfeedrate,
			mintravelfeedrate,
			minsegmenttime,
			max_xy_jerk,
			max_z_jerk,
			max_e_jerk,
			add_homeing,
			Kp,
			unscalePID_i,
			unscalePID_d );
}

inline std::ostream& remote_report_current_printer_settings( std::ostream& os,
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
	os << "Steps per unit:" << "\n";
	os <<   "  M92 X" << axis_steps_per_unit[0] <<
			" Y" << axis_steps_per_unit[1] <<
			" Z" << axis_steps_per_unit[2] <<
			" E" << axis_steps_per_unit[3] << "\n";
	os << "Maximum feedrates (mm/s):\n";
	os << "  M203 X" << max_feedrate[0] <<
			" Y" << max_feedrate[1] <<
			" Z" << max_feedrate[2] <<
			" E" << max_feedrate[3] << "\n";
	os << "Maximum Acceleration (mm/s2):\n";
	os << "  M201 X" << max_acceleration_units_per_sq_second[0] <<
			" Y" << max_acceleration_units_per_sq_second[1] <<
			" Z" << max_acceleration_units_per_sq_second[2] <<
			" E" << max_acceleration_units_per_sq_second[3] << "\n";

	os << "Acceleration: S=acceleration, T=retract acceleration\n";
	os << "  M204 S" <<  acceleration <<
			" T" << retract_acceleration << "\n";

	os << "Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), B=minimum segment time (ms), X=maximum XY jerk (mm/s),  Z=maximum Z jerk (mm/s),  E=maximum E jerk (mm/s)\n";
	os << "  M205 S" << minimumfeedrate <<
			" T" << mintravelfeedrate <<
			" B" << minsegmenttime <<
			" X" << max_xy_jerk <<
			" Z" << max_z_jerk <<
			" E" << max_e_jerk << "\n";

	os << "Home offset (mm):\n";
	os << "  M206 X" << add_homeing[0] <<
			" Y" << add_homeing[1]<<
			" Z" << add_homeing[2]<< "\n";

	os << "PID settings:\n";
	os << "   M301 P" << Kp <<
			" I" << unscalePID_i <<
			" D" << unscalePID_d << "\n";
	return os;
}



inline void report_settings_stored() {
	MSerial.send<printer_message::REPORT_SETTINGS_STORED>();
}

inline std::ostream& remote_report_settings_stored(std::ostream& os) {
	return os << "Settings Stored\n";
}



inline void report_settings_retrieved() {
	MSerial.send<printer_message::REPORT_SETTINGS_RETRIEVED>();
}

inline std::ostream& remote_report_settings_retrieved(std::ostream& os) {
	return os << "Stored settings retrieved\n";
}



inline void report_factory_settings_restored() {
	MSerial.send<printer_message::REPORT_FACTORY_SETTINGS_RESTORED>();
}

inline std::ostream& remote_report_factory_settings_restored(std::ostream& os) {
	return os << "Hardcoded Default Settings Loaded\n";
}



inline void report_enqueing_command( const char* enqued_command) {
	MSerial.send<printer_message::REPORT_ENQUEING_COMMAND>( enqued_command );
}

inline std::ostream& remote_report_enqueing_command(std::ostream& os, const char* enqued_command) {
	return os << "enqueing \"" << enqued_command << "\"\n";
}





inline void report_firmware_information(int free_memory, int planner_buffer_bytes){
	MSerial.send<printer_message::REPORT_FIRMWARE_INFORMATION>(
			free_memory,planner_buffer_bytes
	);
}

inline std::ostream& remote_report_firmware_information(std::ostream& os, int free_memory, int planner_buffer_bytes){
//TODO: Send more firmware information?
//	  SERIAL_ECHOPGM(MSG_MARLIN);
//	  SERIAL_ECHOLNPGM(VERSION_STRING);
//	  #ifdef STRING_VERSION_CONFIG_H
//	    #ifdef STRING_CONFIG_H_AUTHOR
//	      SERIAL_ECHO_START;
//	      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
//	      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
//	      SERIAL_ECHOPGM(MSG_AUTHOR);
//	      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
//	      SERIAL_ECHOPGM("Compiled: ");
//	      SERIAL_ECHOLNPGM(__DATE__);
//	    #endif
//	  #endif
//	  SERIAL_ECHO_START;
//	  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
//	  SERIAL_ECHO(free_memory);
//	  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
//	  SERIAL_ECHOLN(planner_buffer_bytes);
	os << (MSG_MARLIN) <<  (VERSION_STRING) << "\n";
	  os <<   (MSG_FREE_MEMORY) << free_memory <<
			  (MSG_PLANNER_BUFFER_BYTES) << planner_buffer_bytes << "\n";
	  return os;
}



inline void report_file_saved() {
	MSerial.send<printer_message::REPORT_FILE_SAVED>();
}
inline std::ostream& remote_report_file_saved(std::ostream& os) {
	return os << (MSG_FILE_SAVED) << "\n";
}



inline void report_active_extruder( uint8_t extruder) {
	MSerial.send<printer_message::REPORT_ACTIVE_EXTRUDER>( extruder );
}

inline std::ostream& remote_report_active_extruder(std::ostream& os, uint8_t extruder) {
	return os << (MSG_ACTIVE_EXTRUDER) << extruder << "\n";
}




inline void report_unknown_command( const char* unknown_command) {
	MSerial.send<printer_message::REPORT_UNKNOWN_COMMAND>( unknown_command );
}

inline std::ostream& remote_report_unknown_command( std::ostream& os,const char* unknown_command) {
	return os << (MSG_UNKNOWN_COMMAND) << unknown_command << "\"\n";
}



inline void report_wrong_extruder_specified(int16_t command_code, uint8_t extruder) {
	MSerial.send<printer_message::REPORT_WRONG_EXTRUDER_SPECIFIED>(command_code, extruder );
}
inline std::ostream& remote_report_wrong_extruder_specified(std::ostream& os,int command_code, uint8_t extruder) {
	switch (command_code) {
	case 104:
		os << (MSG_M104_INVALID_EXTRUDER);
		break;
	case 105:
		os << (MSG_M105_INVALID_EXTRUDER);
		break;
	case 109:
		os << (MSG_M109_INVALID_EXTRUDER);
		break;
	case 218:
		os << (MSG_M218_INVALID_EXTRUDER);
		break;
	}
	return os << (extruder) << "\n";
}



inline void report_read_wrong_extruder(uint8_t extruder) {
	MSerial.send<printer_message::REPORT_READ_WRONG_EXTRUDER>(extruder);
}
inline std::ostream& remote_report_read_wrong_extruder(std::ostream& os, uint8_t e) {
	return os << errormagic << +e << " - Invalid extruder number !\n";
}



inline void report_error_maxtemp_triggered(uint8_t e) {
	MSerial.send<printer_message::REPORT_ERROR_MAXTEMP>(e);
}
inline std::ostream& remote_report_error_maxtemp_triggered(std::ostream& os, uint8_t e) {
	return os << errormagic << +e << ": Extruder switched off. MAXTEMP triggered !\n";
}



inline void report_error_mintemp_triggered(uint8_t e) {
	MSerial.send<printer_message::REPORT_ERROR_MINTEMP>(e);
}
inline std::ostream& remote_report_error_mintemp_triggered(std::ostream& os, uint8_t e) {
	return os << errormagic << +e << ": Extruder switched off. MINTEMP triggered !\n";
}




inline void report_error_maxtemp_bed_triggered() {
	MSerial.send<printer_message::REPORT_ERROR_MAXTEMP_BED>();
}

inline std::ostream& remote_report_error_maxtemp_bed_triggered(std::ostream& os) {
	return os << errormagic << "Temperature heated bed switched off. MAXTEMP triggered !!\n";
}



inline void report_invalid_extruder_specified( uint8_t extruder) {
	MSerial.send<printer_message::REPORT_INVALID_EXTRUDER_SPECIFIED>(extruder);
}

inline std::ostream& remote_report_invalid_extruder_specified(std::ostream& os, uint8_t extruder) {
	return os << "T" << extruder << " " << (MSG_INVALID_EXTRUDER) << "\n";
}




inline void issue_resend_request(long LastN) {
//TODO::Should this be here: MYSERIAL.flush();
	MSerial.send<printer_message::ISSUE_RESEND_REQUEST>(LastN+1);
}

inline std::ostream& remote_issue_resend_request(std::ostream& os, long LastN) {
	return os << (MSG_RESEND) << LastN << "\n";
}



inline void report_error_in_line_number(long LastN) {
	MSerial.send<printer_message::REPORT_ERROR_IN_LINE_NUMBER>();
}

inline std::ostream& remote_report_error_in_line_number(std::ostream& os, long LastN) {
	return os << (MSG_ERR_LINE_NO) << (LastN) << "\n";
}



inline void report_error_in_checksum(long LastN) {
	MSerial.send<printer_message::REPORT_ERROR_IN_CHECKSUM>(LastN);
}

inline std::ostream& remote_report_error_in_checksum(std::ostream& os, long LastN) {
	return os << (MSG_ERR_CHECKSUM_MISMATCH) << (LastN) << "\n";
}



inline void report_missing_checksum(long LastN) {
	MSerial.send<printer_message::REPORT_MISSING_CHECKSUM>(LastN);
}

inline std::ostream& remote_report_missing_checksum(std::ostream& os, long LastN) {
	return os << (MSG_ERR_NO_CHECKSUM) << (LastN) << "\n";
}



inline void report_no_line_number_with_checksum(long LastN) {
	MSerial.send<printer_message::REPORT_NO_LINE_NUMBER_WITH_CHECKSUM>();
}

inline std::ostream& remote_report_no_line_number_with_checksum(std::ostream& os, long LastN) {
	return os << (MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM) << (LastN) << "\n";
}


inline void report_time_elapsed(unsigned long time_seconds ) {
	MSerial.send<printer_message::REPORT_TIME_ELAPSED>(time_seconds);
}

inline std::ostream& remote_report_time_elapsed( std::ostream& os, unsigned long time_seconds ) {
	return os << (time_seconds) << "\n";
}



inline void report_file_printed( unsigned long time_seconds ) {
	MSerial.send<printer_message::REPORT_FILE_PRINTED>(time_seconds);
}

inline std::ostream& remote_report_file_printed(std::ostream& os, unsigned long time_seconds ) {
	int min = time_seconds / 60;
	int sec = time_seconds % 60;
	return os << (MSG_FILE_PRINTED) << min << " min, " << sec << " sec\n";
}


inline void report_endstop_not_pressed_after_homing() {
	MSerial.send<printer_message::REPORT_ENDSTOP_NOT_PRESSED_AFTER_HOMING>();
}

inline std::ostream& remote_report_endstop_not_pressed_after_homing(std::ostream& os) {
	return os << "Endstop not pressed after homing down. Endstop broken?\n";
}



inline void report_error_endstop_still_pressed() {
	MSerial.send<printer_message::REPORT_ERROR_ENDSTOP_STILL_PRESSED>();
}

inline std::ostream& remote_report_error_endstop_still_pressed( std::ostream& os ){
	return os << "Endstop still pressed after backing off. Endstop stuck?\n";
}



inline void report_temperature_and_power(uint8_t extruder, float deg_hotend, float target_hotend, float deg_bed, float target_bed, uint8_t heater_power, uint8_t bed_power) {
	MSerial.send<printer_message::REPORT_TEMPERATURE_AND_POWER>(extruder, deg_hotend, target_hotend, deg_bed, target_bed, heater_power, bed_power);
}

inline std::ostream& remote_report_temperature_and_power(std::ostream& os, uint8_t extruder, float deg_hotend, float target_hotend, float deg_bed, float target_bed, uint8_t heater_power, uint8_t bed_power) {
	return os << "ok T" << extruder << ":" << deg_hotend << " /" << target_hotend <<
			" B:" << deg_bed << " /" << target_bed <<
			" @:" << heater_power << " B@:" << bed_power << "\n";
}



inline void report_wait_for_temperature( uint8_t extruder, float deg_hotend, char residency, int16_t time_left_seconds ){
	MSerial.send<printer_message::REPORT_WAIT_FOR_TEMPERATURE>(extruder, deg_hotend,residency, time_left_seconds );
}

inline std::ostream& remote_report_wait_for_temperature( std::ostream& os, uint8_t extruder, float deg_hotend, char residency, int16_t time_left_seconds ){
    os << "T: " << deg_hotend << " E: " << extruder;
    if(residency == 'W'){
    	os << " W: " << time_left_seconds << "\n";
    }else if(residency == '?'){
    	os << " ?\n";
    }else{
    	os << "\n";
    }
    return os;
}



inline void report_wait_for_temperature_bed(uint8_t extruder, float deg_hotend, float deg_bed) {
	MSerial.send<printer_message::REPORT_WAIT_FOR_TEMPERATURE_BED>( extruder, deg_hotend, deg_bed);
}

inline std::ostream& remote_report_wait_for_temperature_bed(std::ostream& os, uint8_t extruder, float deg_hotend, float deg_bed) {
	return os << "T:" << deg_hotend << " E:" << extruder << " B:" << deg_bed << "\n";
}



inline void report_lcd_button_info(int16_t encoder_pos, uint8_t button_down) {
	MSerial.send<printer_message::REPORT_LCD_BUTTON_INFO>( encoder_pos, button_down );
}

inline std::ostream& remote_report_lcd_button_info(std::ostream& os, int16_t encoder_pos, uint8_t button_down) {
	return os << "ok R:" << encoder_pos << " B:" << button_down << "\n";
}



inline void report_bed_leveling_probe_sequence(float height_1, float height_2,
		float height_3, float bed_leveling_factor_x, float bed_leveling_factor_y) {
	MSerial.send<printer_message::REPORT_BED_LEVELING_PROBE_SEQUENCE>(
			height_1,
			height_2,
			height_3,
			bed_leveling_factor_x,
			bed_leveling_factor_y);
}

inline std::ostream& remote_report_bed_leveling_probe_sequence( std::ostream& os, float height_1, float height_2,
		float height_3, float bed_leveling_factor_x, float bed_leveling_factor_y) {
	os <<
			height_1 << "\n" <<
			height_2 << "\n" <<
			height_3 << "\n" <<
			bed_leveling_factor_x << "\n" <<
			bed_leveling_factor_y << "\n";
	return os;
}


inline void report_bed_leveling_probe_point(float dest_z) {
	MSerial.send<printer_message::REPORT_BED_LEVELING_PROBE_POINT>( dest_z );
}
inline std::ostream& remote_report_bed_leveling_probe_point( std::ostream& os, float dest_z ) {
	return os << dest_z << "\n";
}


inline void report_single_cap_probe_reading(uint16_t value) {
	MSerial.send<printer_message::REPORT_SINGLE_CAP_PROBE_VALUE>( value );
}

inline std::ostream& remote_report_single_cap_probe_reading( std::ostream& os, uint16_t value) {
#ifdef REMOTE_SERIAL
	auto flags = os.flags();
	os << std::hex << value << "\n";
	os.flags(flags);
#endif
	return os;
}


inline void report_microstep_settings(
		int ms_X1, int ms_X2, int ms_Y1, int ms_Y2, int ms_Z1, int ms_Z2,
		int ms_E0_1, int ms_E0_2, int ms_E1_1, int ms_E1_2
){
	MSerial.send<printer_message::REPORT_MICROSTEP_SETTINGS>(
			ms_X1, ms_X2, ms_Y1, ms_Y2, ms_Z1, ms_Z2,
			ms_E0_1, ms_E0_2, ms_E1_1, ms_E1_2);
}

inline std::ostream& remote_report_microstep_settings(
		std::ostream& os, int ms_X1, int ms_X2, int ms_Y1, int ms_Y2, int ms_Z1, int ms_Z2,
		int ms_E0_1, int ms_E0_2, int ms_E1_1, int ms_E1_2
){
	os << "MS1,MS2 Pins\n";
	os << "X: " << ms_X1 << ms_X2 << "\n";
	os << "Y: " << ms_Y1 << ms_Y2 << "\n";
	os << "Z: " << ms_Z1 << ms_Z2 << "\n";
	os << "E0: " << ms_E0_1 << ms_E0_2 << "\n";
	os << "E1: " << ms_E1_1 << ms_E1_2 << "\n";
	return os;
}


enum class endstop_status: int8_t {
	UNUSED = -1,
	UNTRIGGERD = 0,
	TRIGGERED = 1,
};
inline void report_endstop_status(
		endstop_status x_min, endstop_status x_max,
		endstop_status y_min, endstop_status y_max,
		endstop_status z_min, endstop_status z_max
){
	MSerial.send<printer_message::REPORT_ENDSTOP_STATUS>(
			x_min, x_max, y_min, y_max, z_min, z_max );
}
inline std::ostream& remote_report_endstop_status(
		std::ostream& os,
		endstop_status x_min, endstop_status x_max,
		endstop_status y_min, endstop_status y_max,
		endstop_status z_min, endstop_status z_max
){
	os << (MSG_M119_REPORT) << "\n";
	if( x_min != endstop_status::UNUSED )
		os << (MSG_X_MIN) << (x_min == endstop_status::TRIGGERED ? (MSG_ENDSTOPS_HIT) : (MSG_ENDSTOP_OPEN)) << "\n";
	if( x_max != endstop_status::UNUSED )
		os << (MSG_X_MAX) << (x_max == endstop_status::TRIGGERED ? (MSG_ENDSTOPS_HIT) : (MSG_ENDSTOP_OPEN)) << "\n";
	if( y_min != endstop_status::UNUSED )
		os << (MSG_Y_MIN) << (y_min == endstop_status::TRIGGERED ? (MSG_ENDSTOPS_HIT) : (MSG_ENDSTOP_OPEN)) << "\n";
	if( y_max != endstop_status::UNUSED )
		os << (MSG_Y_MAX) << (y_max == endstop_status::TRIGGERED ? (MSG_ENDSTOPS_HIT) : (MSG_ENDSTOP_OPEN)) << "\n";
	if( z_min != endstop_status::UNUSED )
		os << (MSG_Z_MIN) << (z_min == endstop_status::TRIGGERED ? (MSG_ENDSTOPS_HIT) : (MSG_ENDSTOP_OPEN)) << "\n";
	if( z_max != endstop_status::UNUSED )
		os << (MSG_Z_MAX) << (z_max == endstop_status::TRIGGERED ? (MSG_ENDSTOPS_HIT) : (MSG_ENDSTOP_OPEN)) << "\n";
	return os;
}

inline void report_endstop_hit(Axes axis, float position){
	MSerial.send<printer_message::REPORT_ENDSTOP_HIT>(axis,position);
}

inline std::ostream& remote_report_end_stop_hit(std::ostream& os, Axes axis, float position){
	os << (MSG_ENDSTOPS_HIT);
	switch(axis){
	case Axes::X:
		os << " X:" << position << "\n"; break;
	case Axes::Y:
		os << " Y:" << position << "\n"; break;
	case Axes::Z:
		os << " Z:" << position << "\n"; break;
	//case Axes::E: //TODO: report out of filament??
	default:
		os << "unknown axis (Axes)" << static_cast<int>(axis) << ": " << position << "\n";
	}
	return os;
}


inline void report_current_position(
		float i_x, float i_y, float i_z, float i_e,
		float j_x, float j_y, float j_z
){
	MSerial.send<printer_message::REPORT_CURRENT_POSITION>(
			i_x, i_y,i_z,i_e, j_x, j_y, j_z );
}

inline std::ostream& remote_report_current_position(
		std::ostream& os, float i_x, float i_y, float i_z, float i_e,
		float j_x, float j_y, float j_z
){
	os <<
			"X:" << i_x << "Y:" << i_y << "Z:" << i_z << "E:" << i_e <<
			(MSG_COUNT_X) << j_x << "Y:" << j_y << "Z:" << j_z << "\n";
	return os;
}



inline void report_PID_extruder_parameters(
		float kp, float unscaled_ki,
		float unscaled_kd, float kc
){
	MSerial.send<printer_message::REPORT_PID_EXTRUDER_PARAMETERS>(
			kp, unscaled_ki, unscaled_kd, kc );
}

inline std::ostream& remote_report_PID_extruder_parameters(
		std::ostream& os, float kp, float unscaled_ki,
		float unscaled_kd, float kc
){
	os << (MSG_OK) <<
			" p:" << kp <<
			" i:" << unscaled_ki <<
			" d:" << unscaled_kd <<
			" c:" << kc << "\n";
	return os;
}




inline void report_fiset_full_data( int16_t data_count, uint8_t gain, uint16_t magnitude, int16_t data ){
	MSerial.send<printer_message::REPORT_FISET_FULL_DATA>( data_count, gain, magnitude, data );
}

inline std::ostream& remote_report_fiset_full_data( std::ostream& os, int16_t data_count, uint8_t gain, uint16_t magnitude, int16_t data ){
	return os << "Fiset no " << data_count << ": " << gain << " " << magnitude << " " << data << "\n";
}


template <sdcard_messages sd_message,typename... arg_ts>
inline void send_sd_message(arg_ts... args){
	MSerial.send_sub_message<printer_message::SD_CARD_DATA,sdcard_messages,sd_message>(args...);
}


inline void serial_sd_card_begin_file_list() {
	send_sd_message<sdcard_messages::BEGIN_FILE_LIST>();
}

inline std::ostream& remote_serial_sd_card_begin_file_list( std::ostream& os ) {
	return os << (MSG_BEGIN_FILE_LIST);
}

inline void serial_sd_card_end_file_list() {
	send_sd_message<sdcard_messages::END_FILE_LIST>();
}

inline std::ostream& remote_serial_sd_card_end_file_list( std::ostream& os ) {
	return os << (MSG_END_FILE_LIST);
}

inline void report_sd_list_filename(const char* prepend) {
//	SERIAL_PROTOCOL(prepend);
//	SERIAL_PROTOCOLLN(filename);
}


inline void report_sd_init_card_ok() {
//	SERIAL_ECHO_START;
//	SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
}

inline void report_sd_init_card_failed() {
//	SERIAL_ECHO_START;
//	SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
}

inline void report_sd_init_volume_failed() {
//	SERIAL_ERROR_START;
//	SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
}

inline void report_sd_open_root_failed() {
//	SERIAL_ERROR_START;
//	SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
}

inline void report_sd_subdir_name(char subdirname[13]) {
//	SERIAL_ECHOLN( subdirname );
}

inline void report_sd_file_selected() {
//	SERIAL_PROTOCOLLNPGM( MSG_SD_FILE_SELECTED );
}

inline void report_open_subdir_failed(char lfilename[13]) {
//	SERIAL_ECHO_START;
//	SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
//	SERIAL_ECHOLN(lfilename);
}

inline void report_sd_open_file_failed(const char* fname, bool read) {
	//HANDLE READ
//	SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
//	SERIAL_PROTOCOL(fname);
//	SERIAL_PROTOCOLLNPGM(".");
}

inline void report_sd_open_subdir_file_failed(const char* fname) {
//	SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
//	SERIAL_PROTOCOL(fname);
//	SERIAL_PROTOCOLLNPGM(".");
}

inline void report_sd_file_opened_for_reading(const char* fname) {
//	SERIAL_PROTOCOLPGM( MSG_SD_FILE_OPENED );
//	SERIAL_PROTOCOL( fname );
//	SERIAL_PROTOCOLPGM( MSG_SD_SIZE );
//	SERIAL_PROTOCOLLN( filesize );
}

inline void report_sd_file_opened_for_writing(const char* name) {
//	SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
//	SERIAL_PROTOCOLLN(name);
}

inline void report_sd_file_removed(const char* fname) {
//	SERIAL_PROTOCOLPGM("File deleted:");
//	SERIAL_PROTOCOL(fname);
}

inline void report_sd_remove_file_failed(const char* fname) {
//	SERIAL_PROTOCOLPGM( "Deletion failed, File: " );
//	SERIAL_PROTOCOL( fname );
//	SERIAL_PROTOCOLLNPGM(".");
}


inline void report_sd_card_error() {
//	SERIAL_PROTOCOLPGM("Card error:");
//	SERIAL_PROTOCOLLN(card.errorCode());
}

inline void report_sd_printing_file_status( uint32_t sdpos, uint32_t filesize ) {
//	SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
//	SERIAL_PROTOCOL(sdpos);
//	SERIAL_PROTOCOLPGM("/");
//	SERIAL_PROTOCOLLN(filesize);
}

inline void report_sd_not_printing() {
//	SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
}


inline void report_error_write_to_file() {
//	SERIAL_ERROR_START;
//	SERIAL_ERRORLNPGM( MSG_SD_ERR_WRITE_TO_FILE);
}

inline void report_cant_enter_subdir(const char* relpath) {
//	SERIAL_ECHO_START;
//	SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
//	SERIAL_ECHOLN(relpath);
}




inline void report_PID_autotune_start() {
//	SERIAL_ECHOLN("PID Autotune start");
}

inline void report_PID_autotune_failed_temperature_too_high() {
//	SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature too high");
}

inline void report_PID_autotune_finished() {
//	SERIAL_PROTOCOLLNPGM("PID Autotune finished! Put the Kp, Ki and Kd constants into Configuration.h");
}

inline void report_PID_autotune_failed_timeout() {
//	SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
}

inline void report_PID_autotune_failed_bad_extruder() {
//	SERIAL_ECHOLN("PID Autotune failed. Bad extruder number.");
}

inline void report_PID_autotune_temp_and_power(int extruder, int power, float input) {
//	if (extruder < 0) {
//		SERIAL_PROTOCOLPGM("ok B:");
//	} else {
//		SERIAL_PROTOCOLPGM("ok T:");
//	}
//	SERIAL_PROTOCOL( input );
//	SERIAL_PROTOCOLPGM(" @:");
//	SERIAL_PROTOCOLLN(power);
}

inline void report_PID_autotune_current_parameters(float Ku, float Tu, float Kp, float Ki, float Kd) {
//	SERIAL_PROTOCOLPGM(" Ku: ");
//	SERIAL_PROTOCOL(Ku);
//	SERIAL_PROTOCOLPGM(" Tu: ");
//	SERIAL_PROTOCOLLN(Tu);
//	SERIAL_PROTOCOLLNPGM(" Clasic PID ");
//	SERIAL_PROTOCOLPGM(" Kp: ");
//	SERIAL_PROTOCOLLN(Kp);
//	SERIAL_PROTOCOLPGM(" Ki: ");
//	SERIAL_PROTOCOLLN(Ki);
//	SERIAL_PROTOCOLPGM(" Kd: ");
//	SERIAL_PROTOCOLLN(Kd);
}

inline void report_PID_autotune_current_status(long bias, long d, float min, float max) {
//	SERIAL_PROTOCOLPGM(" bias: ");
//	SERIAL_PROTOCOL(bias);
//	SERIAL_PROTOCOLPGM(" d: ");
//	SERIAL_PROTOCOL(d);
//	SERIAL_PROTOCOLPGM(" min: ");
//	SERIAL_PROTOCOL(min);
//	SERIAL_PROTOCOLPGM(" max: ");
//	SERIAL_PROTOCOLLN(max);
}


#endif /* PRINTER_TO_REMOTE_H_ */
