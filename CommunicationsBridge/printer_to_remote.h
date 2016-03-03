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

template <printer_message message>
struct serial_com{};

template <printer_message message>
struct serial_remote{};

template <>
struct serial_com<printer_message::REPORT_STARTUP_STATUS>{
	static void send( byte mcu ){
		MSerial.send<printer_message::REPORT_STARTUP_STATUS>( mcu );
	}
};

template <>
struct serial_remote<printer_message::REPORT_STARTUP_STATUS>{
	static std::ostream& handle_receive( std::ostream& os, byte mcu) {
		os << "start\n" << "echo:";
		if (mcu & 1) os << (MSG_POWERUP);
		if (mcu & 2) os << (MSG_EXTERNAL_RESET);
		if (mcu & 4) os << (MSG_BROWNOUT_RESET);
		if (mcu & 8) os << (MSG_WATCHDOG_RESET);
		if (mcu & 32) os << (MSG_SOFTWARE_RESET);
		return os << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_FIRMWARE_CAPABILITIES_STRING>{
	static void send( const char* capabilities_string ){
		MSerial.send<printer_message::REPORT_FIRMWARE_CAPABILITIES_STRING>(capabilities_string);
	}
};

template <>
struct serial_remote<printer_message::REPORT_FIRMWARE_CAPABILITIES_STRING>{
	static std::ostream& handle_receive(std::ostream& os, const char* capabilities_string ){
		return os << capabilities_string;
	}
};



template <>
struct serial_com<printer_message::REPORT_OK>{
	static void send() {
		MSerial.send<printer_message::REPORT_OK>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_OK>{
	static std::ostream& handle_receive( std::ostream& os ) {
		return os << MSG_OK << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_PRINTER_KILLED>{
	static void send() {
		MSerial.send<printer_message::REPORT_PRINTER_KILLED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_PRINTER_KILLED>{
	static std::ostream& handle_receive( std::ostream& os ) {
		return os << errormagic << (MSG_ERR_KILLED) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_PRINTER_STOPPED>{
	static void send() {
		MSerial.send<printer_message::REPORT_PRINTER_STOPPED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_PRINTER_STOPPED>{
	static std::ostream& handle_receive( std::ostream& os ) {
		return os << errormagic << (MSG_ERR_STOPPED) << "\n";
	}
};


template <>
struct serial_com<printer_message::REPORT_STEP_RATE_TOO_HIGH>{
	static void send(unsigned short step_rate) {
		MSerial.send<printer_message::REPORT_STEP_RATE_TOO_HIGH>(step_rate);
	}
};

template <>
struct serial_remote<printer_message::REPORT_STEP_RATE_TOO_HIGH>{
	static std::ostream& handle_receive(std::ostream& os, unsigned short step_rate) {
		return os << (MSG_STEPPER_TOO_HIGH) << step_rate << "\n";
	}
};


template <>
struct serial_com<printer_message::REPORT_COLD_EXTRUSION_PREVENTED>{
	static void send() {
		MSerial.send<printer_message::REPORT_COLD_EXTRUSION_PREVENTED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_COLD_EXTRUSION_PREVENTED>{
	static std::ostream& handle_receive( std::ostream& os) {
		return os << echomagic << (MSG_ERR_COLD_EXTRUDE_STOP) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_TOO_LONG_EXTRUSION_PREVENTED>{
	static void send() {
		MSerial.send<printer_message::REPORT_TOO_LONG_EXTRUSION_PREVENTED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_TOO_LONG_EXTRUSION_PREVENTED>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << echomagic << MSG_ERR_LONG_EXTRUDE_STOP << "\n";
	}
};


template <>
struct serial_com<printer_message::REPORT_CURRENT_PRINTER_SETTINGS>{
	static void send(
			sized_array<float> axis_step_per_unit,
			sized_array<float> max_feedrate,
			sized_array<unsigned long> max_acceleration_units_per_sq_second,
			float acceleration,
			float retract_acceleration,
			float minimumfeedrate,
			float mintravelfeedrate,
			unsigned long minsegmenttime,
			float max_xy_jerk,
			float max_z_jerk,
			float max_e_jerk,
			sized_array<float> add_homeing,
			float Kp,
			float unscalePID_i,
			float unscalePID_d
	) {
		MSerial.send<printer_message::REPORT_CURRENT_PRINTER_SETTINGS>(
				/*axis_step_per_unit,
				max_feedrate,*/
				max_acceleration_units_per_sq_second,
				acceleration,
				retract_acceleration,
				minimumfeedrate,
				mintravelfeedrate,
				minsegmenttime,
				max_xy_jerk,
				max_z_jerk,
				max_e_jerk,
				/*add_homeing,*/
				Kp,
				unscalePID_i,
				unscalePID_d );
	}
};


template <>
struct serial_remote<printer_message::REPORT_CURRENT_PRINTER_SETTINGS>{
	static std::ostream& handle_receive( std::ostream& os,
			sized_array<float> axis_steps_per_unit,
			sized_array<float> max_feedrate,
			sized_array<unsigned long> max_acceleration_units_per_sq_second,
			float acceleration,
			float retract_acceleration,
			float minimumfeedrate,
			float mintravelfeedrate,
			unsigned long minsegmenttime,
			float max_xy_jerk,
			float max_z_jerk,
			float max_e_jerk,
			sized_array<float> add_homeing,
			float Kp,
			float unscalePID_i,
			float unscalePID_d
	) {
		// Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
		os << "Steps per unit:" << "\n";
		os <<   "  M92 X" << axis_steps_per_unit.data[0] <<
				" Y" << axis_steps_per_unit.data[1] <<
				" Z" << axis_steps_per_unit.data[2] <<
				" E" << axis_steps_per_unit.data[3] << "\n";
		os << "Maximum feedrates (mm/s):\n";
		os << "  M203 X" << max_feedrate.data[0] <<
				" Y" << max_feedrate.data[1] <<
				" Z" << max_feedrate.data[2] <<
				" E" << max_feedrate.data[3] << "\n";
		os << "Maximum Acceleration (mm/s2):\n";
		os << "  M201 X" << max_acceleration_units_per_sq_second.data[0] <<
				" Y" << max_acceleration_units_per_sq_second.data[1] <<
				" Z" << max_acceleration_units_per_sq_second.data[2] <<
				" E" << max_acceleration_units_per_sq_second.data[3] << "\n";

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
		os << "  M206 X" << add_homeing.data[0] <<
				" Y" << add_homeing.data[1]<<
				" Z" << add_homeing.data[2]<< "\n";

		os << "PID settings:\n";
		os << "   M301 P" << Kp <<
				" I" << unscalePID_i <<
				" D" << unscalePID_d << "\n";
		return os;
	}
};



template <>
struct serial_com<printer_message::REPORT_SETTINGS_STORED>{
	static void send() {
		MSerial.send<printer_message::REPORT_SETTINGS_STORED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_SETTINGS_STORED>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << "Settings Stored\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_SETTINGS_RETRIEVED>{
	static void send() {
		MSerial.send<printer_message::REPORT_SETTINGS_RETRIEVED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_SETTINGS_RETRIEVED>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << "Stored settings retrieved\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_FACTORY_SETTINGS_RESTORED>{
	static void send() {
		MSerial.send<printer_message::REPORT_FACTORY_SETTINGS_RESTORED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_FACTORY_SETTINGS_RESTORED>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << "Hardcoded Default Settings Loaded\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ENQUEING_COMMAND>{
	static void send( const char* enqued_command) {
		MSerial.send<printer_message::REPORT_ENQUEING_COMMAND>( enqued_command );
	}
};

template <>
struct serial_remote<printer_message::REPORT_ENQUEING_COMMAND>{
	static std::ostream& handle_receive(std::ostream& os, const char* enqued_command) {
		return os << "enqueing \"" << enqued_command << "\"\n";
	}
};





template <>
struct serial_com<printer_message::REPORT_FIRMWARE_INFORMATION>{
	static void send(int free_memory, int planner_buffer_bytes){
		MSerial.send<printer_message::REPORT_FIRMWARE_INFORMATION>(
				free_memory,planner_buffer_bytes
		);
	}
};

template <>
struct serial_remote<printer_message::REPORT_FIRMWARE_INFORMATION>{
	static std::ostream& handle_receive(std::ostream& os, int free_memory, int planner_buffer_bytes){
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
};



template <>
struct serial_com<printer_message::REPORT_FILE_SAVED>{
	static void send() {
		MSerial.send<printer_message::REPORT_FILE_SAVED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_FILE_SAVED>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << (MSG_FILE_SAVED) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ACTIVE_EXTRUDER>{
	static void send( uint8_t extruder) {
		MSerial.send<printer_message::REPORT_ACTIVE_EXTRUDER>( extruder );
	}
};

template <>
struct serial_remote<printer_message::REPORT_ACTIVE_EXTRUDER>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t extruder) {
		return os << (MSG_ACTIVE_EXTRUDER) << extruder << "\n";
	}
};




template <>
struct serial_com<printer_message::REPORT_UNKNOWN_COMMAND>{
	static void send( const char* unknown_command) {
		MSerial.send<printer_message::REPORT_UNKNOWN_COMMAND>( unknown_command );
	}
};

template <>
struct serial_remote<printer_message::REPORT_UNKNOWN_COMMAND>{
	static std::ostream& handle_receive( std::ostream& os,const char* unknown_command) {
		return os << (MSG_UNKNOWN_COMMAND) << unknown_command << "\"\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_WRONG_EXTRUDER_SPECIFIED>{
	static void send(int16_t command_code, uint8_t extruder) {
		MSerial.send<printer_message::REPORT_WRONG_EXTRUDER_SPECIFIED>(command_code, extruder );
	}
};

template <>
struct serial_remote<printer_message::REPORT_WRONG_EXTRUDER_SPECIFIED>{
	static std::ostream& handle_receive(std::ostream& os,int command_code, uint8_t extruder) {
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
};



template <>
struct serial_com<printer_message::REPORT_READ_WRONG_EXTRUDER>{
	static void send(uint8_t extruder) {
		MSerial.send<printer_message::REPORT_READ_WRONG_EXTRUDER>(extruder);
	}
};

template <>
struct serial_remote<printer_message::REPORT_READ_WRONG_EXTRUDER>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t e) {
		return os << errormagic << +e << " - Invalid extruder number !\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ERROR_MAXTEMP>{
	static void send(uint8_t e) {
		MSerial.send<printer_message::REPORT_ERROR_MAXTEMP>(e);
	}
};

template <>
struct serial_remote<printer_message::REPORT_ERROR_MAXTEMP>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t e) {
		return os << errormagic << +e << ": Extruder switched off. MAXTEMP triggered !\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ERROR_MINTEMP>{
	static void send(uint8_t e) {
		MSerial.send<printer_message::REPORT_ERROR_MINTEMP>(e);
	}
};
template <>
struct serial_remote<printer_message::REPORT_ERROR_MINTEMP>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t e) {
		return os << errormagic << +e << ": Extruder switched off. MINTEMP triggered !\n";
	}
};




template <>
struct serial_com<printer_message::REPORT_ERROR_MAXTEMP_BED>{
	static void send() {
		MSerial.send<printer_message::REPORT_ERROR_MAXTEMP_BED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_ERROR_MAXTEMP_BED>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << errormagic << "Temperature heated bed switched off. MAXTEMP triggered !!\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_INVALID_EXTRUDER_SPECIFIED>{
	static void send( uint8_t extruder) {
		MSerial.send<printer_message::REPORT_INVALID_EXTRUDER_SPECIFIED>(extruder);
	}
};

template <>
struct serial_remote<printer_message::REPORT_INVALID_EXTRUDER_SPECIFIED>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t extruder) {
		return os << "T" << extruder << " " << (MSG_INVALID_EXTRUDER) << "\n";
	}
};




template <>
struct serial_com<printer_message::ISSUE_RESEND_REQUEST>{
	static void send(long LastN) {
		//TODO::Should this be here: MYSERIAL.flush();
		MSerial.send<printer_message::ISSUE_RESEND_REQUEST>(LastN+1);
	}
};

template <>
struct serial_remote<printer_message::ISSUE_RESEND_REQUEST>{
	static std::ostream& handle_receive(std::ostream& os, long LastN) {
		return os << (MSG_RESEND) << LastN << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ERROR_IN_LINE_NUMBER>{
	static void send(long LastN) {
		MSerial.send<printer_message::REPORT_ERROR_IN_LINE_NUMBER>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_ERROR_IN_LINE_NUMBER>{
	static std::ostream& handle_receive(std::ostream& os, long LastN) {
		return os << (MSG_ERR_LINE_NO) << (LastN) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ERROR_IN_CHECKSUM>{
	static void send(long LastN) {
		MSerial.send<printer_message::REPORT_ERROR_IN_CHECKSUM>(LastN);
	}
};

template <>
struct serial_remote<printer_message::REPORT_ERROR_IN_CHECKSUM>{
	static std::ostream& handle_receive(std::ostream& os, long LastN) {
		return os << (MSG_ERR_CHECKSUM_MISMATCH) << (LastN) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_MISSING_CHECKSUM>{
	static void send(long LastN) {
		MSerial.send<printer_message::REPORT_MISSING_CHECKSUM>(LastN);
	}
};

template <>
struct serial_remote<printer_message::REPORT_MISSING_CHECKSUM>{
	static std::ostream& handle_receive(std::ostream& os, long LastN) {
		return os << (MSG_ERR_NO_CHECKSUM) << (LastN) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_NO_LINE_NUMBER_WITH_CHECKSUM>{
	static void send(long LastN) {
		MSerial.send<printer_message::REPORT_NO_LINE_NUMBER_WITH_CHECKSUM>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_NO_LINE_NUMBER_WITH_CHECKSUM>{
	static std::ostream& handle_receive(std::ostream& os, long LastN) {
		return os << (MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM) << (LastN) << "\n";
	}
};


template <>
struct serial_com<printer_message::REPORT_TIME_ELAPSED>{
	static void send(unsigned long time_seconds ) {
		MSerial.send<printer_message::REPORT_TIME_ELAPSED>(time_seconds);
	}
};

template <>
struct serial_remote<printer_message::REPORT_TIME_ELAPSED>{
	static std::ostream& handle_receive( std::ostream& os, unsigned long time_seconds ) {
		return os << (time_seconds) << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_FILE_PRINTED>{
	static void send( unsigned long time_seconds ) {
		MSerial.send<printer_message::REPORT_FILE_PRINTED>(time_seconds);
	}
};

template <>
struct serial_remote<printer_message::REPORT_FILE_PRINTED>{
	static std::ostream& handle_receive(std::ostream& os, unsigned long time_seconds ) {
		int min = time_seconds / 60;
		int sec = time_seconds % 60;
		return os << (MSG_FILE_PRINTED) << min << " min, " << sec << " sec\n";
	}
};


template <>
struct serial_com<printer_message::REPORT_ENDSTOP_NOT_PRESSED_AFTER_HOMING>{
	static void send() {
		MSerial.send<printer_message::REPORT_ENDSTOP_NOT_PRESSED_AFTER_HOMING>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_ENDSTOP_NOT_PRESSED_AFTER_HOMING>{
	static std::ostream& handle_receive(std::ostream& os) {
		return os << "Endstop not pressed after homing down. Endstop broken?\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_ERROR_ENDSTOP_STILL_PRESSED>{
	static void send() {
		MSerial.send<printer_message::REPORT_ERROR_ENDSTOP_STILL_PRESSED>();
	}
};

template <>
struct serial_remote<printer_message::REPORT_ERROR_ENDSTOP_STILL_PRESSED>{
	static std::ostream& handle_receive( std::ostream& os ){
		return os << "Endstop still pressed after backing off. Endstop stuck?\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_TEMPERATURE_AND_POWER>{
	static void send(uint8_t extruder, float deg_hotend, float target_hotend, float deg_bed, float target_bed, uint8_t heater_power, uint8_t bed_power) {
		MSerial.send<printer_message::REPORT_TEMPERATURE_AND_POWER>(extruder, deg_hotend, target_hotend, deg_bed, target_bed, heater_power, bed_power);
	}
};

template <>
struct serial_remote<printer_message::REPORT_TEMPERATURE_AND_POWER>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t extruder, float deg_hotend, float target_hotend, float deg_bed, float target_bed, uint8_t heater_power, uint8_t bed_power) {
		return os << "ok T" << extruder << ":" << deg_hotend << " /" << target_hotend <<
				" B:" << deg_bed << " /" << target_bed <<
				" @:" << heater_power << " B@:" << bed_power << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_WAIT_FOR_TEMPERATURE>{
	static void send( uint8_t extruder, float deg_hotend, char residency, int16_t time_left_seconds ){
		MSerial.send<printer_message::REPORT_WAIT_FOR_TEMPERATURE>(extruder, deg_hotend,residency, time_left_seconds );
	}
};

template <>
struct serial_remote<printer_message::REPORT_WAIT_FOR_TEMPERATURE>{
	static std::ostream& handle_receive( std::ostream& os, uint8_t extruder, float deg_hotend, char residency, int16_t time_left_seconds ){
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
};



template <>
struct serial_com<printer_message::REPORT_WAIT_FOR_TEMPERATURE_BED>{
	static void send(uint8_t extruder, float deg_hotend, float deg_bed) {
		MSerial.send<printer_message::REPORT_WAIT_FOR_TEMPERATURE_BED>( extruder, deg_hotend, deg_bed);
	}
};

template <>
struct serial_remote<printer_message::REPORT_WAIT_FOR_TEMPERATURE_BED>{
	static std::ostream& handle_receive(std::ostream& os, uint8_t extruder, float deg_hotend, float deg_bed) {
		return os << "T:" << deg_hotend << " E:" << extruder << " B:" << deg_bed << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_LCD_BUTTON_INFO>{
	static void send(int16_t encoder_pos, uint8_t button_down) {
		MSerial.send<printer_message::REPORT_LCD_BUTTON_INFO>( encoder_pos, button_down );
	}
};

template <>
struct serial_remote<printer_message::REPORT_LCD_BUTTON_INFO>{
	static std::ostream& handle_receive(std::ostream& os, int16_t encoder_pos, uint8_t button_down) {
		return os << "ok R:" << encoder_pos << " B:" << button_down << "\n";
	}
};



template <>
struct serial_com<printer_message::REPORT_BED_LEVELING_PROBE_SEQUENCE>{
	static void send(float height_1, float height_2,
			float height_3, float bed_leveling_factor_x, float bed_leveling_factor_y) {
		MSerial.send<printer_message::REPORT_BED_LEVELING_PROBE_SEQUENCE>(
				height_1,
				height_2,
				height_3,
				bed_leveling_factor_x,
				bed_leveling_factor_y);
	}
};

template <>
struct serial_remote<printer_message::REPORT_BED_LEVELING_PROBE_SEQUENCE>{
	static std::ostream& handle_receive( std::ostream& os, float height_1, float height_2,
			float height_3, float bed_leveling_factor_x, float bed_leveling_factor_y) {
		os <<
				height_1 << "\n" <<
				height_2 << "\n" <<
				height_3 << "\n" <<
				bed_leveling_factor_x << "\n" <<
				bed_leveling_factor_y << "\n";
		return os;
	}
};


template <>
struct serial_com<printer_message::REPORT_BED_LEVELING_PROBE_POINT>{
	static void send(float dest_z) {
		MSerial.send<printer_message::REPORT_BED_LEVELING_PROBE_POINT>( dest_z );
	}
};

template <>
struct serial_remote<printer_message::REPORT_BED_LEVELING_PROBE_POINT>{
	static std::ostream& handle_receive( std::ostream& os, float dest_z ) {
		return os << dest_z << "\n";
	}
};


template <>
struct serial_com<printer_message::REPORT_SINGLE_CAP_PROBE_VALUE>{
	static void send(uint16_t value) {
		MSerial.send<printer_message::REPORT_SINGLE_CAP_PROBE_VALUE>( value );
	}
};

template <>
struct serial_remote<printer_message::REPORT_SINGLE_CAP_PROBE_VALUE>{
	static std::ostream& handle_receive( std::ostream& os, uint16_t value) {
#ifdef REMOTE_SERIAL
		auto flags = os.flags();
		os << std::hex << value << "\n";
		os.flags(flags);
#endif
		return os;
	}
};


template <>
struct serial_com<printer_message::REPORT_MICROSTEP_SETTINGS>{
	static void send(
			int ms_X1, int ms_X2, int ms_Y1, int ms_Y2, int ms_Z1, int ms_Z2,
			int ms_E0_1, int ms_E0_2, int ms_E1_1, int ms_E1_2
	){
		MSerial.send<printer_message::REPORT_MICROSTEP_SETTINGS>(
				ms_X1, ms_X2, ms_Y1, ms_Y2, ms_Z1, ms_Z2,
				ms_E0_1, ms_E0_2, ms_E1_1, ms_E1_2);
	}
};

template <>
struct serial_remote<printer_message::REPORT_MICROSTEP_SETTINGS>{
	static std::ostream& handle_receive(
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
};


enum class endstop_status: int8_t {
	UNUSED = -1,
			UNTRIGGERD = 0,
			TRIGGERED = 1,
};
template <>
struct serial_com<printer_message::REPORT_ENDSTOP_STATUS>{
	static void send(
			endstop_status x_min, endstop_status x_max,
			endstop_status y_min, endstop_status y_max,
			endstop_status z_min, endstop_status z_max
	){
		MSerial.send<printer_message::REPORT_ENDSTOP_STATUS>(
				x_min, x_max, y_min, y_max, z_min, z_max );
	}
};

template <>
struct serial_remote<printer_message::REPORT_ENDSTOP_STATUS>{
	static std::ostream& handle_receive(
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
};

template <>
struct serial_com<printer_message::REPORT_ENDSTOP_HIT>{
	static void send(Axes axis, float position){
		MSerial.send<printer_message::REPORT_ENDSTOP_HIT>(axis,position);
	}
};

template <>
struct serial_remote<printer_message::REPORT_ENDSTOP_HIT>{
	static std::ostream& handle_receive(std::ostream& os, Axes axis, float position){
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
};


template <>
struct serial_com<printer_message::REPORT_CURRENT_POSITION>{
	static void send(
			float i_x, float i_y, float i_z, float i_e,
			float j_x, float j_y, float j_z
	){
		MSerial.send<printer_message::REPORT_CURRENT_POSITION>(
				i_x, i_y,i_z,i_e, j_x, j_y, j_z );
	}
};

template <>
struct serial_remote<printer_message::REPORT_CURRENT_POSITION>{
	static std::ostream& handle_receive(
			std::ostream& os, float i_x, float i_y, float i_z, float i_e,
			float j_x, float j_y, float j_z
	){
		os <<
				"X:" << i_x << "Y:" << i_y << "Z:" << i_z << "E:" << i_e <<
				(MSG_COUNT_X) << j_x << "Y:" << j_y << "Z:" << j_z << "\n";
		return os;
	}
};



template <>
struct serial_com<printer_message::REPORT_PID_EXTRUDER_PARAMETERS>{
	static void send(
			float kp, float unscaled_ki,
			float unscaled_kd, float kc
	){
		MSerial.send<printer_message::REPORT_PID_EXTRUDER_PARAMETERS>(
				kp, unscaled_ki, unscaled_kd, kc );
	}
};

template <>
struct serial_remote<printer_message::REPORT_PID_EXTRUDER_PARAMETERS>{
	static std::ostream& handle_receive(
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
};




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
