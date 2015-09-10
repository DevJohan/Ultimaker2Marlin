// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define VERSION_STRING  "1.0.0"

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>


#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#ifndef AT90USB
#define  HardwareSerial_h // trick to disable the standard HWserial
#endif

#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
  //Arduino < 1.0.0 does not define this, so we need to do it ourselfs
# define analogInputToDigitalPin(p) ((p) + A0)
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

class critical_section_guard{
critical_section_guard(const critical_section_guard&);
critical_section_guard& operator = (const critical_section_guard&);
public:
critical_section_guard():_sreg(SREG) { cli(); }
~critical_section_guard(){ SREG = _sreg; }
private:
	unsigned char _sreg;
};

#include "../CommunicationsBridge/printer_to_remote.h"

#include "WString.h"






void get_command();
void process_commands();

void manage_inactivity();

#if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
  #define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
  #define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
  #define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
  #define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { WRITE(Z_ENABLE_PIN, Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN, Z_ENABLE_ON); }
    #define disable_z() { WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON); WRITE(Z2_ENABLE_PIN,!Z_ENABLE_ON); }
  #else
    #define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
    #define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if defined(E0_ENABLE_PIN) && (E0_ENABLE_PIN > -1)
  #define enable_e0() WRITE(E0_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e0() WRITE(E0_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (EXTRUDERS > 1) && defined(E1_ENABLE_PIN) && (E1_ENABLE_PIN > -1)
  #define enable_e1() WRITE(E1_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e1() WRITE(E1_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (EXTRUDERS > 2) && defined(E2_ENABLE_PIN) && (E2_ENABLE_PIN > -1)
  #define enable_e2() WRITE(E2_ENABLE_PIN, E_ENABLE_ON)
  #define disable_e2() WRITE(E2_ENABLE_PIN,!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif


enum class Axes: uint8_t { X, Y, Z, E};

constexpr uint8_t to_index(Axes axis){
	return static_cast<uint8_t>(axis);
}
constexpr uint8_t axis_mask(Axes axis){
	return (1<<to_index(axis));
}
static_assert(to_index(Axes::X) == 0u, "Axes::X value not as expected");
static_assert(to_index(Axes::Y) == 1u, "Axes::Y value not as expected");
static_assert(to_index(Axes::Z) == 2u, "Axes::Z value not as expected");
static_assert(to_index(Axes::E) == 3u, "Axes::E value not as expected");

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
#ifdef DELTA
void calculate_delta(float cartesian[3]);
#endif
void prepare_move();
void kill();
#define STOP_REASON_MAXTEMP              1
#define STOP_REASON_MINTEMP              2
#define STOP_REASON_MAXTEMP_BED          3
#define STOP_REASON_HEATER_ERROR         4
#define STOP_REASON_Z_ENDSTOP_BROKEN_ERROR 5
#define STOP_REASON_Z_ENDSTOP_STUCK_ERROR  6
#define STOP_REASON_XY_ENDSTOP_BROKEN_ERROR 7
#define STOP_REASON_XY_ENDSTOP_STUCK_ERROR  8
#define STOP_REASON_SAFETY_TRIGGER       10
void Stop(uint8_t reasonNr);

bool IsStopped();
uint8_t StoppedReason();

void clear_command_queue();
void enquecommand(const char *cmd); //put an ascii command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ascii command at the end of the current buffer, read from flash
bool is_command_queued();
uint8_t commands_queued();
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

#ifdef FAST_PWM_FAN
void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply[EXTRUDERS]; // Sets extrude multiply factor (in percent)
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern uint8_t fanSpeed;
extern uint8_t fanSpeedPercent;
#ifdef BARICUDA
extern int ValvePressure;
extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
extern unsigned char fanSpeedSoftPwm;
#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_feedrate, retract_zlift;
#if EXTRUDERS > 1
extern float extruder_swap_retract_length;
#endif
extern float retract_recover_length, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

//The printing state from the main command processor. Is not zero when the command processor is in a loop waiting for a result.
enum class PRINT_STATE: uint8_t {
NORMAL      = 0,
DWELL       = 1,
WAIT_USER   = 2,
HEATING     = 3,
HEATING_BED = 4,
HOMING      = 5
};

extern PRINT_STATE printing_state;

// Handling multiple extruders pins
extern uint8_t active_extruder;

#if EXTRUDERS > 3
  # error Unsupported number of extruders
#elif EXTRUDERS > 2
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2, v3 }
#elif EXTRUDERS > 1
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1, v2 }
#else
  # define ARRAY_BY_EXTRUDERS(v1, v2, v3) { v1 }
#endif

#endif
