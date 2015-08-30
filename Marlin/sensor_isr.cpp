/*
 * sensor_isr.cpp
 *
 *  Created on: 1 feb 2015
 *      Author: johan
 */

#include "Marlin.h"
#include "temperature.h"
#include "UltiLCD2.h"


// Timer 0 is shared with millies
ISR(TIMER0_COMPB_vect)
{
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned long raw_temp_1_value = 0;
  static unsigned long raw_temp_2_value = 0;
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char temp_state = 5;

  manage_temperature();


  switch(temp_state) {
    case 1: // Measure TEMP_0
      #ifdef HEATER_0_USES_ADS101X
      #elif defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        raw_temp_0_value += ADC;
      #endif
      // Prepare TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 2;
      break;
    case 2: // Measure TEMP_BED
      #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
        raw_temp_bed_value += ADC;
      #endif
      // Prepare TEMP_1
      #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1) && EXTRUDERS > 1
        #if TEMP_1_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 3;
      break;
    case 3: // Measure TEMP_1
      #ifdef HEATER_1_USES_ADS101X
      #elif defined(TEMP_1_PIN) && (TEMP_1_PIN > -1) && EXTRUDERS > 1
        raw_temp_1_value += ADC;
      #endif
      // Prepare TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
        #if TEMP_2_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 4;
      break;
    case 4: // Measure TEMP_2
      #if defined(TEMP_2_PIN) && (TEMP_2_PIN > -1) && EXTRUDERS > 2
        raw_temp_2_value += ADC;
      #endif
      temp_count++;
      //Fall trough to state 0
    case 0: // Prepare TEMP_0
      #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1<<MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1<<ADSC; // Start conversion
      #endif
      lcd_buttons_update();
      temp_state = 1;
      break;
    case 5: //Startup, delay initial temp reading a tiny bit so the hardware can settle.
      temp_state = 0;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  }

  if(temp_count >= OVERSAMPLENR) // 8 ms * 16 = 128ms.
  {
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
#ifndef HEATER_0_USES_MAX6675
      current_temperature_raw[0] = raw_temp_0_value;
#endif
#if EXTRUDERS > 1
      current_temperature_raw[1] = raw_temp_1_value;
#endif
#ifdef TEMP_SENSOR_1_AS_REDUNDANT
      redundant_temperature_raw = raw_temp_1_value;
#endif
#if EXTRUDERS > 2
      current_temperature_raw[2] = raw_temp_2_value;
#endif
      current_temperature_bed_raw = raw_temp_bed_value;
    }

    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
    raw_temp_1_value = 0;
    raw_temp_2_value = 0;
    raw_temp_bed_value = 0;
    #if defined(HEATER_0_USES_ADS101X) || defined(HEATER_1_USES_ADS101X) || defined(HEATER_2_USES_ADS101X) || defined(BED_USES_ADS101X)
    ads101x_state = 0;
    #endif

#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw[0] <= maxttemp_raw[0]) {
#else
    if(current_temperature_raw[0] >= maxttemp_raw[0]) {
#endif
        temp_error_cnt[0] += 10;
        if (temp_error_cnt[0] > 20)
            max_temp_error(0);
    }
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    else if(current_temperature_raw[0] >= minttemp_raw[0]) {
#else
    else if(current_temperature_raw[0] <= minttemp_raw[0]) {
#endif
        temp_error_cnt[0] += 10;
        if (temp_error_cnt[0] > 20)
            min_temp_error(0);
    }else if (temp_error_cnt[0] > 0)
    {
        temp_error_cnt[0]--;
    }
#if EXTRUDERS > 1
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
    if(current_temperature_raw[1] <= maxttemp_raw[1]) {
#else
    if(current_temperature_raw[1] >= maxttemp_raw[1]) {
#endif
        temp_error_cnt[1] += 10;
        if (temp_error_cnt[1] > 20)
            max_temp_error(1);
    }
#if HEATER_1_RAW_LO_TEMP > HEATER_1_RAW_HI_TEMP
    if(current_temperature_raw[1] >= minttemp_raw[1]) {
#else
    if(current_temperature_raw[1] <= minttemp_raw[1]) {
#endif
        temp_error_cnt[1] += 10;
        if (temp_error_cnt[1] > 20)
            min_temp_error(1);
    }else if (temp_error_cnt[1] > 0)
    {
        temp_error_cnt[1]--;
    }
#endif
#if EXTRUDERS > 2
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
    if(current_temperature_raw[2] <= maxttemp_raw[2]) {
#else
    if(current_temperature_raw[2] >= maxttemp_raw[2]) {
#endif
        temp_error_cnt[2] += 10;
        if (temp_error_cnt[2] > 20)
            max_temp_error(2);
    }
#if HEATER_2_RAW_LO_TEMP > HEATER_2_RAW_HI_TEMP
    else if(current_temperature_raw[2] >= minttemp_raw[2]) {
#else
    else if(current_temperature_raw[2] <= minttemp_raw[2]) {
#endif
        temp_error_cnt[2] += 10;
        if (temp_error_cnt[2] > 20)
            min_temp_error(2);
    }else if (temp_error_cnt[2] > 0)
    {
        temp_error_cnt[2]--;
    }
#endif

  /* No bed MINTEMP error? */
#if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
    if(current_temperature_bed_raw <= bed_maxttemp_raw) {
#else
    if(current_temperature_bed_raw >= bed_maxttemp_raw) {
#endif
        temp_error_bed_cnt += 10;
        if (temp_error_bed_cnt > 20)
        {
            target_temperature_bed = 0;
            bed_max_temp_error();
        }
    }else if (temp_error_bed_cnt > 0) {
        temp_error_bed_cnt--;
    }
#endif
  }
}
