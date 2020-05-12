/*
  wiring_analog.cpp - analog input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2018-2019 Arduino SA

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "Arduino.h"
#include "pins_arduino.h"
#include "nrfx_saadc.h"
#include "PeripheralPins.h"

static int write_resolution = 8;
static int read_resolution = 10;

/* Flag to indicate wheter the config has been changed from the default one */
static bool adcConfigChanged = false;

/* 
 * Configuration used for all the active ADC channels, it is initialized with the mbed default values
 * When it is changed, all the ADC channels are reconfigured accordingly 
 */
static nrf_saadc_channel_config_t adcCurrentConfig = {
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain       = NRF_SAADC_GAIN1_4,
    .reference  = NRF_SAADC_REFERENCE_VDD4,
    .acq_time   = NRF_SAADC_ACQTIME_10US,
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
    .burst      = NRF_SAADC_BURST_DISABLED,
    .pin_p      = NRF_SAADC_INPUT_DISABLED,
    .pin_n      = NRF_SAADC_INPUT_DISABLED
};

/* Private function to reconfigure already active ADC channels */
static inline void analogConfigure(pin_size_t pin);
static void analogUpdate();

#ifdef digitalPinToPwmObj
static mbed::PwmOut* PinNameToPwmObj(PinName P) {
  // reverse search for pinName in g_APinDescription[P].name fields
  for (pin_size_t i=0; i < PINS_COUNT; i++) {
    if (g_APinDescription[i].name == P) {
      return g_APinDescription[i].pwm;
    }
  }
  return NULL;
}
#endif

void analogWrite(PinName pin, int val)
{
  float percent = (float)val/(float)(1 << write_resolution);
#ifdef digitalPinToPwmObj
  mbed::PwmOut* pwm = PinNameToPwmObj(pin);
  if (pwm == NULL) {
    pwm = new mbed::PwmOut(pin);
    digitalPinToPwmObj(pin) = pwm;
    pwm->period_ms(2); //500Hz
  }
#else
  // attention: this leaks badly
  mbed::PwmOut* pwm = new mbed::PwmOut(digitalPinToPinName(pin));
#endif
  pwm->write(percent);
}

void analogWrite(pin_size_t pin, int val)
{
  float percent = (float)val/(float)(1 << write_resolution);
#ifdef digitalPinToPwmObj
  mbed::PwmOut* pwm = digitalPinToPwmObj(pin);
  if (pwm == NULL) {
    pwm = new mbed::PwmOut(digitalPinToPinName(pin));
    digitalPinToPwmObj(pin) = pwm;
    pwm->period_ms(2); //500Hz
  }
  pwm->write(percent);
#endif
}

void analogWriteResolution(int bits)
{
  write_resolution = bits;
}

int analogRead(PinName pin)
{
  for (pin_size_t i = 0; i <= NUM_ANALOG_INPUTS; i++) {
    if (analogPinToPinName(i) == pin) {
      return analogRead(i + A0);
    }
  }
  return -1;
}

/* Weaknesses of this method:
 * - ADC channels should be firstly init with mbed functions and then can be reconfigured. 
 * - ADC reconfiguration does not use the mbed mutex --> could be CRITICAL 
 * To solve them, the mbed library should be modified. */
int analogRead(pin_size_t pin)
{
  int multiply_factor = 1;
#ifdef ANALOG_BUG_MBED
  multiply_factor = 4;
#endif
  mbed::AnalogIn* adc = analogPinToAdcObj(pin);
  if (adc == NULL) {
    adc = new mbed::AnalogIn(analogPinToPinName(pin));
    analogPinToAdcObj(pin) = adc;
    if (adcConfigChanged) {
      analogConfigure(pin);
    }
  }
  return (adc->read_u16() >> (16 - read_resolution)) * multiply_factor;
}

void analogReadResolution(int bits)
{
  read_resolution = bits;
}

void analogReference(uint8_t mode)
{
  nrf_saadc_reference_t reference;
  nrf_saadc_gain_t gain;
  if (mode == AR_VDD) {
    reference = NRF_SAADC_REFERENCE_VDD4;
    gain = NRF_SAADC_GAIN1_4;
  } else if (mode == AR_INTERNAL) {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1;
  } else if (mode == AR_INTERNAL1V2) {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1_2;
  } else if (mode == AR_INTERNAL2V4) {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1_4;
  }
  adcCurrentConfig.reference = reference;
  adcCurrentConfig.gain = gain;
  analogUpdate();
}

void analogAcquisitionTime(uint8_t time)
{
  nrf_saadc_acqtime_t acqTime;
  if (time == AT_3_US) {
    acqTime = NRF_SAADC_ACQTIME_3US;
  } else if (time == AT_5_US) {
    acqTime = NRF_SAADC_ACQTIME_5US;
  } else if (time == AT_10_US) {
    acqTime = NRF_SAADC_ACQTIME_10US;
  } else if (time == AT_15_US) {
    acqTime = NRF_SAADC_ACQTIME_15US;
  } else if (time == AT_20_US) {
    acqTime = NRF_SAADC_ACQTIME_20US;
  } else if (time == AT_40_US) {
    acqTime = NRF_SAADC_ACQTIME_40US;
  }
  adcCurrentConfig.acq_time = acqTime;
  analogUpdate();
}

/*
 * Bypass mbed and use directly Nordic libraries in order to reinitialize 
 * the channel associated to the selected pin with a custom configuration 
 */
static inline void analogConfigure(pin_size_t pin)
{
  PinName pinName = analogPinToPinName(pin);
  uint8_t channel = (int)pinmap_find_function(pinName, PinMap_ADC);
  nrf_saadc_input_t input = (nrf_saadc_input_t) (channel + 1);
  adcCurrentConfig.pin_p = input;
  nrfx_saadc_channel_init(channel, &adcCurrentConfig);
}

/* Spot all active ADCs to reconfigure them */
static void analogUpdate() 
{
  adcConfigChanged = true;
  //for (pin_size_t i = A0; i <= A0 + NUM_ANALOG_INPUTS; i++) {  //also the other works
  for (pin_size_t i = 0; i <= NUM_ANALOG_INPUTS; i++) {
    if (analogPinToAdcObj(i) != NULL) {
      analogConfigure(i);
    }
  }
}

