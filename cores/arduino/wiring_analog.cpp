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

static inline void analogConfigure(pin_size_t pin);
static void analogUpdate();

static bool adcConfigChanged = false;
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
  Serial.println("pin");
  for (pin_size_t i = 0; i <= NUM_ANALOG_INPUTS; i++) {
    if (analogPinToPinName(i) == pin) {
      return analogRead(i + A0);
    }
  }
  return -1;
}

int analogRead(pin_size_t pin)
{
  int multiply_factor = 1;
#ifdef ANALOG_BUG_MBED
  multiply_factor = 4;
#endif
  mbed::AnalogIn* adc = analogPinToAdcObj(pin);
  if (adc == NULL) {
    adc = new mbed::AnalogIn(analogPinToPinName(pin));
    Serial.println("new adc");
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
  if (mode == VDD) {
    reference = NRF_SAADC_REFERENCE_VDD4;
    gain = NRF_SAADC_GAIN1_4;
  } else {
    reference = NRF_SAADC_REFERENCE_INTERNAL;
    gain = NRF_SAADC_GAIN1;
  }
  adcCurrentConfig.reference = reference;
  adcCurrentConfig.gain = gain;
  analogUpdate();
}

static inline void analogConfigure(pin_size_t pin)
{
  PinName pinName = analogPinToPinName(pin);
  uint8_t channel = (int)pinmap_find_function(pinName, PinMap_ADC);
  nrf_saadc_input_t input = (nrf_saadc_input_t) (channel + 1);
  adcCurrentConfig.pin_p = input;
  nrfx_saadc_channel_init(channel, &adcCurrentConfig);
}

static void analogUpdate() 
{
  /* Check if AnalogIn was initialized 
  pin_size_t numPin = pin - A0;
  auto pinAdc = analogPinDescription[numPin].adc;
  if (pinAdc == NULL) {
    return;
  }
  */
  adcConfigChanged = true;
  /* Updates already configured adc */
  //for (pin_size_t i = A0; i <= A0 + NUM_ANALOG_INPUTS; i++) {  //also the other works
  for (pin_size_t i = 0; i <= NUM_ANALOG_INPUTS; i++) {
    if (analogPinToAdcObj(i) != NULL) {
      analogConfigure(i);
    }
  }
  //obj->channel = channel; //the channel should be already assigned in the first adc init 
  // Also this init does not use the mbed mutex --> CRITICAL !
  // These are two weakness of this method. To solve them the mbed library should be modified.
}

