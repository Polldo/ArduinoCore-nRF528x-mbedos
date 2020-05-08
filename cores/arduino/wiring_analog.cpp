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

#ifdef ANALOG_TEST
/* Types used for the table below */
typedef struct _AnalogPinDescription
{
  PinName name;
  mbed::AnalogIn* adc;
} AnalogPinDescription;

static AnalogPinDescription analogPinDescription[] = 
{
    // A0 - A7
  P0_4,  NULL,     // A0
  P0_5,  NULL,     // A1
  P0_30, NULL,     // A2
  P0_29, NULL,     // A3
  P0_31, NULL,     // A4/SDA
  P0_2,  NULL,     // A5/SCL
  P0_28, NULL,     // A6
  P0_3,  NULL,     // A7
};

static nrf_saadc_channel_config_t adcCurrentConfig = {
//#define SAADC_DEFAULT_CONF = { //do this as in a define
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
#endif //ANALOG_TEST

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






#ifdef ANALOG_TEST

static AnalogPinDescription* PinNameToAnalogPinDescription(PinName P) {
  for (pin_size_t i=0; i <= NUM_ANALOG_INPUTS; i++) {
    if (analogPinDescription[i].name == P) {
      return (analogPinDescription + i);
    }
  }
  return NULL;
}

int analogRead(PinName pin)
{
  return 0;
}

int analogRead(pin_size_t pin)
{
  int multiply_factor = 1;
#ifdef ANALOG_BUG_MBED
  multiply_factor = 4;
#endif
  //return (mbed::AnalogIn(pin).read_u16() >> (16 - read_resolution)) * multiply_factor;

  auto analogPin = &analogPinDescription[pin - A0]; 
  auto pinAdc = analogPin->adc;
  if (pinAdc == NULL) {
    pinAdc = new mbed::AnalogIn(analogPin->name);
    analogPin->adc = pinAdc;
  }
  return (pinAdc->read_u16() >> (16 - read_resolution)) * multiply_factor;
}

void analogReadResolution(int bits)
{
  read_resolution = bits;
}

void analogChannelUpdate(pin_size_t pin, nrf_saadc_channel_config_t config)
{
  PinName pinName = analogPinDescription[pin].name;
  uint8_t channel = (int)pinmap_find_function(pinName, PinMap_ADC);
}

void analogReadConfigure(pin_size_t pin, nrf_saadc_channel_config_t config) 
{
  pin_size_t numPin = pin - A0;
  auto pinAdc = analogPinDescription[numPin].adc;
  if (pinAdc == NULL) {
    return;
  }

  /* Updates already configured adc */
  for (pin_size_t i = 0; i <= NUM_ANALOG_INPUTS; i++) {
    if (analogPinDescription[i].adc != NULL) {
    }
  }

  nrf_saadc_input_t input = (nrf_saadc_input_t) 9; //NRF_SAADC_INPUT_DISABLED;

  PinName pinName = analogPinDescription[numPin].name;
  uint8_t channel = (int)pinmap_find_function(pinName, PinMap_ADC);
  //Serial.println(channel);


  nrfx_saadc_channel_init(channel, &config);

  /* Store channel in ADC object. */
  //obj->channel = channel; //the channel should be already assigned in the first adc init 
}

#endif //ANALOG_TEST
