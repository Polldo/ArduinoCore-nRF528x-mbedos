//not working refactored
#include "Arduino.h"

#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_ppi.h>
#include "nrfx_gpiote.h"
#include "nrfx_ppi.h"

/* Hot encoded peripherals. Could be chosen with a more clever strategy */
#define TEST_TIMER (NRF_TIMER0)
#define TIMEOUT_TIMER (NRF_TIMER3)
#define WAIT_CHANNEL (NRF_TIMER_CC_CHANNEL2)
#define WAIT_TASK_CAPTURE (NRF_TIMER_TASK_CAPTURE2)

#define TEMP_TIMER (NRF_TIMER2)
#define TIMER_STARTED_CHANNEL (NRF_TIMER_CC_CHANNEL0)
#define TIMER_FIRST_CHANNEL (NRF_TIMER_CC_CHANNEL1)
#define TIMER_SECOND_CHANNEL (NRF_TIMER_CC_CHANNEL2)
#define TIMER_STARTED_CAPTURE (NRF_TIMER_TASK_CAPTURE0)
#define TIMER_FIRST_CAPTURE (NRF_TIMER_TASK_CAPTURE1)
#define TIMER_SECOND_CAPTURE (NRF_TIMER_TASK_CAPTURE2)

#define TIMEOUT_US (0)
#define ERROR_US (0xFFFFFFFF)

//#define MEASURE_CRITICAL

/* Define again this structure to make it working with g++ comp 
#define NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(hi_accu) \
    {                                               \
        .sense = NRF_GPIOTE_POLARITY_TOGGLE,        \
        .pull = NRF_GPIO_PIN_NOPULL,                \
        .is_watcher = false,                        \
        .hi_accuracy = hi_accu,                     \
        .skip_gpio_setup = false                    \
    }
*/

static nrfx_gpiote_in_config_t cfg = 
    {
        .sense = NRF_GPIOTE_POLARITY_TOGGLE,        
        .pull = NRF_GPIO_PIN_NOPULL,                
        .is_watcher = false,                       
        .hi_accuracy = true,                     
        .skip_gpio_setup = true//false                        
    };

static uint8_t measurePulse(PinName pin, PinStatus state, nrf_ppi_channel_group_t firstGroup) 
{
    uint32_t firstState, secondState, thirdState; 
    core_util_critical_section_enter();
    firstState = nrf_gpio_pin_read(pin);
    nrf_ppi_group_enable(firstGroup);
    secondState = nrf_gpio_pin_read(pin);
    __NOP();
    thirdState = nrf_gpio_pin_read(pin);
    core_util_critical_section_exit();
    uint8_t pulseToTake = 0;
    if (firstState == secondState && firstState == thirdState) {
        if (firstState != state) {
            pulseToTake = 1;
        } else {
            pulseToTake = 2;
        }
    } else {
        pulseToTake = TIMEOUT_US;
    }
    return pulseToTake;
}

//CONTROLLA effetti di pulsein sul pin considerato. posso usarlo come niente fosse dopo? eventuali interrupt vengono tolti?
// prova ad utilizzare altri pin in input e output sia prima che dopo l'utilizzo di questa funzione
//FIGHISSIMO in questo modo si può implementare una metodo per contare tempo di esecuzione di blocchi di istruzioni.
//unsigned long pulseIn(PinName pin, PinStatus state, unsigned long timeout)
unsigned long pulseIn(PinName pin, PinStatus state, unsigned long timeout)
{
    /* Configure timer */
    nrf_timer_mode_set(TEMP_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_STOP);
    //nrf_timer_frequency_set(TEMP_TIMER, NRF_TIMER_FREQ_16MHz); 
    nrf_timer_frequency_set(TEMP_TIMER, NRF_TIMER_FREQ_1MHz); 
    nrf_timer_bit_width_set(TEMP_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_cc_write(TEMP_TIMER, TIMER_FIRST_CHANNEL, 0);
    nrf_timer_cc_write(TEMP_TIMER, TIMER_SECOND_CHANNEL, 0);
    //nrf_timer_cc_write(TEMP_TIMER, TIMER_STARTED_CHANNEL, 100);
    //nrf_timer_cc_write(TEMP_TIMER, TIMER_STARTED_CHANNEL, 0);
    nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_CLEAR);
    /* Configure pin Toggle Event */
    nrfx_gpiote_in_init(pin, &cfg, NULL);
    nrfx_gpiote_in_event_enable(pin, true); 
    


    /* Configure timer */
    /*
    nrf_timer_mode_set(TEST_TIMER, NRF_TIMER_MODE_COUNTER);
    nrf_timer_bit_width_set(TEST_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_task_trigger(TEST_TIMER, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(TEST_TIMER, NRF_TIMER_TASK_START);

    nrf_timer_cc_write(TEST_TIMER, NRF_TIMER_CC_CHANNEL0, 0);
    nrf_timer_task_trigger(TEST_TIMER, NRF_TIMER_TASK_COUNT);
*/




    /* Allocate PPI channels for starting and stopping the timer */
    nrf_ppi_channel_t firstPPIchannel, firstPPIchannelControl;
    nrf_ppi_channel_t secondPPIchannel, secondPPIchannelControl;
    nrf_ppi_channel_t thirdPPIchannel, thirdPPIchannelControl;
    nrfx_ppi_channel_alloc(&firstPPIchannel);
    nrfx_ppi_channel_alloc(&firstPPIchannelControl);
    nrfx_ppi_channel_alloc(&secondPPIchannel);
    nrfx_ppi_channel_alloc(&secondPPIchannelControl);
    nrfx_ppi_channel_alloc(&thirdPPIchannel);
    nrfx_ppi_channel_alloc(&thirdPPIchannelControl);
    /* Allocate PPI Group channels to allow activation and deactivation of channels as PPI tasks */
    nrf_ppi_channel_group_t firstGroup, secondGroup, thirdGroup;
    nrfx_ppi_group_alloc(&firstGroup);
    nrfx_ppi_group_alloc(&secondGroup);
    nrfx_ppi_group_alloc(&thirdGroup);

    /* Insert channels in corresponding group */
    nrfx_ppi_channel_include_in_group(firstPPIchannel, firstGroup);
    nrfx_ppi_channel_include_in_group(firstPPIchannelControl, firstGroup);
    nrfx_ppi_channel_include_in_group(secondPPIchannel, secondGroup);
    nrfx_ppi_channel_include_in_group(secondPPIchannelControl, secondGroup);
    nrfx_ppi_channel_include_in_group(thirdPPIchannel, thirdGroup);
    nrfx_ppi_channel_include_in_group(thirdPPIchannelControl, thirdGroup);

    /* Configure PPI channels for Start and Stop events */
    nrf_ppi_channel_and_fork_endpoint_setup(firstPPIchannel,
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    //(uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_STARTED_CAPTURE),
                                    //(uint32_t) nrf_timer_task_address_get(TIMEOUT_TIMER, NRF_TIMER_TASK_STOP),
                                    //(uint32_t) nrf_timer_task_address_get(TEST_TIMER, NRF_TIMER_TASK_COUNT),
                                    (uint32_t) nrf_timer_task_address_get(TEST_TIMER, NRF_TIMER_TASK_CAPTURE0),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_START));
                                    //(uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_START),
                                    //(uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_STARTED_CAPTURE));
    nrf_ppi_channel_and_fork_endpoint_setup(firstPPIchannelControl,
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrfx_ppi_task_addr_group_enable_get(secondGroup),
                                    (uint32_t) nrfx_ppi_task_addr_group_disable_get(firstGroup));

    //nrf_ppi_channel_endpoint_setup(secondPPIchannel, 
    nrf_ppi_channel_and_fork_endpoint_setup(secondPPIchannel, 
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    //(uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_FIRST_CAPTURE));
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_FIRST_CAPTURE),
                                    (uint32_t) nrf_timer_task_address_get(TIMEOUT_TIMER, NRF_TIMER_TASK_STOP));
    nrf_ppi_channel_and_fork_endpoint_setup(secondPPIchannelControl,
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrfx_ppi_task_addr_group_enable_get(thirdGroup),
                                    (uint32_t) nrfx_ppi_task_addr_group_disable_get(secondGroup));

    nrf_ppi_channel_and_fork_endpoint_setup(thirdPPIchannel,
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrf_timer_task_address_get(TIMEOUT_TIMER, NRF_TIMER_TASK_STOP),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_SECOND_CAPTURE));
    //nrf_ppi_channel_endpoint_setup(thirdPPIchannelControl, 
    nrf_ppi_channel_and_fork_endpoint_setup(thirdPPIchannelControl, 
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    //(uint32_t) nrfx_ppi_task_addr_group_disable_get(thirdGroup));
                                    (uint32_t) nrfx_ppi_task_addr_group_disable_get(thirdGroup),
                                    (uint32_t) nrf_timer_task_address_get(TEST_TIMER, NRF_TIMER_TASK_STOP));


#define EXT_FUNC
#ifdef EXT_FUNC
    uint8_t pulseToTake = TIMEOUT_US;
    //auto startTime = micros();
    //unsigned long remainingTime = timeout;

    //wrong if the timer has been already enabled (temp_timer) it must be reconfigured, same for the channels 
    //while (startState == TIMEOUT_US && remainingTime > 0) {

        pulseToTake = measurePulse(pin, state, firstGroup);
        
        /*
        if (startState == TIMEOUT_US) {
            //startState = measurePulse(pin, state, firstGroup);
            nrf_ppi_group_disable(firstGroup);
            nrf_ppi_group_disable(secondGroup);
            nrf_ppi_group_disable(thirdGroup);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_CLEAR);
            //nrf_ppi_group_enable(firstGroup);
            nrf_ppi_group_disable(firstGroup);
            nrf_ppi_group_disable(secondGroup);
            nrf_ppi_group_disable(thirdGroup);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_CLEAR);
            nrf_ppi_group_disable(firstGroup);
            nrf_ppi_group_disable(secondGroup);
            nrf_ppi_group_disable(thirdGroup);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_CLEAR);
            nrf_ppi_group_disable(firstGroup);
            nrf_ppi_group_disable(secondGroup);
            nrf_ppi_group_disable(thirdGroup);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_STOP);
            nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_CLEAR);
        }
        remainingTime = timeout - (micros() - startTime);
    }
    */
#else
    uint32_t firstState, secondState, thirdState; 
    core_util_critical_section_enter();
    firstState = nrf_gpio_pin_read(pin);
    nrf_ppi_group_enable(firstGroup);
    secondState = nrf_gpio_pin_read(pin);
    __NOP();
    thirdState = nrf_gpio_pin_read(pin);
    core_util_critical_section_exit();

    uint8_t pulseToTake;
    if (firstState == secondState && firstState == thirdState) {
        if (firstState != state) {
            pulseToTake = 1;
        } else {
            pulseToTake = 2;
        }
    } else {
        pulseToTake = 0;
    }
#endif
/*
    uint32_t firstState, secondState, timerStartedValue; 
    core_util_critical_section_enter();
    firstState = nrf_gpio_pin_read(pin);
    //__NOP();
    //__NOP();
    //__NOP();
    nrf_ppi_group_enable(firstGroup);
    secondState = nrf_gpio_pin_read(pin);
    core_util_critical_section_exit();

    timerStartedValue = nrf_timer_cc_read(TEMP_TIMER, TIMER_STARTED_CHANNEL);

    //bool timerStarted = (timerStartedValue == 1);//(timerStartedValue != 100); 
    /*
    uint8_t pulseToTake = 0;
    if (firstState == secondState) {
        if (firstState != state) {
            pulseToTake = 1;
        } else {
            pulseToTake = 2;
        }
    } else {
        if (firstState != state) {
            if (timerStarted){
                pulseToTake = 1;
            } else {
                //edge missed -> alternatives:  -- return timeout -- try another time until timeout -- start timer anyway (not here but previously) -- capture 3 pulses, return the third one 
                // pulseToTake = 3;
                pulseToTake = 2;
            }
        } else {
            if (timerStarted) {
                pulseToTake = 2;
            } else {
                pulseToTake = 1;
            }
        }
    }
    */


    //uint8_t pulseToTake = 0;

    

    unsigned long pulseTime = 0;
    unsigned long pulseFirst = 0;
    unsigned long pulseSecond = 0;

    //uint8_t pulseToTake = startState;

    if (!pulseToTake) {
        pulseTime = TIMEOUT_US;
    }

    if (pulseToTake >= 1) {
        while (!pulseFirst) {
            pulseFirst = nrf_timer_cc_read(TEMP_TIMER, TIMER_FIRST_CHANNEL);
        }
        pulseTime = pulseFirst;
    }

    if (pulseToTake == 2) {
        while (!pulseSecond) {
            pulseSecond = nrf_timer_cc_read(TEMP_TIMER, TIMER_SECOND_CHANNEL);
        }
        pulseTime = (uint32_t) ( (int)pulseSecond - (int)pulseFirst);
    }
    
    
    nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_SHUTDOWN);
    nrfx_gpiote_in_uninit(pin);
    nrfx_ppi_group_free(firstGroup);
    nrfx_ppi_group_free(secondGroup);
    nrfx_ppi_group_free(thirdGroup);
    nrf_ppi_channel_group_clear(firstGroup);
    nrf_ppi_channel_group_clear(secondGroup);
    nrf_ppi_channel_group_clear(thirdGroup);
    nrfx_ppi_channel_free(firstPPIchannel);
    nrfx_ppi_channel_free(firstPPIchannelControl);
    nrfx_ppi_channel_free(secondPPIchannel);
    nrfx_ppi_channel_free(secondPPIchannelControl);
    nrfx_ppi_channel_free(thirdPPIchannel);
    nrfx_ppi_channel_free(thirdPPIchannelControl);

    //return timerStartedValue;
#ifdef DEBUG
    if (false && firstState != secondState) {
    Serial.print("1 state: ");
    Serial.print(firstState);
    Serial.print("  2 state: ");
    Serial.print(secondState);
    Serial.print("  timer started: ");
    Serial.print(timerStarted);
    Serial.print('\n');
    Serial.print("1 pulse: ");
    Serial.print(pulseFirst);
    Serial.print("  2 pulse: ");
    Serial.print(pulseSecond);
    Serial.print('\n');
    Serial.print("pulse time: ");
    Serial.print(pulseTime);
    Serial.print('\n');
    Serial.print('\n');
    }    
#endif

    return pulseTime; // 
    //return (pulseTime >> 4); //for 16 MHz timers
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
    return pulseIn(digitalPinToPinName(pin), (PinStatus)state, timeout);
}
