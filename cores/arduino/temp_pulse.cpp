//not working refactored
#include "Arduino.h"

#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_ppi.h>
#include "nrfx_gpiote.h"
#include "nrfx_ppi.h"

/* Hot encoded peripherals. Could be chosen with a more clever strategy */
#define TIMEOUT_TIMER (NRF_TIMER3)
#define WAIT_CHANNEL (NRF_TIMER_CC_CHANNEL2)
#define WAIT_TASK_CAPTURE (NRF_TIMER_TASK_CAPTURE2)
#define TEMP_TIMER (NRF_TIMER2)
#define TIMER_CHANNEL (NRF_TIMER_CC_CHANNEL1)
#define TIMER_TASK_CAPTURE (NRF_TIMER_TASK_CAPTURE1)

#define TIMEOUT_US (0)
#define ERROR_US (0xFFFFFFFF)

#define MEASURE_CRITICAL

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

static unsigned long measurePulse(PinName pin, PinStatus state, unsigned long timeout, nrf_ppi_channel_group_t startGroup);

//CONTROLLA effetti di pulsein sul pin considerato. posso usarlo come niente fosse dopo? eventuali interrupt vengono tolti?
// prova ad utilizzare altri pin in input e output sia prima che dopo l'utilizzo di questa funzione
//FIGHISSIMO in questo modo si può implementare una metodo per contare tempo di esecuzione di blocchi di istruzioni.
//unsigned long pulseIn(PinName pin, PinStatus state, unsigned long timeout)
unsigned long pulseIn(PinName pin, PinStatus state, unsigned long timeout)
{
    /* Configure timer */
    nrf_timer_mode_set(TEMP_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_frequency_set(TEMP_TIMER, NRF_TIMER_FREQ_1MHz); 
    nrf_timer_bit_width_set(TEMP_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_cc_write(TEMP_TIMER, TIMER_CHANNEL, 0);
    /* Configure pin Toggle Event */
    nrfx_gpiote_in_init(pin, &cfg, NULL);
    nrfx_gpiote_in_event_enable(pin, true); 

    /* Allocate PPI channels for starting and stopping the timer */
    nrf_ppi_channel_t startPPIchannel, startPPIchannelControl;
    nrf_ppi_channel_t stopPPIchannel, stopPPIchannelControl;
    nrfx_ppi_channel_alloc(&startPPIchannel);
    nrfx_ppi_channel_alloc(&startPPIchannelControl);
    nrfx_ppi_channel_alloc(&stopPPIchannel);
    nrfx_ppi_channel_alloc(&stopPPIchannelControl);
    /* Allocate PPI Group channels to allow activation and deactivation of channels as PPI tasks */
    nrf_ppi_channel_group_t startGroup, stopGroup;
    nrfx_ppi_group_alloc(&startGroup);
    nrfx_ppi_group_alloc(&stopGroup);

    /* Insert channels in corresponding group */
    nrfx_ppi_channel_include_in_group(startPPIchannel, startGroup);
    nrfx_ppi_channel_include_in_group(startPPIchannelControl, startGroup);
    nrfx_ppi_channel_include_in_group(stopPPIchannel, stopGroup);
    nrfx_ppi_channel_include_in_group(stopPPIchannelControl, stopGroup);

    /* Configure PPI channels for Start and Stop events */
    nrf_ppi_channel_and_fork_endpoint_setup(startPPIchannel,
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_START),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_CLEAR));
    nrf_ppi_channel_and_fork_endpoint_setup(startPPIchannelControl,
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrfx_ppi_task_addr_group_enable_get(stopGroup),
                                    (uint32_t) nrfx_ppi_task_addr_group_disable_get(startGroup));

    nrf_ppi_channel_and_fork_endpoint_setup(stopPPIchannel, 
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_TASK_CAPTURE),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_STOP));
    nrf_ppi_channel_endpoint_setup(stopPPIchannelControl, 
                                    (uint32_t) nrfx_gpiote_in_event_addr_get(pin),
                                    (uint32_t) nrfx_ppi_task_addr_group_disable_get(stopGroup));

    //unsigned long pulseTime;
#ifndef MEASURE_CRITICAL
    pulseTime = ERROR_US;
    auto startTime = micros();
    unsigned long remainingTime = timeout;
    //wrong if the timer has been already enabled (temp_timer) it must be reconfigured, same for the channels 
    while (pulseTime == ERROR_US && remainingTime > 0) {
        pulseTime = measurePulse(pin, state, remainingTime, startGroup);
        if (pulseTime == ERROR_US) {
            //pulseTime = measurePulse(pin, state, timeout, startGroup);
            nrf_ppi_group_disable(startGroup);
            nrf_ppi_group_disable(stopGroup);
        }
        remainingTime = timeout - (micros() - startTime);
    }
#else

    /* Wait for the pulse and measure it in another function */
    unsigned long pulseTime = measurePulse(pin, state, timeout, startGroup);

#endif

    /* Shutdown timer to lower power consumption */
    nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_SHUTDOWN);
    /* Deinitialize PPI channels and Events */
    nrfx_gpiote_in_uninit(pin);
    nrfx_ppi_group_free(startGroup);
    nrfx_ppi_group_free(stopGroup);
    nrf_ppi_channel_group_clear(startGroup);
    nrf_ppi_channel_group_clear(stopGroup);
    nrfx_ppi_channel_free(startPPIchannel);
    nrfx_ppi_channel_free(startPPIchannelControl);
    nrfx_ppi_channel_free(stopPPIchannel);
    nrfx_ppi_channel_free(stopPPIchannelControl);

    /* Return measured pulse in microseconds. 0 is returned in case of timeout */
    return pulseTime; // (pulse_time >> 4) for 16 MHz timers
}

static unsigned long measurePulse(PinName pin, PinStatus state, unsigned long timeout, nrf_ppi_channel_group_t startGroup)
{
    /* Configure a timer to handle timeout -> 1 MHz frequency, start counting from 0 */
    nrf_timer_mode_set(TIMEOUT_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_frequency_set(TIMEOUT_TIMER, NRF_TIMER_FREQ_1MHz); 
    nrf_timer_bit_width_set(TIMEOUT_TIMER, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_cc_write(TIMEOUT_TIMER, WAIT_CHANNEL, 0);
    nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_CLEAR);
    /* Start timeout timer */
    nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_START);

    nrf_timer_task_trigger(TIMEOUT_TIMER, WAIT_TASK_CAPTURE);
    timeout += nrf_timer_cc_read(TIMEOUT_TIMER, WAIT_CHANNEL);


#ifdef MEASURE_CRITICAL
    core_util_critical_section_enter();
#endif
    while (nrf_gpio_pin_read(pin) != state) {
        nrf_timer_task_trigger(TIMEOUT_TIMER, WAIT_TASK_CAPTURE);
        if (nrf_timer_cc_read(TIMEOUT_TIMER, WAIT_CHANNEL) >= timeout) {
#ifdef MEASURE_CRITICAL
            core_util_critical_section_exit();
#endif
            nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_SHUTDOWN);
			return TIMEOUT_US;
        }
    }

    while (nrf_gpio_pin_read(pin) == state) {
        nrf_timer_task_trigger(TIMEOUT_TIMER, WAIT_TASK_CAPTURE);
        if (nrf_timer_cc_read(TIMEOUT_TIMER, WAIT_CHANNEL) >= timeout) {
#ifdef MEASURE_CRITICAL
            core_util_critical_section_exit();
#endif
            nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_SHUTDOWN);
            return TIMEOUT_US;
        }
    }

    // Check if the pin is still != state ???   it slows down, maybe 1 us pulse cannot be handled
    // the problem is that if there is an interruption during this process, we could miss the edge -> critical section solves it
    // maybe the critical section can be reduced to just a check of the pin until the two while, if the edge has not passed then activate the group
    // else return with a timeout -----> if the entire code doesn't handle 1 us pulse errors will be frequent, otherwise I don't expect wrong results
    // but rare timeouts. 

    //PROBLEM without using critical section interrupts could cause many timeouts (this is a problem also for older cores)
    // in this way when the correct edge is missed, a timeout is returned. it is rare for pulses greater than 1 microsecond.
#ifndef MEASURE_CRITICAL

    core_util_critical_section_enter();
    nrf_ppi_group_enable(startGroup);
    if (nrf_gpio_pin_read(pin) == state) {
        core_util_critical_section_exit();
        nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_SHUTDOWN);
        return ERROR_US;
    }
    core_util_critical_section_exit();


    //nrf_ppi_group_enable(startGroup);

/*
    core_util_critical_section_enter();
    if (nrf_gpio_pin_read(pin) == state) {
        core_util_critical_section_exit();
        nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_SHUTDOWN);
        return 0;
    } else {
        nrf_ppi_group_enable(startGroup);
        core_util_critical_section_exit();
    }
*/
#endif

#ifdef MEASURE_CRITICAL
    nrf_ppi_group_enable(startGroup);
    core_util_critical_section_exit();
#endif

    //unsigned long pulseTime = 0;
    unsigned long pulseTime = TIMEOUT_US;
    while (!pulseTime) {
        pulseTime = nrf_timer_cc_read(TEMP_TIMER, TIMER_CHANNEL);
        nrf_timer_task_trigger(TIMEOUT_TIMER, WAIT_TASK_CAPTURE);
        if (nrf_timer_cc_read(TIMEOUT_TIMER, WAIT_CHANNEL) >= timeout)
        {
            nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_SHUTDOWN);
			return pulseTime;
			//return 0;
        }
    }

    nrf_timer_task_trigger(TIMEOUT_TIMER, NRF_TIMER_TASK_SHUTDOWN);
    return pulseTime;
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
    return pulseIn(digitalPinToPinName(pin), (PinStatus)state, timeout);
}
