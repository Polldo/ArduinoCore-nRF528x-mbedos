#include "Arduino.h"

#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>
#include "nrfx_gpiote.h"

/* Hot encoded peripherals. Could be chosen with a more clever strategy */
#define TEMP_TIMER (NRF_TIMER2)
#define TIMER_CHANNEL (NRF_TIMER_CC_CHANNEL1)
#define TIMER_TASK_CAPTURE (NRF_TIMER_TASK_CAPTURE1)
#define GPIOTE_START_EVENT (NRF_GPIOTE_EVENTS_IN_1)
#define GPIOTE_START_EVENT_IDX (1)
#define PPI_CHANNEL_START (NRF_PPI_CHANNEL3)
#define GPIOTE_STOP_EVENT (NRF_GPIOTE_EVENTS_IN_2)
#define GPIOTE_STOP_EVENT_IDX (2)
#define PPI_CHANNEL_STOP (NRF_PPI_CHANNEL4)

#ifdef debug
//CONTROLLA effetti di pulsein sul pin considerato. posso usarlo come niente fosse dopo? eventuali interrupt vengono tolti?
// prova ad utilizzare altri pin in input e output sia prima che dopo l'utilizzo di questa funzione
//FIGHISSIMO in questo modo si può implementare una metodo per contare tempo di esecuzione di blocchi di istruzioni.
//implementare una ricerca di CANALI non usati e GPIOTE non usati prima di tutto, se non ce ne sono ritorna 0.
//unsigned long pulseIn(PinName pin, PinStatus state, unsigned long timeout)
unsigned long pulseInSpec(PinName pin, PinStatus state, unsigned long timeout)
{

    /* Select correct trigger polarity */
    auto polarityStartTrigger = state ? NRF_GPIOTE_POLARITY_HITOLO : NRF_GPIOTE_POLARITY_LOTOHI;
    auto polarityStopTrigger = state ? NRF_GPIOTE_POLARITY_LOTOHI : NRF_GPIOTE_POLARITY_HITOLO;
    /* Configure timer */
    nrf_timer_mode_set(TEMP_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_frequency_set(TEMP_TIMER, NRF_TIMER_FREQ_1MHz); //NRF_TIMER_FREQ_16MHz
    nrf_timer_bit_width_set(TEMP_TIMER, NRF_TIMER_BIT_WIDTH_32);
    /* Clear latched channel value */
    nrf_timer_cc_write(TEMP_TIMER, TIMER_CHANNEL, 0);
    /* Initialize GPIOTE if not already initialized */
    static bool first_init = true;
    if (first_init) {
        if (!nrfx_gpiote_is_init()) {
            nrfx_gpiote_init();
        }
    }
    /* Configure Start Event -> when timer will start to count */
    nrf_gpiote_event_configure(GPIOTE_START_EVENT_IDX, pin, NRF_GPIOTE_POLARITY_TOGGLE);
    nrf_gpiote_event_enable(GPIOTE_START_EVENT_IDX);
    /* Configure PPI channels for Start and Stop events */
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_CHANNEL_START,
                                    (uint32_t) nrf_gpiote_event_addr_get(GPIOTE_START_EVENT),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_START),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_TASK_CAPTURE));


    gpio_t digPin = {pin};
	while (gpio_read(&digPin) != state); 
    nrf_ppi_channel_enable(PPI_CHANNEL_START);  
core_util_critical_section_enter(); 
//critical section
    uint32_t pulseTime = 0;
    while (!pulseTime) {
        pulseTime = nrf_timer_cc_read(TEMP_TIMER, TIMER_CHANNEL);
    }
    /* Deinitialize PPI channels and Events */
    nrf_ppi_channel_disable(PPI_CHANNEL_START);
core_util_critical_section_exit(); 
    nrf_gpiote_event_disable(GPIOTE_START_EVENT_IDX);
    nrf_gpiote_event_clear(GPIOTE_START_EVENT);
    /* Shutdown timer to lower power consumption */
    nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_SHUTDOWN);

    return pulseTime; // (pulse_time >> 4) for 16 MHz timers
}
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
    return pulseIn(digitalPinToPinName(pin), (PinStatus)state, timeout);
}
#endif

unsigned long pulseInLong(PinName pin, PinStatus state, unsigned long timeout)
{
    /* Select correct trigger polarity */
    auto polarityStartTrigger = state ? NRF_GPIOTE_POLARITY_LOTOHI : NRF_GPIOTE_POLARITY_HITOLO;
    auto polarityStopTrigger = state ? NRF_GPIOTE_POLARITY_HITOLO : NRF_GPIOTE_POLARITY_LOTOHI;
    /* Configure timer */
    nrf_timer_mode_set(TEMP_TIMER, NRF_TIMER_MODE_TIMER);
    nrf_timer_frequency_set(TEMP_TIMER, NRF_TIMER_FREQ_1MHz); //NRF_TIMER_FREQ_16MHz
    nrf_timer_bit_width_set(TEMP_TIMER, NRF_TIMER_BIT_WIDTH_32);
    /* Clear latched channel value */
    nrf_timer_cc_write(TEMP_TIMER, TIMER_CHANNEL, 0);
    /* Initialize GPIOTE if not already initialized */
    static bool first_init = true;
    if (first_init) {
        if (!nrfx_gpiote_is_init()) {
            nrfx_gpiote_init();
        }
    }
    /* Configure Start Event -> when timer will start to count */
    nrf_gpiote_event_configure(GPIOTE_START_EVENT_IDX, pin, polarityStartTrigger);
    nrf_gpiote_event_enable(GPIOTE_START_EVENT_IDX);
    /* Configure Stop Event -> when timer will stop counting */
    nrf_gpiote_event_configure(GPIOTE_STOP_EVENT_IDX, pin, polarityStopTrigger);
    nrf_gpiote_event_enable(GPIOTE_STOP_EVENT_IDX);
    /* Configure PPI channels for Start and Stop events */
    nrf_ppi_channel_and_fork_endpoint_setup(PPI_CHANNEL_START,
                                    (uint32_t) nrf_gpiote_event_addr_get(GPIOTE_START_EVENT),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_START),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_CLEAR));
    nrf_ppi_channel_enable(PPI_CHANNEL_START);  

    nrf_ppi_channel_and_fork_endpoint_setup(PPI_CHANNEL_STOP, 
                                    (uint32_t) nrf_gpiote_event_addr_get(GPIOTE_STOP_EVENT),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, TIMER_TASK_CAPTURE),
                                    (uint32_t) nrf_timer_task_address_get(TEMP_TIMER, NRF_TIMER_TASK_STOP));
    nrf_ppi_channel_enable(PPI_CHANNEL_STOP);  

    uint32_t pulseTime = 0;
    while (!pulseTime) {
        pulseTime = nrf_timer_cc_read(TEMP_TIMER, TIMER_CHANNEL);
    }
    /* Deinitialize PPI channels and Events */
    nrf_ppi_channel_disable(PPI_CHANNEL_START);
    nrf_ppi_channel_disable(PPI_CHANNEL_STOP);
    nrf_gpiote_event_disable(GPIOTE_START_EVENT_IDX);
    nrf_gpiote_event_disable(GPIOTE_STOP_EVENT_IDX);
    nrf_gpiote_event_clear(GPIOTE_START_EVENT);
    nrf_gpiote_event_clear(GPIOTE_STOP_EVENT);
    /* Shutdown timer to lower power consumption */
    nrf_timer_task_trigger(TEMP_TIMER, NRF_TIMER_TASK_SHUTDOWN);

    return pulseTime; // (pulse_time >> 4) for 16 MHz timers
}

unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout)
{
    return pulseInLong(digitalPinToPinName(pin), (PinStatus)state, timeout);
}




