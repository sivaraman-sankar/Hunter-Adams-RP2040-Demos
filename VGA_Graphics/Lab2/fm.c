#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "pt_cornell_rp2040_v1_3.h"

// Low-level alarm infrastructure
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0))
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)((((signed long long)(a)) << 15) / (b))

// Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0 // 2^32
#define Fs 20000.0
#define DELAY 50 // 1/Fs in microseconds

// Drum sound parameters
const float F_main = 0.5;
const float F_fm = 0.8;
int drum_mode = 0;

// DDS units for core 0
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (F_main * two32) / Fs; // Carrier frequency of 261Hz (middle C)

// Carrier and Modulator phase accumulators
volatile unsigned int phase_accum_mod_0;
volatile unsigned int phase_incr_mod_0 = (F_fm * two32) / Fs; // Modulator frequency = 1.8 * Carrier

// DDS sine table
#define sine_table_size 256
fix15 sin_table[sine_table_size];

// Values output to DAC
int DAC_output_0;

// FM Synthesis parameters
fix15 max_amplitude = int2fix15(1);
fix15 attack_inc;
fix15 decay_inc;
fix15 current_amplitude_0 = int2fix15(1);
fix15 modulation_index = float2fix15(1.5); // Modulation index

// Timing parameters for beeps
#define ATTACK_TIME 1
#define DECAY_TIME 10750
#define BEEP_DURATION 10750
#define BEEP_REPEAT_INTERVAL 1000

// State machine variables
volatile unsigned int STATE_0 = 0;
volatile unsigned int count_0 = 0;

// SPI data
uint16_t DAC_data_0;

// DAC parameters
#define DAC_config_chan_B 0b1011000000000000 // B-channel active

// SPI configurations (GPIO numbers, not pin numbers)
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
#define LED 25
#define SPI_PORT spi0

// GPIO for timing the ISR
#define ISR_GPIO 2

// Timer ISR
co

// Protothread for LED blinking
static PT_THREAD(protothread_core_0(struct pt *pt))
{
    PT_BEGIN(pt);

    while (1)
    {
        gpio_put(LED, !gpio_get(LED));
        PT_YIELD_usec(500000);
    }

    PT_END(pt);
}

// Core 0 entry point
int main()
{
    stdio_init_all();
    printf("FM Synthesis on RP2040!\n");

    spi_init(SPI_PORT, 20000000);
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc = divfix(max_amplitude, int2fix15(DECAY_TIME));

    for (int ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    irq_set_enabled(ALARM_IRQ, true);
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    pt_add_thread(protothread_core_0);
    pt_schedule_start;
}