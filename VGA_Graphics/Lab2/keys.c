/***************
 *  V. Hunter Adams (vha3@cornell.edu)

    A timer interrupt on core 0 generates a 400Hz beep
    thru an SPI DAC, once per second. A single protothread
    blinks the LED.

    GPIO 5 (pin 7) Chip select
    GPIO 6 (pin 9) SCK/spi0_sclk
    GPIO 7 (pin 10) MOSI/spi0_tx
    GPIO 2 (pin 4) GPIO output for timing ISR
    3.3v (pin 36) -> VCC on DAC 
    GND (pin 3)  -> GND on DAC 

 */

// Consolidated includes
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#include "pt_cornell_rp2040_v1_3.h"
#include "vga16_graphics.h"

// Enum for key scale states
typedef enum { A_MAJOR, C_MAJOR, G_MAJOR } KeyState;
volatile KeyState current_key = A_MAJOR;

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// Base frequencies for A, C, and G major pentatonic scales
float pentatonic_freqs[3][5] = {
    {220.00, 246.94, 293.66, 329.63, 370.00}, // A major
    {261.63, 293.66, 329.63, 392.00, 440.00}, // C major
    {196.00, 220.00, 246.94, 293.66, 329.63}  // G major
};
volatile int current_note_index = -1;
volatile int active_key = -1;

#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             250
#define DECAY_TIME              250
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10500
#define BEEP_REPEAT_INTERVAL    50000

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0
// ADC constants
#define ADC0_PIN 26
#define ADC1_PIN 27
#define ADC2_PIN 28
#define ADC0_CHAN 0
#define ADC1_CHAN 1
#define ADC2_CHAN 2
// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;
volatile fix15 pitch_multiplier_fix15 = float2fix15(1.0); // Default: no pitch bend


char keytext[40];
int prev_key = 0;

//GPIO for timing the ISR
#define ISR_GPIO 2


/* ===== SETUP FUNCTIONS ===== */

void setupADC0() {
    adc_init();
    adc_gpio_init(ADC0_PIN);
    adc_select_input(ADC0_CHAN);
}

void setupADC1() {
    adc_init();
    adc_gpio_init(ADC1_PIN);
    adc_select_input(ADC1_CHAN);
}

void setupADC2() {
    adc_init();
    adc_gpio_init(ADC2_PIN);
    adc_select_input(ADC2_CHAN);
}


void setupKeypad() {
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN));
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN));
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN));
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4));
    gpio_pull_down((BASE_KEYPAD_PIN + 5));
    gpio_pull_down((BASE_KEYPAD_PIN + 6));
}

// This timer ISR is called on core 0
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    if (active_key >= 0 && active_key < 5) {
        float base_freq = pentatonic_freqs[current_key][active_key];
        float adjusted_freq = base_freq * fix2float15(pitch_multiplier_fix15);
        phase_incr_main_0 = (unsigned int)((adjusted_freq * two32) / Fs);
        phase_accum_main_0 += phase_incr_main_0;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0 >> 24])) + 2048;

        // Maintain full amplitude
        if (current_amplitude_0 < max_amplitude) {
            current_amplitude_0 += attack_inc;
            if (current_amplitude_0 > max_amplitude) current_amplitude_0 = max_amplitude;
        }

        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
    } else {
        // No key held, ramp down amplitude
        if (current_amplitude_0 > 0) {
            current_amplitude_0 -= decay_inc;
            if (current_amplitude_0 < 0) current_amplitude_0 = 0;

            phase_accum_main_0 += phase_incr_main_0;
            DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                sin_table[phase_accum_main_0 >> 24])) + 2048;

            DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));
            spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
        }
    }

    // Removed legacy STATE_0 and BEEP_REPEAT_INTERVAL logic; frequency update handled above.

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0) ;

}



// Fixed-point multiplier range for pitch bending: [0.9, 1.1]
#define FIX15_MIN_PITCH float2fix15(1.0)
#define FIX15_RANGE_PITCH float2fix15(1.12)
#define BEND_THRESHOLD float2fix15(0.005)


static PT_THREAD(protothread_pitch_bend(struct pt *pt))
{
    PT_BEGIN(pt);

    static uint16_t adc_raw;
    static fix15 new_fix_mult;
    static fix15 last_fix_mult = float2fix15(1.0); // Default to no bend

    while (1)
    {
        // Read the potentiometer on ADC0
        adc_select_input(ADC0_CHAN);
        adc_raw = adc_read();

        // Map [0, 4095] to [0.9, 1.1] in fixed point
        new_fix_mult = FIX15_MIN_PITCH + multfix15(int2fix15(adc_raw), divfix(FIX15_RANGE_PITCH, int2fix15(4095)));

        // Filter potentiometer noise
        if (absfix15(new_fix_mult - last_fix_mult) >= BEND_THRESHOLD) {
            pitch_multiplier_fix15 = new_fix_mult;
            printf("\nPitch Multiplier: %f\n", fix2float15(pitch_multiplier_fix15));
            // adc raw 
            printf("ADC Raw: %d\n", adc_raw);
            last_fix_mult = new_fix_mult;
        }

        

        PT_YIELD_usec(30000);
    }

    PT_END(pt);
}


// This thread runs on core 1
static PT_THREAD (protothread_core_1(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    while(1) {
        // Blink LED and check for key press
        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ; 
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if (keypad & button) break ;
        }
        // If we found a button . . .
        if (keypad & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;

        // Print key to terminal
        // printf("\nKey pressed: %d", i) ;

        // Update active key
        if (i >= 1 && i <= 5) {
            active_key = i - 1;
        } else {
            active_key = -1;
        }

        // Keypad mapping for scale selection: 6=A_MAJOR, 7=C_MAJOR, 8=G_MAJOR
        if (i == 6) {
            current_key = A_MAJOR;
        } else if (i == 7) {
            current_key = C_MAJOR;
        } else if (i == 8) {
            current_key = G_MAJOR;
        }

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        
        // Toggle on LED
        gpio_put(LED, !gpio_get(LED)) ;

        // Yield for 500 ms
        PT_YIELD_usec(500000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}



// Core 0 entry point
int main() {
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();

    printf("\nHello, friends!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    
    setupADC0() ;
    setupKeypad() ;


    // Add core 0 threads
    pt_add_thread(protothread_core_1) ;
    pt_add_thread(protothread_pitch_bend);

    // Start scheduling core 0 threads
    pt_schedule_start ;

    current_note_index = -1; // No tone by default
}
