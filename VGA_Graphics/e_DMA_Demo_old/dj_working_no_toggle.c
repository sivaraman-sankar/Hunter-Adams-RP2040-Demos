/*
    DJ Sampler Pad + Audio Mixer
    ECE 5730 Final Project
    Nicholas Papapanou (ngp37) and Sivaraman Sankar (ss4362) 

    VGA Connections:
    - GPIO 16 <---> VGA Hsync (White)
    - GPIO 17 <---> VGA Vsync (Gray)
    - GPIO 18 <---> 330 ohm resistor ---> VGA Red
    - GPIO 19 <---> 330 ohm resistor ---> VGA Green
    - GPIO 20 <---> 330 ohm resistor ---> VGA Blue
    - RP2040 GND <---> VGA GND (Black)

    DAC Connections:
    - RP2040 3.3V <---> VDD (Pin 1 - Red)
    - GPI0 5 <---> CS (Pin 2 - Yellow)
    - GPIO 6 <---> SCK (Pin 3 - Orange)
    - GPIO 7 <---> MOSI (Pin 4 - Blue)
    - GPIO 8 <---> LDAC (Pin 5 - Green)
    - Right Speaker <---> Out B (Pin 6 - Yellow)
    - RP2040 GND <---> VSS (Pin 7 - Black)
    - Left Speaker <---> Out A (Pin 8 - Yellow)

    Keypad Connections:
    - GPIO 9 <---> 330 Ohms <---> Pin 1 (Row 1 - White)
    - GPIO 10 <---> 330 Ohms <---> Pin 2 (Row 2 - Blue)
    - GPIO 11 <---> 330 Ohms <---> Pin 3 (Row 3 - Red)
    - GPIO 12 <---> 330 Ohms <---> Pin 4 (Row 4 - Yellow)
    - GPIO 13 <---> Pin 5 (Column 1 - Orange)
    - GPIO 14 <---> Pin 6 (Column 2 - Brown)
    - GPIO 15 <---> Pin 7 (Column 3 - Green)

    ADC Connections (NEED TO FIX ON BREADBOARD):
    - GPIO 26 (ADC 0) <---> Left Potentiometer (0 - 3.3V, White)
    - GPIO 27 (ADC 1) <---> Right Potentiometer (0 - 3.3V, White)
    - GPIO 28 (ADC 2) <---> PROTECTION? <---> Kick Pedal (0 - ?? V)

    Serial Connections:
    - GPIO 0 <---> UART RX (Yellow)
    - GPIO 1 <---> UART TX (Orange)
    - RP2040 GND <---> UART GND (Black)
*/

/* ===== IMPORTS ===== */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"

#include "hardware/dma.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/adc.h"
#include "hardware/sync.h"

#include "pt_cornell_rp2040_v1_3.h"
#include "vga16_graphics.h"

#include "backing_drums1.c"
#include "backing_jazzy.c"

#include "kick.c"
#include "snare.c"
#include "hihat.c"

/* ===== DEFINITIONS ===== */

// Enum for key scale states
typedef enum { A_MAJOR, C_MAJOR, G_MAJOR } KeyState;
#define NUM_KEYS 3
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
    {220.00, 246.94, 277.18, 329.63, 370.00}, // A major
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

// Fixed-point multiplier range for pitch bending: [0.94, 1.06]
#define FIX15_MIN_PITCH float2fix15(0.94)
#define FIX15_RANGE_PITCH float2fix15(0.12)
#define BEND_THRESHOLD float2fix15(0.001)

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
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

// GPIO for timing the ISR
#define ISR_GPIO 2

// DAC config macros
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// Backing track struct
typedef struct {
    const unsigned short *data;
    const unsigned int length;
} AudioTrack;

AudioTrack tracks[] = {
    { backing_drums1, backing_drums1_len },
    { backing_jazzy, backing_jazzy_len },
    // Add more here
};

#define NUM_TRACKS 2
int currentTrack = 1;

typedef struct {
    const unsigned short *data;
    const unsigned int length;
} DrumSample;

DrumSample drums[] = {
    { kick, kick_len },
    { snare, snare_len },
    { hihat, hihat_len }
};

#define NUM_DRUMS (sizeof(drums)/sizeof(drums[0]))

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

int data_chan_A;
int ctrl_chan_A;
int data_chan_B;
int ctrl_chan_B;

void playDrum(int drumIndex) {
    if (drumIndex < 0 || drumIndex >= NUM_DRUMS) return;

    // Abort any previous transfer
    dma_channel_abort(data_chan_B);
    dma_channel_abort(ctrl_chan_B);

    // Clear any pending interrupt flags (optional but clean)
    dma_channel_acknowledge_irq0(data_chan_B);
    dma_channel_acknowledge_irq0(ctrl_chan_B);

    // Setup new transfer
    dma_channel_set_read_addr(ctrl_chan_B, &drums[drumIndex].data, false);
    dma_channel_set_trans_count(ctrl_chan_B, 1, false);
    dma_channel_set_read_addr(data_chan_B, drums[drumIndex].data, false);
    dma_channel_set_trans_count(data_chan_B, drums[drumIndex].length, false);

    // Start playback
    dma_start_channel_mask((1u << ctrl_chan_B));
}

/* ===== TIMER ISR FOR PIANO PLAYING ===== */

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

/* ===== PITCH BENDING THREAD ===== */

static PT_THREAD(thread_pitch_bend(struct pt *pt))
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

        // Precompute the scale in fixed-point
        fix15 adc_frac_fix15 = divfix(int2fix15(adc_raw), int2fix15(4095)); // adc_raw / 4095
        new_fix_mult = FIX15_MIN_PITCH + multfix15(adc_frac_fix15, FIX15_RANGE_PITCH);

        // Filter potentiometer noise
        if (absfix15(new_fix_mult - last_fix_mult) >= BEND_THRESHOLD) {
            pitch_multiplier_fix15 = new_fix_mult;
            // printf("\nPitch Multiplier: %f\n", fix2float15(pitch_multiplier_fix15));
            // adc raw 
            // printf("ADC Raw: %d\n", adc_raw);
            last_fix_mult = new_fix_mult;
        }

        PT_YIELD_usec(30000);
    }

    PT_END(pt);
}

/* ===== KEYPAD THREAD ===== */

static PT_THREAD (thread_keypad_input(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    // Add static variables for debounce
    static int possible = 0;
    static enum { NOT_PRESSED, MAYBE_PRESSED, PRESSED, MAYBE_NOT_PRESSED } BOUNCE_STATE = NOT_PRESSED;


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

        // Keypad debounce for 6 - 9
        switch (BOUNCE_STATE) {
            case NOT_PRESSED:
                if (i >= 6 && i <= 11) {
                    possible = i;
                    BOUNCE_STATE = MAYBE_PRESSED;
                }
                break;
        
            case MAYBE_PRESSED:
                if (i == possible) {
                    if (possible == 6) {
                        // Key 6 -> change scale
                        current_key = (current_key + 1) % NUM_KEYS;
                    } else if (possible == 7) {
                        // Key 7 -> Play kick
                        playDrum(0); // 0 = kick
                    } else if (possible == 8) {
                        // Key 8 -> Play snare
                        playDrum(1); // 1 = snare
                    } else if (possible == 9) {
                        // Key 9 -> Play hihat
                        playDrum(2); // 2 = hihat
                    }
                    else if (possible == 11) { // Key 12: cycle backing track
                    }
                    BOUNCE_STATE = PRESSED;
                } else {
                    BOUNCE_STATE = NOT_PRESSED;
                }
                break;
        
            case PRESSED:
                if (i != possible) {
                    BOUNCE_STATE = MAYBE_NOT_PRESSED;
                }
                break;
        
            case MAYBE_NOT_PRESSED:
                if (i == possible) {
                    BOUNCE_STATE = PRESSED;
                } else {
                    BOUNCE_STATE = NOT_PRESSED;
                }
                break;
        }

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

/* ===== MAIN ===== */

int main() {

    // Initidalize stdio
    stdio_init_all();

    // Setup SPI
    spi_init(SPI_PORT, 20000000); // baud rate set to 20MHz
    spi_set_format(SPI_PORT, 16, 0, 0, 0); // (channel, data bits per transfer, polarity, phase, order)

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

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

    // set up increments for calculating envelope
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

    /* ===== Backing Track DMA Setup (Left Speaker, DAC Channel A) ===== */
    data_chan_A = dma_claim_unused_channel(true);
    ctrl_chan_A = dma_claim_unused_channel(true);

    dma_channel_config c_ctrl_A = dma_channel_get_default_config(ctrl_chan_A);
    channel_config_set_transfer_data_size(&c_ctrl_A, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl_A, false);
    channel_config_set_write_increment(&c_ctrl_A, false);
    channel_config_set_chain_to(&c_ctrl_A, data_chan_A);

    dma_channel_configure(
        ctrl_chan_A, &c_ctrl_A,
        &dma_hw->ch[data_chan_A].read_addr,
        (void*)&tracks[currentTrack].data,
        1, false
    );

    dma_channel_config c_data_A = dma_channel_get_default_config(data_chan_A);
    channel_config_set_transfer_data_size(&c_data_A, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data_A, true);
    channel_config_set_write_increment(&c_data_A, false);
    
    // Set DMA Timer 0 for 22.05 kHz playback (125 MHz × 4 / 22673 = 22050 Hz)
    dma_timer_set_fraction(0, 4, 22673);

    channel_config_set_dreq(&c_data_A, 0x3B);
    channel_config_set_chain_to(&c_data_A, ctrl_chan_A);

    dma_channel_configure(
        data_chan_A, &c_data_A,
        &spi_get_hw(SPI_PORT)->dr,
        tracks[currentTrack].data,
        tracks[currentTrack].length,
        false
    );

    /* ===== Drum DMA Setup (Right Speaker, DAC Channel B) ===== */
    data_chan_B = dma_claim_unused_channel(true);
    ctrl_chan_B = dma_claim_unused_channel(true);

    // Control channel B setup
    dma_channel_config c_ctrl_B = dma_channel_get_default_config(ctrl_chan_B);
    channel_config_set_transfer_data_size(&c_ctrl_B, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl_B, false);
    channel_config_set_write_increment(&c_ctrl_B, false);
    channel_config_set_chain_to(&c_ctrl_B, data_chan_B);

    dma_channel_configure(
        ctrl_chan_B, &c_ctrl_B,
        &dma_hw->ch[data_chan_B].read_addr,
        (void*)&drums[0].data,  // dummy default, will change when triggered
        1, false
    );
    
    // Data channel B setup
    dma_channel_config c_data_B = dma_channel_get_default_config(data_chan_B);
    channel_config_set_transfer_data_size(&c_data_B, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data_B, true);
    channel_config_set_write_increment(&c_data_B, false);
    
    // Use same DMA timer 0 (0x3B), same frequency
    channel_config_set_dreq(&c_data_B, 0x3B);
    channel_config_set_chain_to(&c_data_B, data_chan_B);
    
    dma_channel_configure(
        data_chan_B, &c_data_B,
        &spi_get_hw(SPI_PORT)->dr,
        drums[0].data,
        drums[0].length,
        false
    );

    // Start playback DMA chain
    dma_start_channel_mask((1u << ctrl_chan_A));

    setupADC0() ;
    setupKeypad() ;

    // Add core 0 threads
    pt_add_thread(thread_keypad_input) ;
    pt_add_thread(thread_pitch_bend);

    // Start scheduling core 0 threads
    pt_schedule_start ;

    current_note_index = -1; // No tone by default

}