/**
 * Bruce Land and Hunter Adams (vha3@cornell.edu)
 * ========================================================
 * Basic Karplus-Strong Algorithm:
 * The actual syntheis is about 25 lines of code in the
 * compute_sample ISR routine
 * ========================================================
 *
 * Protothreads v1.1.1
 *
 *
 * DAC :
 * GPIO 5 (pin 7) Chip select
   GPIO 6 (pin 9) SCK/spi0_sclk
   GPIO 7 (pin 10) MOSI/spi0_tx
   3.3v (pin 36) -> VCC on DAC
   GND (pin 3)  -> GND on DAC
 *
 *
 * ==========
 * Core1:
 *
 * -- synthesis ISR triggered by a timer alarm
 * ---- runs at 20KHz (50 uSec interval) for audio synthesis.
 * ---- Worst case execution time is about 3 uSec.
 * ---- Computes Karplus-Strong string sound
 * ---- Pushes output to an SPI channel
 * ---- Fills an array with DDS results for the FFT thread -- NOT part of basic synthesis
 *
 * -- A play thread which
 * ---- sequences a C scale of 8 notes.
 * ====================================================
 * KS setup
 * see also
 * Extensions of the Karplus-Strong Plucked-String Algorithm
   David A. Jaffe, Julius O. Smith
   Computer Music Journal, Vol. 7, No. 2 (Summer, 1983), pp. 56-69

   -----------
 * for one octave scale C4 to B4
 *   notes C4	   C#	    D	    Eb	    E	    F	   F#	   G	   G#	   A	  Bb	    B4
          261.6	277.2	293.7	311.1	329.6	349.2	370.0	392.0	415.3	440.0	466.2	493.9
now compuate the string lengths as string_length = 20000/note_freq
 so lengths are 76.4526   72.1501   68.0967   64.2880   60.6796   57.2738   54.0541   51.0204 48.1580   45.4545   42.9000   40.4940
but a shift reqister cannot have fractional stages, os shift register lengths are
 string_length[n_note] = {76,  72,  68,  64,  60,  57,  54,  51,  48,  45,  42, 40};
a fractional length is added by using a one-sample maximun phase shifter which adds time coresponding
to the following fractional sample values
  fine_tune[n_note] = {.45, .15, .10, .29, .68, .27, .05, .02, .26, .45, .90, .49};
 if eta = (1-fine_tune)/(1+fine_tune)
 then
 tuning_out =  (eta * (tuning_input - last_tune_out)) + last_tune_in ;
 the
 full algorithm then is shift, lowpass filter, tuning filter, feed back into shifter
 */
// ==========================================
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/spi.h"

// ==========================================
// === hardware and protothreads globals
// ==========================================
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "string.h"
// protothreads header
#include "pt_cornell_rp2040_v1_3.h"

int play_scale = false;

// ==========================================
// === fixed point s15x16
// ==========================================
// s15x16 fixed point macros ==
// == resolution 2^-16 = 1.5e-5
// == dynamic range is 32767/-32768
typedef signed int s15x16;
#define muls15x16(a, b) ((s15x16)((((signed long long)(a)) * ((signed long long)(b))) >> 16)) // multiply two fixed 16:16
#define float_to_s15x16(a) ((s15x16)((a) * 65536.0))                                          // 2^16
#define s15x16_to_float(a) ((float)(a) / 65536.0)
#define s15x16_to_int(a) ((int)((a) >> 16))
#define int_to_s15x16(a) ((s15x16)((a) << 16))
#define divs15x16(a, b) ((s15x16)((((signed long long)(a) << 16) / (b))))
#define abss15x16(a) abs(a)

// =========================================
// === KS parameters
// =========================================
// # of notes in one octave
#define n_note 12
// index into the note tables
volatile int current_note;
// max size ~100 Hz -- could be bigger
#define max_string_size 200
s15x16 string[max_string_size];
s15x16 init_string[5][max_string_size];
// current sring pointers
int ptrin, ptrout;

//  notes C4	C#	    D	    Eb	    E	    F	   F#	   G	    G#	   A	    Bb	    B4
//        261.6	277.2	293.7	311.1	329.6	349.2	370.0	392.0	415.3	440.0	466.2	493.9
//  matlab length 76.4526   72.1501   68.0967   64.2880   60.6796   57.2738   54.0541   51.0204
//                48.1580   45.4545   42.9000   40.4940

volatile int string_length[n_note] = {76, 72, 68, 64, 60, 57, 54, 51, 48, 45, 42, 40};
// fractional length
// volatile fixAccum  fine_tune[n_note] = {.45, .15, .10, .29, .68, .27, .05, .02, .26, .45, .90, .49};
// eta = (1-fine_tune)/(1+fine_tune)
volatile s15x16 eta[n_note] = {float_to_s15x16(0.3768), float_to_s15x16(0.739), float_to_s15x16(0.8237),
                               float_to_s15x16(0.5528), float_to_s15x16(0.1908), float_to_s15x16(0.57), float_to_s15x16(0.8974),
                               float_to_s15x16(0.96), float_to_s15x16(0.727), float_to_s15x16(0.375), float_to_s15x16(0.0526), float_to_s15x16(0.3387)};

// 0 is random, 1 is lowpassed random;  2 is gaussian, 3 is sawtooth
// 0 is basic KS
int current_pluck = 1;
// gaussian
int pluck_pos = 20, pluck_width = 10, output_pos = 15;
// sawtoth
int saw_length = 20;

// filter variables for string
volatile s15x16 tuning_out, last_tune_out, last_tune_in;
volatile s15x16 lowpass_out;

// damping coefficient reduction in amp per synthesis sample
// implies a e-fold decay time constant of 100 samples
// at 20kHz, 5 mSec
volatile s15x16 damping_coeff = float_to_s15x16(0.998);

// Add near other global variables
volatile char desired_note = 'D';  // Default to C4
volatile bool note_played = false; // Flag to track if note has been played

// ==========================================
// === set up SPI DAC
// ==========================================
// All SPI DAC setup was gotten from HUnter Adams
// https://vanhunteradams.com/Pico/TimerIRQ/SPI_DDS.html
// DAC parameters
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// SPI configurations
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// data for the spi port
uint16_t DAC_data;

// ==========================================
// === set up timer ISR
// ==========================================
// 1/Fs in microseconds
// 20 KSamples/sec
volatile int alarm_period = 50;
// signals the ISR to pluck the string
volatile int note_start = true;

// ==========================================
// === set up timer ISR  used in this pgm
// ==========================================
// === timer alarm ========================
// !! modifiying alarm zero trashes the cpu
//        and causes LED  4 long - 4 short
// !! DO NOT USE alarm 0
// This low-level setup is ocnsiderably faster to execute
// than the hogh-level callback

#define ALARM_NUM 1
#define ALARM_IRQ TIMER_IRQ_1

// Add before protothread_play
int note_char_to_index(char note)
{
    switch (note)
    {
    case 'C':
        return 0; // C4
    case 'D':
        return 2; // D4
    case 'E':
        return 4; // E4
    case 'F':
        return 5; // F4
    case 'G':
        return 7; // G4
    case 'A':
        return 9; // A4
    case 'B':
        return 11; // B4
    // Handle sharps/flats
    case 'c':
        return 1; // C#4
    case 'd':
        return 3; // D#4/Eb4
    case 'f':
        return 6; // F#4
    case 'g':
        return 8; // G#4
    case 'a':
        return 10; // A#4/Bb4
    default:
        return 0; // Default to C4
    }
}

// ISR interval will be 10 uSec
//
// the actual ISR
void compute_sample(void);
//
static void alarm_irq(void)
{
    // mark ISR entry
    gpio_put(2, 1);
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    // arm the next interrupt
    // Write the lower 32 bits of the target time to the alarm to arm it
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + alarm_period;
    //
    // the routine which actually does the PDE
    compute_sample();

    // mark ISR exit
    gpio_put(2, 0);
}

// set up the timer alarm ISR
static void alarm_in_us(uint32_t delay_us)
{
    // Enable the interrupt for our alarm (the timer outputs 4 alarm irqs)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Set irq handler for alarm irq
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm irq
    irq_set_enabled(ALARM_IRQ, true);
    // Enable interrupt in block and at processor
    // Alarm is only 32 bits
    uint64_t target = timer_hw->timerawl + delay_us;
    // Write the lower 32 bits of the target time to the alarm which
    // will arm it
    timer_hw->alarm[ALARM_NUM] = (uint32_t)target;
}

// ==================================================
// === play the scale thread on core 1
// ==================================================

static PT_THREAD(protothread_play(struct pt *pt))
{
    PT_BEGIN(pt);

    // data structure for interval timer
    PT_INTERVAL_INIT();

    while (1)
    {
        static int i;
        PT_YIELD_usec(1000);
        if (play_scale)
        {

            // mod_attack_inc = div(current_depth, mod_attack_time) ;
            // mod_decay_inc = div(current_depth, mod_decay_time) ;
            for (i = 0; i < n_note; i++)
            {
                // current_main_inc = main_inc[i] ;
                // rho =
                current_note = i;
                note_start = true;
                // PT_YIELD_usec(20000) ;
                //  play until too small for DAC
                // PT_YIELD_UNTIL(pt, current_amp<onefix);
                PT_YIELD_usec(1000000);
            }
        }
        //
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // play thread

static PT_THREAD(protothread_play_note(struct pt *pt))
{
    PT_BEGIN(pt);

    // data structure for interval timer
    PT_INTERVAL_INIT();

    while (1)
    {
        PT_YIELD_usec(1000);

        // Check if the note has already been played
        if (!note_played)
        {
            // Convert the desired note character to an index
            current_note = note_char_to_index(desired_note);

            // Start playing the note
            note_start = true;

            // Wait for 1 second while the note plays
            PT_YIELD_usec(1000000);

            // Mark the note as played
            note_played = true;

            // Reset string values to silence the note
            for (int i = 0; i <= max_string_size; i++)
            {
                string[i] = 0;
            }
        }

        // Just idle after playing the note
        PT_YIELD_usec(100000);
    }
    PT_END(pt);
}

// ==================================================
// === ISR routine -- RUNNING on core 1
// ==================================================

void compute_sample(void)
{

    // ==== start a string with new data ===
    // If the note has been played, output silence
    if (note_played)
    {
        // Output silence (midpoint value for DAC)
        DAC_data = (DAC_config_chan_A | (2048 & 0xfff));
        spi0_hw->dr = DAC_data;
        // Reset the note played flag
        return;
    }

    if (note_start)
    {
        // reset the start flag
        note_start = false;
        // load the string with inietial condition
        for (int i = 0; i <= string_length[current_note]; i++)
        {
            string[i] = init_string[current_pluck][i]; // noise_table[i] ;
        }
        // reset the pointers
        ptrin = 1;
        ptrout = 2;
    } // note start

    // ==== play the KS ===
    else
    {
        // === low pass filter ===
        // the following is the classic KS lowpass with damping
        // a two-tap FIR running average
        lowpass_out = muls15x16(damping_coeff, ((string[ptrin] + string[ptrout]) >> 1));

        // === tuning all-pass filter ===
        // necessary becuase musical notes are not integer-related to sampling frequency
        tuning_out = muls15x16(eta[current_note], (lowpass_out - last_tune_out)) + last_tune_in;
        // all-pass state vars
        last_tune_out = tuning_out;
        last_tune_in = lowpass_out;

        // === string feedback ===
        string[ptrin] = tuning_out; ///(ptrin==20)?(string_input) : 0 ;
        // could add input here

        // === update and wrap pointers ===
        if (ptrin == string_length[current_note])
            ptrin = 1;
        else
            ptrin = ptrin + 1;
        //
        if (ptrout == string_length[current_note])
            ptrout = 1;
        else
            ptrout = ptrout + 1;

        // === DAC sscaling and output ===
        int temp = s15x16_to_int(string[ptrout]);
        DAC_data = (DAC_config_chan_A | ((temp + 2048) & 0xfff));

        // Write data to DAC
        // spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
        // NOTE === nonblocking SPI write ===
        spi0_hw->dr = DAC_data;

    } // current amp > 0
} // end ISR

static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000);
    // announce the threader version
    // non-blocking write
    serial_write;
    while (1)
    {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-7: ");
        // non-blocking write
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%d", &user_input);
        // update boid color
        if ((user_input > 0) && (user_input < 9))
        {
            note_played = false;

            // convert to note
            switch (user_input)
            {
            case 1:
                desired_note = 'C';
                break;
            case 2:
                desired_note = 'D';
                break;
            case 3:
                desired_note = 'E';
                break;
            case 4:
                desired_note = 'F';
                break;
            case 5:
                desired_note = 'G';
                break;
            case 6:
                desired_note = 'A';
                break;
            case 7:
                desired_note = 'B';
                break;

            default:
                break;
            }
        }
    } // END WHILE(1)
    PT_END(pt);
} // timer thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{

    // fire off synthesis interrupt
    alarm_in_us(alarm_period);

    //  === add threads  ====================
    // for core 1
    pt_add_thread(protothread_play_note);
    //
    // === initalize the scheduler ==========
    pt_schedule_start;
    // NEVER exits
    // ======================================
}

// ========================================
// === core 0 main
// ========================================
int main()
{

    // start the serial i/o
    stdio_init_all();

    // announce the threader version on system reset
    printf("\n\rProtothreads RP2040 v1.3 two-core, priority\n\r");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    // connected to spi DAC
    spi_init(SPI_PORT, 20000000);
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    // monitor ISR
    gpio_init(2);
    gpio_set_dir(2, GPIO_OUT);

    // set up four possible sring initial conditions arrays
    for (int i = 0; i < max_string_size; i++)
    {
        // random pluck -- very bright twang
        init_string[0][i] = int_to_s15x16((rand() & 0xfff) - 2047);
        // slghgtly lowpassed random drive to soften pluck transient
        if (i > 3)
        {
            init_string[1][i] = (init_string[0][i - 3] + init_string[0][i - 2] + init_string[0][i - 1] + init_string[0][i]) >> 2;
        }
        // Gaussian pluck -- more like a padded mallet strike
        init_string[2][i] = float_to_s15x16(2000 * exp(-(float)(i - pluck_pos) * (i - pluck_pos) / (pluck_width * pluck_width)));
        // sawtooth
        init_string[3][i] = float_to_s15x16((float)((i % saw_length) - 10) * 50);
    }

    // start core 1 threads
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // === config threads ========================
    pt_add_thread(protothread_serial);
    //
    // === initalize the scheduler ===============
    pt_schedule_start;
    // NEVER exits
    // ===========================================
} // end main