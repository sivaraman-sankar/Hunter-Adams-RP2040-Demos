/**
 * karplus_strong.c
 * Implementation of Karplus-Strong string synthesis algorithm
 */

#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/multicore.h"
#include "karplus_strong.h"

// Fixed point macros
typedef signed int s15x16;
#define muls15x16(a, b) ((s15x16)((((signed long long)(a)) * ((signed long long)(b))) >> 16))
#define float_to_s15x16(a) ((s15x16)((a) * 65536.0))
#define s15x16_to_float(a) ((float)(a) / 65536.0)
#define s15x16_to_int(a) ((int)((a) >> 16))
#define int_to_s15x16(a) ((s15x16)((a) << 16))
#define abss15x16(a) abs(a)

#define n_note 22
#define max_string_size 200
volatile int current_note;
s15x16 string[max_string_size];
s15x16 init_string[5][max_string_size];
int ptrin, ptrout;
int pluck_pos = 20, pluck_width = 10, output_pos = 15;
int saw_length = 20;

volatile int string_length[n_note + 1] = {
    76, 72, 68, 64, 60, 57, 54, 51, 48, 45, 42, 40,
    38, 36, 34, 32, 30, 28, 26, 25, 23, 22 // A5 added
};

volatile s15x16 eta[n_note + 1] = {
    float_to_s15x16(0.3768), float_to_s15x16(0.739), float_to_s15x16(0.8237),
    float_to_s15x16(0.5528), float_to_s15x16(0.1908), float_to_s15x16(0.57),
    float_to_s15x16(0.8974), float_to_s15x16(0.96), float_to_s15x16(0.727),
    float_to_s15x16(0.375), float_to_s15x16(0.0526), float_to_s15x16(0.3387),
    float_to_s15x16(0.315), float_to_s15x16(0.298), float_to_s15x16(0.282),
    float_to_s15x16(0.267), float_to_s15x16(0.253), float_to_s15x16(0.239),
    float_to_s15x16(0.226), float_to_s15x16(0.213), float_to_s15x16(0.156), float_to_s15x16(0.201)};
int current_pluck = 1;
volatile s15x16 tuning_out, last_tune_out, last_tune_in;
volatile s15x16 lowpass_out;
volatile s15x16 damping_coeff = float_to_s15x16(0.998);

// Public variables
volatile char desired_note = 'D';
volatile bool note_played = false;

// DAC configuration
#define DAC_config_chan_B 0b1011000000000000
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0
uint16_t DAC_data;

// Timer setup
volatile int alarm_period = 50;
volatile int note_start = true;
#define ALARM_NUM 1
#define ALARM_IRQ TIMER_IRQ_1

int note_char_to_index(char *note)
{
    if (strcmp(note, "C4") == 0)
        return 0;
    else if (strcmp(note, "C#4") == 0)
        return 1;
    else if (strcmp(note, "D4") == 0)
        return 2;
    else if (strcmp(note, "D#4") == 0)
        return 3;
    else if (strcmp(note, "E4") == 0)
        return 4;
    else if (strcmp(note, "F4") == 0)
        return 5;
    else if (strcmp(note, "F#4") == 0)
        return 6;
    else if (strcmp(note, "G4") == 0)
        return 7;
    else if (strcmp(note, "G#4") == 0)
        return 8;
    else if (strcmp(note, "A4") == 0)
        return 9;
    else if (strcmp(note, "A#4") == 0)
        return 10;
    else if (strcmp(note, "B4") == 0)
        return 11;
    else if (strcmp(note, "C5") == 0)
        return 12;
    else if (strcmp(note, "C#5") == 0)
        return 13;
    else if (strcmp(note, "D5") == 0)
        return 14;
    else if (strcmp(note, "D#5") == 0)
        return 15;
    else if (strcmp(note, "E5") == 0)
        return 16;
    else if (strcmp(note, "F5") == 0)
        return 17;
    else if (strcmp(note, "F#5") == 0)
        return 18;
    else if (strcmp(note, "G5") == 0)
        return 19;
    else if (strcmp(note, "G#5") == 0)
        return 20;
    else if (strcmp(note, "A5") == 0)
        return 21;
    else
        return -1; // Invalid note
}

int compute_sample(void)
{
    // Low pass filter
    lowpass_out = muls15x16(damping_coeff, ((string[ptrin] + string[ptrout]) >> 1));

    // Tuning all-pass filter
    tuning_out = muls15x16(eta[current_note], (lowpass_out - last_tune_out)) + last_tune_in;
    last_tune_out = tuning_out;
    last_tune_in = lowpass_out;

    // String feedback
    string[ptrin] = tuning_out;

    // Update and wrap pointers
    if (ptrin == string_length[current_note])
        ptrin = 1;
    else
        ptrin = ptrin + 1;

    if (ptrout == string_length[current_note])
        ptrout = 1;
    else
        ptrout = ptrout + 1;

    // DAC output
    int temp = s15x16_to_int(string[ptrout]);
    return temp;
}

void init_karplus_strong(void)
{
    // Initialize different plucking styles
    for (int i = 0; i < max_string_size; i++)
    {
        // Random pluck
        init_string[0][i] = int_to_s15x16((rand() & 0xfff) - 2047);

        // Lowpassed random
        if (i > 3)
        {
            init_string[1][i] = (init_string[0][i - 3] + init_string[0][i - 2] +
                                 init_string[0][i - 1] + init_string[0][i]) >>
                                2;
        }

        // Gaussian pluck
        init_string[2][i] = float_to_s15x16(2000 * exp(-(float)(i - pluck_pos) *
                                                       (i - pluck_pos) / (pluck_width * pluck_width)));

        // Sawtooth
        init_string[3][i] = float_to_s15x16((float)((i % saw_length) - 10) * 50);
    }
}

void play_guitar_note(char *note)
{
    current_note = note_char_to_index(note);

    // Initialize the string buffer with the chosen pluck style
    for (int i = 0; i <= string_length[current_note]; i++)
    {
        string[i] = init_string[current_pluck][i];
    }
    ptrin = 1;
    ptrout = 2;
    last_tune_out = 0;
    last_tune_in = 0;

    // Determine how many samples to play for a sustained note
    // e.g., for 1 second duration at a sample period of alarm_period (in µs)
    long num_samples_to_play = 250000L / alarm_period; // For 1 second
    // Or, a simpler fixed large number if alarm_period is consistent for sampling rate
    // For example, if alarm_period = 50 (for 20kHz), then 20000 samples for 1 sec.

    for (long i = 0; i < num_samples_to_play; i++)
    {
        int sample = compute_sample();
        DAC_data = DAC_config_chan_B | ((sample + 2048) & 0x0fff);
        // The line "DAC_data |= DAC_config_chan_A;" was redundant and can be removed.

        // spi_write16_blocking is appropriate here if not in an ISR context
        spi_write16_blocking(SPI_PORT, &DAC_data, 1);

        sleep_us(alarm_period);

        // Optional: Add a condition to break if sound decays to silence earlier
        // if (abss15x16(string[ptrout]) < threshold_of_silence) break;
    }

    // Optionally, clear the string buffer after playing to ensure silence
    for (int i = 0; i <= string_length[current_note]; i++)
    {
        string[i] = 0;
    }
}
