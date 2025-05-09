/*
    DJ Synthesizer Pad
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

    ADC Connections:
    - GPIO 26 (ADC 0) <---> Left Potentiometer (0 - 3.3V, White)
    - GPIO 27 (ADC 1) <---> Right Potentiometer (0 - 3.3V, White)

    Serial Connections:
    - GPIO 0 <---> UART RX (Yellow)
    - GPIO 1 <---> UART TX (Orange)
    - RP2040 GND <---> UART GND (Black)
*/

/* ********************************************************************* */
/*                          INITIALIZATION                               */
/* ********************************************************************* */

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

#include "vga16_graphics.h"

#include "backing_drums1.c"
#include "backing_jazzy.c"
#include "backing_rock.c"
#include "backing_mellow.c"

#include "kick.c"
#include "snare.c"
#include "hihat.c"
#include "lowtom.c"
#include "hightom.c"
#include "clap.c"

#include "shared.h"
#include "karplus_strong.h"

/* ===== DEFINITIONS ===== */

// Enum for key scale states
typedef enum
{
    C_MAJOR,
    A_MAJOR,
    G_MAJOR
} KeyState;
#define NUM_KEYS 3
volatile KeyState current_key = C_MAJOR;
char *key_names[3] = {"C", "A", "G"};

// Enum for instrument states
typedef enum
{
    PIANO,
    GUITAR,
    DRUMS
} InstrumentState;

typedef enum
{
    PLAY,
    STOPPED
} PlaybackState;

#define NUM_PLAYBACK_STATES 2
volatile PlaybackState current_playback_state = STOPPED; // Default to stopped

#define NUM_INSTRUMENTS 3
volatile InstrumentState current_instrument = PIANO;
char *instrument_names[3] = {"Piano", "Guitar", "Drums"};

#define NUM_TRACKS 4
int currentTrack = 0;

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
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
#define two32 4294967296.0 // 2^32 (a constant)
#define Fs 50000
#define DELAY 20 // 1/Fs (in microseconds)

// Phase accumulator and phase increment, increment sets output frequency
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0 * two32) / Fs;

// Full major scale frequencies (1 octave) for A, C, and G major
float major_freqs[3][8] = {
    {220.00, 246.94, 277.18, 293.66, 329.63, 370.00, 415.30, 440.00}, // A major: A B C# D E F# G# A
    {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25}, // C major: C D E F G A B C
    {196.00, 220.00, 246.94, 261.63, 293.66, 329.63, 369.99, 392.00}  // G major: G A B C D E F# G
};

volatile int current_note_index = -1;
volatile int active_note = -1;
int active_key_text = 0;

const char *white_keys[] = {
    "C4", "D4", "E4", "F4", "G4", "A4", "B4",
    "C5", "D5", "E5", "F5", "G5", "A5", "B5"};

const struct
{
    const char *name;
    int left_white_index; // index of the white key to the left
} black_keys[] = {
    {"C#4", 0}, {"D#4", 1}, {"F#4", 3}, {"G#4", 4}, {"A#4", 5}, {"C#5", 7}, {"D#5", 8}, {"F#5", 10}, {"G#5", 11}, {"A#5", 12}};

#define sine_table_size 256
fix15 sin_table[sine_table_size];

// Values output to DAC
int DAC_output_0;
int DAC_output_1;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1); // maximum amplitude
fix15 attack_inc;                   // rate at which sound ramps up
fix15 decay_inc;                    // rate at which sound ramps down
fix15 current_amplitude_0 = 0;      // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0;      // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME 250
#define DECAY_TIME 250
#define SUSTAIN_TIME 10000
#define BEEP_DURATION 10500
#define BEEP_REPEAT_INTERVAL 50000

// Fixed-point multiplier range for pitch bending: [0.94, 1.06]
#define FIX15_MIN_PITCH float2fix15(0.94)
#define FIX15_RANGE_PITCH float2fix15(0.12)
#define BEND_THRESHOLD float2fix15(0.001)

// State machine variables
volatile unsigned int STATE_0 = 0;
volatile unsigned int count_0 = 0;

// SPI data
uint16_t DAC_data_1; // output value
uint16_t DAC_data_0; // output value

// SPI configurations
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define LDAC 8
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
#define KEYROWS 4
#define NUMKEYS 12

unsigned int keycodes[12] = {0x28, 0x11, 0x21, 0x41, 0x12,
                             0x22, 0x42, 0x14, 0x24, 0x44,
                             0x18, 0x48};
unsigned int scancodes[4] = {0x01, 0x02, 0x04, 0x08};
unsigned int button = 0x70;
volatile fix15 pitch_multiplier_fix15 = float2fix15(1.0); // Default: no pitch bend
volatile int seek_position = 0;
volatile int seek_frame_counter = 0;

char keytext[40];
int prev_key = 0;

// GPIO for timing the ISR
#define ISR_GPIO 2

// DAC config macros
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// Declare DMA channels directly
#define DATA_CHAN_A 10
#define CTRL_CHAN_A 11
#define DATA_CHAN_B 8
#define CTRL_CHAN_B 9

// uS per frame
#define FRAME_RATE 33000 // ~30 fps
#define FRAME_RATE2 66000 // ~60 fps

// Screen dimensions
#define X_DIMENSION 640
#define Y_DIMENSION 480

char current_pressed_note[4] = "X"; // Default to C4
int current_pressed_drums = 0;      // Default to 0, ranges from [0,5]

// Backing track struct
typedef struct
{
    const unsigned short *data;
    const unsigned int length;
} AudioTrack;

AudioTrack tracks[] = {
    {backing_drums1, backing_drums1_len},
    {backing_jazzy, backing_jazzy_len},
    {backing_rock, backing_rock_len},
    {backing_mellow, backing_mellow_len},
};

const int track_total_frames[] = {
    182, // Drums: 6.00 sec * 30.3 fps
    162, // Jazzy: 5.36 sec * 30.3 fps
    265, // Rock: 8.74 sec * 30.3 fps
    243, // Mellow: 8.03 sec * 30.3 fps
};

typedef struct
{
    const unsigned short *data;
    const unsigned int length;
} DrumSample;

DrumSample drums[] = {
    {kick, kick_len},
    {snare, snare_len},
    {hihat, hihat_len},
    {lowtom, lowtom_len},
    {hightom, hightom_len},
    {clap, clap_len}};

#define NUM_DRUMS (sizeof(drums) / sizeof(drums[0]))

/* ********************************************************************* */
/*                         HELPER FUNCTIONS                              */
/* ********************************************************************* */

/* ===== VGA STATE HELPERS ===== */

void set_current_instrument(uint8_t instrument)
{
    if (instrument < NUM_INSTRUMENTS)
    {
        current_instrument = (InstrumentState)instrument;
    }
    else
    {
        current_instrument = PIANO; // default fallback
    }
}

void set_current_pressed_note(int note)
{
    switch (current_key)
    {
    case 0:
        // C Major
        switch (note)
        {
        case 1:
            strcpy(current_pressed_note, "C4");
            break;
        case 2:
            strcpy(current_pressed_note, "D4");
            break;
        case 3:
            strcpy(current_pressed_note, "E4");
            break;
        case 4:
            strcpy(current_pressed_note, "F4");
            break;
        case 5:
            strcpy(current_pressed_note, "G4");
            break;
        case 6:
            strcpy(current_pressed_note, "A4");
            break;
        case 7:
            strcpy(current_pressed_note, "B4");
            break;
        case 8:
            strcpy(current_pressed_note, "C5");
            break;
        }
        break;

    case 1:
        // A Major
        switch (note)
        {
        case 1:
            strcpy(current_pressed_note, "A4");
            break;
        case 2:
            strcpy(current_pressed_note, "B4");
            break;
        case 3: // c is mapped to C#
            strcpy(current_pressed_note, "C#5");
            break;
        case 4:
            strcpy(current_pressed_note, "D5");
            break;
        case 5:
            strcpy(current_pressed_note, "E5");
            break;
        case 6:
            strcpy(current_pressed_note, "F#5");
            break;
        case 7:
            strcpy(current_pressed_note, "G#5");
            break;
        case 8:
            strcpy(current_pressed_note, "A5");
            break;
        }

        break;

    case 2:
        // G Major: G4, A4, B4, C5, D5, E5, F#5, G5
        switch (note)
        {
        case 1:
            strcpy(current_pressed_note, "G4");
            break;
        case 2:
            strcpy(current_pressed_note, "A4");
            break;
        case 3:
            strcpy(current_pressed_note, "B4");
            break;
        case 4:
            strcpy(current_pressed_note, "C5");
            break;
        case 5:
            strcpy(current_pressed_note, "D5");
            break;
        case 6:
            strcpy(current_pressed_note, "E5");
            break;
        case 7:
            strcpy(current_pressed_note, "F#5");
            break;
        case 8:
            strcpy(current_pressed_note, "G5");
            break;
        }
        break;
    default:
        break;
    }
}

char *get_current_pressed_note()
{
    return current_pressed_note;
}

void poll_queue_step()
{
    if (multicore_fifo_rvalid())
    {
        uint32_t raw = multicore_fifo_pop_blocking();
        StateMessage msg = *(StateMessage *)&raw; // Bitcast back to struct

        switch (msg.type)
        {
        case MSG_NOTE_CHANGE:
            printf("\n[Core 1] Displaying note %c on VGA\n", msg.payload);
            // msg.payload will be a char of '1' , '2' ...
            int msg_payload = msg.payload - '0';   // Convert char to int
            set_current_pressed_note(msg_payload); // Set the current pressed note
            break;
        case MSG_KEY_SIGNATURE_CHANGE:
            // printf("\n[Core 1] Key signature changed to %s\n", msg.payload);
            // set_current_key_signature(msg.payload);
            break;
        case MSG_BACKING_TRACK_CHANGE:
            // Handle other types if needed
            break;
        case MSG_DRUMS_CHANGE:
            printf("\n[Core 1] Playing drum %d\n", msg.payload);
            current_pressed_drums = msg.payload - '0'; // Convert char to int
            break;
        case MSG_INSTRUMENT_CHANGE:
            set_current_instrument((uint8_t)msg.payload);
            break;
        default:
            printf("[Core 1] Unknown message type\n");
        }
    }
}

void send_note_to_vga(MessageType type, char payload)
{
    StateMessage msg;
    msg.type = type;
    msg.payload = payload;
    multicore_fifo_push_blocking(*(uint32_t *)&msg); // Bitcast struct as 32-bit
    return;
}

/* ===== ADC HELPERS ===== */

void setupADC0()
{
    adc_init();
    adc_gpio_init(ADC0_PIN);
    adc_select_input(ADC0_CHAN);
}

void setupADC1()
{
    adc_init();
    adc_gpio_init(ADC1_PIN);
    adc_select_input(ADC1_CHAN);
}

void setupADC2()
{
    adc_init();
    adc_gpio_init(ADC2_PIN);
    adc_select_input(ADC2_CHAN);
}

/* ===== KEYPAD HELPER ===== */

void setupKeypad()
{
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

/* ===== TIMER ISR FOR PIANO PLAYING ===== */

static void alarm_irq(void)
{

    // Assert a GPIO when we enter the interrupt
    gpio_put(ISR_GPIO, 1);

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    if (active_note >= 0 && active_note < 8 && current_instrument == PIANO)
    {
        float base_freq = major_freqs[current_key][active_note];
        float adjusted_freq = base_freq * fix2float15(pitch_multiplier_fix15);
        phase_incr_main_0 = (unsigned int)((adjusted_freq * two32) / Fs);
        phase_accum_main_0 += phase_incr_main_0;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                                           sin_table[phase_accum_main_0 >> 24])) +
                       2048;

        // Maintain full amplitude
        if (current_amplitude_0 < max_amplitude)
        {
            current_amplitude_0 += attack_inc;
            if (current_amplitude_0 > max_amplitude)
                current_amplitude_0 = max_amplitude;
        }

        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
    }

    else
    {
        // No key held, ramp down amplitude
        if (current_amplitude_0 > 0)
        {
            current_amplitude_0 -= decay_inc;
            if (current_amplitude_0 < 0)
                current_amplitude_0 = 0;

            phase_accum_main_0 += phase_incr_main_0;
            DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
                                               sin_table[phase_accum_main_0 >> 24])) +
                           2048;

            DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff));
            spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
        }
    }

    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

/* ===== DMA HELPERS ===== */

void setupBackingTrackDMA()
{
    // Backing Track DMA Setup (Left Speaker, DAC Channel A)

    dma_channel_config c_ctrl_A = dma_channel_get_default_config(CTRL_CHAN_A);
    channel_config_set_transfer_data_size(&c_ctrl_A, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl_A, false);
    channel_config_set_write_increment(&c_ctrl_A, false);
    channel_config_set_chain_to(&c_ctrl_A, DATA_CHAN_A);

    dma_channel_configure(
        CTRL_CHAN_A, &c_ctrl_A,
        &dma_hw->ch[DATA_CHAN_A].read_addr,
        (void *)&tracks[currentTrack].data,
        1, false);

    dma_channel_config c_data_A = dma_channel_get_default_config(DATA_CHAN_A);
    channel_config_set_transfer_data_size(&c_data_A, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data_A, true);
    channel_config_set_write_increment(&c_data_A, false);

    // Set DMA Timer 0 for 22.05 kHz playback (250 MHz × 2 / 22673 = 22050 Hz)
    dma_timer_set_fraction(0, 2, 22673);

    channel_config_set_dreq(&c_data_A, 0x3B);
    channel_config_set_chain_to(&c_data_A, CTRL_CHAN_A);

    dma_channel_configure(
        DATA_CHAN_A, &c_data_A,
        &spi_get_hw(SPI_PORT)->dr,
        tracks[currentTrack].data,
        tracks[currentTrack].length,
        false);
}

void playback()
{
    if (current_playback_state == STOPPED)
    {
        // If stopped, just return
        return;
    }
    // Start playback
    dma_channel_set_read_addr(CTRL_CHAN_A, (void *)&tracks[currentTrack].data, false);
    dma_channel_set_read_addr(DATA_CHAN_A, tracks[currentTrack].data, false);
    dma_channel_set_trans_count(DATA_CHAN_A, tracks[currentTrack].length, false);

    // Start only the control channel — this will chain into data channel
    dma_channel_start(CTRL_CHAN_A);
}

void stopPlayback()
{
    // Stop playback
    dma_channel_abort(DATA_CHAN_A);
    dma_channel_abort(CTRL_CHAN_A);
}

void setupDrumsDMA()
{
    // Control channel B setup
    dma_channel_config c_ctrl_B = dma_channel_get_default_config(CTRL_CHAN_B);
    channel_config_set_transfer_data_size(&c_ctrl_B, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl_B, false);
    channel_config_set_write_increment(&c_ctrl_B, false);
    channel_config_set_chain_to(&c_ctrl_B, DATA_CHAN_B);

    dma_channel_configure(
        CTRL_CHAN_B, &c_ctrl_B,
        &dma_hw->ch[DATA_CHAN_B].read_addr,
        (void *)&drums[0].data, // dummy default, will change when triggered
        1, false);

    // Data channel B setup
    dma_channel_config c_data_B = dma_channel_get_default_config(DATA_CHAN_B);
    channel_config_set_transfer_data_size(&c_data_B, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data_B, true);
    channel_config_set_write_increment(&c_data_B, false);

    // Set DMA Timer 0 for 22.05 kHz playback (250 MHz × 2 / 22673 = 22050 Hz)
    dma_timer_set_fraction(0, 2, 22673);

    // Use same DMA timer 0 (0x3B), same frequency
    channel_config_set_dreq(&c_data_B, 0x3B);
    channel_config_set_chain_to(&c_data_B, DATA_CHAN_B);

    dma_channel_configure(
        DATA_CHAN_B, &c_data_B,
        &spi_get_hw(SPI_PORT)->dr,
        drums[0].data,
        drums[0].length,
        false);
}

void playDrum(int drumIndex)
{
    if (drumIndex < 0 || drumIndex >= NUM_DRUMS)
        return;

    printf("\nPlaying drum %d\n", drumIndex);

    // Abort any previous transfer
    dma_channel_abort(DATA_CHAN_B);
    dma_channel_abort(CTRL_CHAN_B);

    // Clear any pending interrupt flags (optional but clean)
    dma_channel_acknowledge_irq0(DATA_CHAN_B);
    dma_channel_acknowledge_irq0(CTRL_CHAN_B);

    // Setup new transfer
    dma_channel_set_read_addr(CTRL_CHAN_B, &drums[drumIndex].data, false);
    dma_channel_set_trans_count(CTRL_CHAN_B, 1, false);
    dma_channel_set_read_addr(DATA_CHAN_B, drums[drumIndex].data, false);
    dma_channel_set_trans_count(DATA_CHAN_B, drums[drumIndex].length, false);

    // Start playback
    dma_start_channel_mask((1u << CTRL_CHAN_B));
}

/* ********************************************************************* */
/*                              THREADS                                  */
/* ********************************************************************* */

/* ===== KEYPAD THREAD ===== */

static PT_THREAD(thread_keypad_input(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt);

    // Some variables
    static int i;
    static uint32_t keypad;

    // Add static variables for debounce
    static int possible = 0;
    static enum { NOT_PRESSED,
                  MAYBE_PRESSED,
                  PRESSED,
                  MAYBE_NOT_PRESSED } BOUNCE_STATE = NOT_PRESSED;

    while (1)
    {
        // Scan the keypad!
        for (i = 0; i < KEYROWS; i++)
        {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN));
            // Small delay required
            sleep_us(1);
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F);
            // Break if button(s) are pressed
            if (keypad & button)
                break;
        }
        // If we found a button . . .
        if (keypad & button)
        {
            // Look for a valid keycode.
            for (i = 0; i < NUMKEYS; i++)
            {
                if (keypad == keycodes[i])
                    break;
            }
            // If we don't find one, report invalid keycode
            if (i == NUMKEYS)
                (i = -1);
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else
            (i = -1);

        // Print key to terminal
        // printf("\nKey pressed: %d", i);

        // Update active note
        if (i >= 1 && i <= 8)
        {
            active_note = i - 1;

            // printf("\nActive note: %d", active_note);
            // Send note to VGA via multicore FIFO method
            // convert to char ie. 1 to '1' , 2 to '2', etc.
            char active_key_text = '0' + (char)(active_note + 1);
            send_note_to_vga(MSG_NOTE_CHANGE, active_key_text);
        }
        else
        {
            active_note = -1;
        }

        // Keypad debouncing FSM
        switch (BOUNCE_STATE)
        {
        case NOT_PRESSED:
            possible = i;
            BOUNCE_STATE = MAYBE_PRESSED;
            break;

        case MAYBE_PRESSED:
            if (i == possible)
            {
                //
                if (possible == 1 && current_instrument == DRUMS)
                {
                    playDrum(0); // 0 = kick
                    send_note_to_vga(MSG_DRUMS_CHANGE, '0');
                }
                if (possible == 2 && current_instrument == DRUMS)
                {
                    playDrum(1); // 1 = snare
                    send_note_to_vga(MSG_DRUMS_CHANGE, '1');
                }
                if (possible == 3 && current_instrument == DRUMS)
                {
                    playDrum(2); // 2 = hihat
                    send_note_to_vga(MSG_DRUMS_CHANGE, '2');
                }
                if (possible == 4 && current_instrument == DRUMS)
                {
                    playDrum(3); // 3 = low tom
                    send_note_to_vga(MSG_DRUMS_CHANGE, '3');
                }
                if (possible == 5 && current_instrument == DRUMS)
                {
                    playDrum(4); // 4 = high tom
                    send_note_to_vga(MSG_DRUMS_CHANGE, '4');
                }
                if (possible == 6 && current_instrument == DRUMS)
                {
                    playDrum(5); // 5 = clap
                    send_note_to_vga(MSG_DRUMS_CHANGE, '5');
                }
                if (possible == 9)
                {
                    // 9 Button -> change scale
                    current_key = (current_key + 1) % NUM_KEYS;
                }
                if (possible == 10)
                {
                    // * Button -> change backing track
                    currentTrack = (currentTrack + 1) % NUM_TRACKS;
                    seek_position = 0;
                    seek_frame_counter = 0;
                    stopPlayback();
                    setupBackingTrackDMA();
                    if (current_playback_state == PLAY)
                    {
                        playback();
                    }
                }
                if (possible == 11)
                {
                    // # Button -> change instrument
                    current_instrument = (current_instrument + 1) % NUM_INSTRUMENTS;
                }
                if (possible == 0)
                {
                    // 0 Button -> stop/play
                    current_playback_state = (current_playback_state + 1) % NUM_PLAYBACK_STATES;
                    if (current_playback_state == PLAY)
                    {
                        stopPlayback();
                        playback();
                    }
                    else
                    {
                        // Stop the backing track
                        stopPlayback();
                    }
                }
                BOUNCE_STATE = PRESSED;
            }
            else
            {
                BOUNCE_STATE = NOT_PRESSED;
            }
            break;

        case PRESSED:
            if (i != possible)
            {
                BOUNCE_STATE = MAYBE_NOT_PRESSED;
            }
            break;

        case MAYBE_NOT_PRESSED:
            if (i == possible)
            {
                BOUNCE_STATE = PRESSED;
            }
            else
            {
                BOUNCE_STATE = NOT_PRESSED;
            }
            break;
        }

        PT_YIELD_usec(30000);
    }
    PT_END(pt);
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
        if (absfix15(new_fix_mult - last_fix_mult) >= BEND_THRESHOLD)
        {
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

/* ===== GUITAR THREAD ===== */

static PT_THREAD(thread_guitar(struct pt *pt))
{
    PT_BEGIN(pt);

    while (1)
    {
        if (current_instrument == GUITAR && active_note >= 0 && active_note < 8)
        {
            // Set the phase increment for the guitar
            char *current_pressed_note = get_current_pressed_note();
            play_guitar_note(current_pressed_note);
        }

        PT_YIELD_usec(30000);
    }

    PT_END(pt);
}

/* ===== POLL QUEUE THREAD ===== */

static PT_THREAD(thread_poll_queue(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1)
    {
        poll_queue_step();
        PT_YIELD(pt);
    }
    PT_END(pt);
}

/* ===== PIANO KEY VISUALIZATION VGA THREAD ===== */

static PT_THREAD(protothread_keys(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;

    int x_offset = 265;
    int y_offset = (Y_DIMENSION / 2) - 24;

    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();
        char *current_pressed_note = get_current_pressed_note();

        if (current_instrument == DRUMS)
        {
            strcpy(current_pressed_note, "X");
        }

        // Draw white keys
        for (int i = 0; i < 14; i++)
        {
            const char *note = white_keys[i];
            int x = x_offset + i * 21;

            // Determine top alignment style
            int top_x;
            if (strcmp(note, "C4") == 0 || strcmp(note, "F4") == 0 ||
                strcmp(note, "C5") == 0 || strcmp(note, "F5") == 0)
            {
                top_x = x; // No black key to its left
            }
            else
            {
                top_x = x + 5; // Black key to its left
            }

            uint16_t color = strcmp(current_pressed_note, note) == 0 ? ORANGE : WHITE;
            if (strcmp(note, "D4") == 0 || strcmp(note, "G4") == 0 || strcmp(note, "A4") == 0 || 
                strcmp(note, "D5") == 0 || strcmp(note, "G5") == 0 || strcmp(note, "A5") == 0)
                {
                    fillRect(top_x, y_offset, 10, 60, color);  // top portion for keys between two black notes
                }
            else {
                fillRect(top_x, y_offset, 15, 60, color);  // top portion for all other keys
            }
            fillRect(x, y_offset + 60, 20, 40, color); // bottom portion for all keys
        }

        // Draw black keys
        for (int i = 0; i < 10; i++)
        {
            int left_white_x = x_offset + black_keys[i].left_white_index * 21;
            int black_x = left_white_x + 15; // centered between white keys
            uint16_t color = strcmp(current_pressed_note, black_keys[i].name) == 0 ? ORANGE : BLACK;
            fillRect(black_x, y_offset, 11, 60, color);
        }

        spare_time = FRAME_RATE2 - (time_us_32() - begin_time);
        PT_YIELD_usec(spare_time);
    }
    PT_END(pt);
} 

/* ===== DRUM PAD VISUALIZATION VGA THREAD ===== */

static PT_THREAD(protothread_vga_drums(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;

    // x,y offsets for the drums positions
    int x_offset = 2 * (X_DIMENSION / 10) - 40;
    int y_offset = (Y_DIMENSION / 2) - 25;
    int drum_size = 50;
    int spacing = drum_size + 2;

    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();

        // First row: drums 0, 1, 2
        fillRect(x_offset, y_offset, drum_size, drum_size, current_pressed_drums == 0 ? WHITE : LIGHT_PINK);
        setCursor(x_offset + 14, y_offset + 23); // center "KICK"
        setTextColor2(BLACK, current_pressed_drums == 0 ? WHITE : LIGHT_PINK);
        setTextSize(1);
        writeString("KICK");

        fillRect(x_offset + spacing, y_offset, drum_size, drum_size, current_pressed_drums == 1 ? WHITE : BLUE);
        setCursor(x_offset + spacing + 9, y_offset + 23); // center "SNARE"
        setTextColor2(BLACK, current_pressed_drums == 1 ? WHITE : BLUE);
        writeString("SNARE");

        fillRect(x_offset + 2 * spacing, y_offset, drum_size, drum_size, current_pressed_drums == 2 ? WHITE : RED);
        setCursor(x_offset + 2 * spacing + 7, y_offset + 23); // center "HI-HAT"
        setTextColor2(BLACK, current_pressed_drums == 2 ? WHITE : RED);
        writeString("HI-HAT");

        // Second row: drums 3, 4, 5
        fillRect(x_offset, y_offset + spacing, drum_size, drum_size, current_pressed_drums == 3 ? WHITE : MED_GREEN);
        setCursor(x_offset + 11, y_offset + spacing + 21); // center "TOM2"
        setTextColor2(BLACK, current_pressed_drums == 3 ? WHITE : MED_GREEN);
        writeString("TOM 2");

        fillRect(x_offset + spacing, y_offset + spacing, drum_size, drum_size, current_pressed_drums == 4 ? WHITE : ORANGE);
        setCursor(x_offset + spacing + 11, y_offset + spacing + 21); // center "TOM1"
        setTextColor2(BLACK, current_pressed_drums == 4 ? WHITE : ORANGE);
        writeString("TOM 1");

        fillRect(x_offset + 2 * spacing, y_offset + spacing, drum_size, drum_size, current_pressed_drums == 5 ? WHITE : MAGENTA);
        setCursor(x_offset + 2 * spacing + 13, y_offset + spacing + 21); // center "CLAP"
        setTextColor2(BLACK, current_pressed_drums == 5 ? WHITE : MAGENTA);
        writeString("CLAP");

        // reset the drum state to -1
        current_pressed_drums = -1;

        // Frame rate control
        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        PT_YIELD_usec(spare_time);
    }

    PT_END(pt);
}

/* ===== TITLE AND INSTRUCTION VGA THREAD ===== */

static PT_THREAD(protothread_vga_title(struct pt *pt))
{
    PT_BEGIN(pt);

    static int begin_time;
    static int spare_time;
    static uint32_t instructions_start_time = 0;
    static bool title_drawn = false;
    static const int title_x_offset = 140;

    static char *
        instructions[] = {
            "Press # to change instrument and * to change backing track.",
            "In piano or guitar mode, press keys 1-8 to play notes, or press 9 to change key signatures.",
            "Use the knob to bend piano notes, and press keys 1-6 to play drums.",
            "Press 0 to play or stop the backing track.",
            "NOW GO MAKE SOME MUSIC AND HAVE FUN!"};
    static const int instructions_start_y = 102;
    const int num_instructions = sizeof(instructions) / sizeof(instructions[0]);

    while (1)
    {
        begin_time = time_us_32();

        // Draw title and instructions once
        if (!title_drawn)
        {
            // Draw title
            setCursor((X_DIMENSION / 2) - title_x_offset, 50);
            setTextSize(4);
            setTextColor2(WHITE, BLACK);
            writeString("DJ Synth Pad");

            // Draw instructions once
            setTextSize(1);
            setTextColor2(PINK, BLACK);
            int center_offset = 0;
            int title_start = X_DIMENSION / 2 - title_x_offset;
            for (int i = 0; i < num_instructions; i++)
            {
                int y_pos = instructions_start_y + (20 * i);

                switch (i)
                {
                case 0:
                    center_offset = -35;
                    break;
                case 1:
                    center_offset = -130;
                    break;
                case 2:
                    center_offset = -55;
                    break;
                case 3:
                    center_offset = 15;
                    break;
                case 4:
                    center_offset = 37;
                    setTextColor2(GREEN, BLACK);
                    y_pos = y_pos + 8;
                    break;
                default:
                    center_offset = 0;
                    break;
                }

                setCursor(title_start + center_offset, y_pos);
                writeString(instructions[i]);
            }

            title_drawn = true;
            instructions_start_time = begin_time;
        }
        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        PT_YIELD_usec(spare_time);
    }
    PT_END(pt);
}

/* ===== TRACK SEEK VISUALIZATION + STATE UPDATE VGA THREAD ===== */

static PT_THREAD(protothread_vga_state(struct pt *pt))
{
    PT_BEGIN(pt);

    static int begin_time;
    static int spare_time;
    static int previous_seek_position = 0; // Store previous circle location
    int x_offset = 240;
    int y_offset = 3 * (Y_DIMENSION / 5) + 45;
    char screentext[40];

    while (1)
    {
        begin_time = time_us_32();

        // Draw seek line
        drawHLine(x_offset, y_offset, 160, WHITE);

        // Erase the previous circle by overdrawing it in black
        fillCircle(x_offset + previous_seek_position, y_offset, 3, BLACK);

        // Update seek position
        if (current_playback_state == PLAY) {
            seek_frame_counter++;
            if (seek_frame_counter >= track_total_frames[currentTrack]) {
                seek_frame_counter = 0;
            }
            seek_position = (160 * seek_frame_counter) / track_total_frames[currentTrack];
        } else {
            seek_frame_counter = 0;
            seek_position = 0;
        }

        // Draw the new circle
        fillCircle(x_offset + seek_position, y_offset, 3, WHITE);

        // Update the previous position
        previous_seek_position = seek_position;

        // Print play/pause state
        setCursor(x_offset, y_offset + 15);
        setTextSize(1);
        setTextColor2(current_playback_state == PLAY ? GREEN : RED, BLACK);
        writeString(current_playback_state == PLAY ? "<<      Playing...      >>" : "<<       Stopped!       >>");

        // Print backing track name
        setCursor(x_offset + 18, y_offset + 35);
        setTextSize(1);
        setTextColor2(WHITE, BLACK);
        sprintf(screentext, "Backing Track: %s      ",
            (currentTrack == 0) ? "Drums" :
            (currentTrack == 1) ? "Jazzy" :
            (currentTrack == 2) ? "Rock" : 
            (currentTrack == 3) ? "Mellow" : "None");
        writeString(screentext);

        // Print key signature
        setCursor(x_offset + 30, y_offset + 55);
        setTextSize(1);
        setTextColor2(YELLOW, BLACK);
        sprintf(screentext, "Key Signature: %s      ", key_names[current_key]);
        writeString(screentext);

        // Print instrument
        setCursor(x_offset + 26, y_offset + 75);
        setTextSize(1);
        setTextColor2(BLUE, BLACK);
        sprintf(screentext, "Instrument: %s      ", instrument_names[current_instrument]);
        writeString(screentext);

        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        PT_YIELD_usec(spare_time);
    }

    PT_END(pt);
}

/* ********************************************************************* */
/*                          MAIN FUNCTIONS                               */
/* ********************************************************************* */

/* ===== CORE 1 MAIN FUNCTION ===== */

void core1_main()
{
    // add all VGA threads
    pt_add_thread(thread_poll_queue); // poll queue thread
    pt_add_thread(protothread_vga_title);
    pt_add_thread(protothread_vga_state);
    pt_add_thread(protothread_vga_drums);
    pt_add_thread(protothread_keys);
    // Start the scheduler
    pt_schedule_start;
}

/* ===== MAIN ===== */

int main()
{
    // Overclock the chip to 250MHz
    set_sys_clock_khz(250000, true);

    stdio_init_all();

    /* ===== Setup VGA, ADC, and Keypad ===== */

    initVGA();
    setupADC0();
    setupKeypad();

    /* ===== Setup SPI/DAC ===== */

    spi_init(SPI_PORT, 20000000);          // baud rate set to 20MHz
    spi_set_format(SPI_PORT, 16, 0, 0, 0); // (channel, data bits per transfer, polarity, phase, order)

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC);
    gpio_set_dir(LDAC, GPIO_OUT);
    gpio_put(LDAC, 0);

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO);
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0);

    /* ===== SETUP PIANO (DDS) + GUITAR (KARPLUS STRONG) SYNTHESIS ===== */

    // set up increments for calculating envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME));
    decay_inc = divfix(max_amplitude, int2fix15(DECAY_TIME));

    // Build the sine lookup table scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++)
    {
        sin_table[ii] = float2fix15(2047 * sin((float)ii * 6.283 / (float)sine_table_size));
    }

    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true);
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

    init_karplus_strong();

    /* ===== Setup DMA ===== */

    setupBackingTrackDMA();
    setupDrumsDMA();
    playback();

    /* ===== Add Core 0 Threads ===== */

    pt_add_thread(thread_keypad_input);
    pt_add_thread(thread_pitch_bend);
    pt_add_thread(thread_guitar);

    /* ===== Start Core 1 + Scheduler ===== */

    multicore_reset_core1();
    multicore_launch_core1(&core1_main);
    pt_schedule_start;
}