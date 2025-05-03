/**
 * dma-playback.c  
 * Nicholas Papapanou (ngp37)
 * Based on dma-demo.c by V. Hunter Adams (vha3)
 *
 * Sets up two independent DMA chains to output audio via the MCP4822 DAC:
 *  - Channel A (left speaker): Plays a converted .wav backing track from flash using DMA Timer 0
 *  - Channel B (right speaker): Loops a sine wave using DMA Timer 1
 *
 * The backing track is preprocessed into a const array using convertWav.py
 * and included in its own .c file. Both channels run continuously via DMA chaining.
*/

/* ===== IMPORTS ===== */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/spi.h"
#include "backing_drums1.c"

/* ===== DEFINITIONS ===== */

// Number of samples per period in sine table
#define SINE_TABLE_SIZE 256

// SPI configurations
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// DAC config macros
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// Number of DMA transfers per event
const uint32_t transfer_count = SINE_TABLE_SIZE;

// Backing track struct
typedef struct {
    const unsigned short *data;
    const unsigned int length;
} AudioTrack;

AudioTrack tracks[] = {
    { backing_drums1, backing_drums1_len },
    // Add more here
};

#define NUM_TRACKS (sizeof(tracks)/sizeof(tracks[0]))
int currentTrack = 0;

/* ===== GLOBAL DATA (CHANNEL B: Sine Test) ===== */

int raw_sine_B[SINE_TABLE_SIZE];
unsigned short DAC_data_B[SINE_TABLE_SIZE];
unsigned short *address_pointer_B = &DAC_data_B[0];

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

    // Channel B sine wave setup
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        float phase = (float)i * 6.283f / SINE_TABLE_SIZE;
        raw_sine_B[i] = (int)(2047 * sin(phase) + 2047);
        DAC_data_B[i] = DAC_config_chan_B | (raw_sine_B[i] & 0x0FFF);
    }

    /* ===== DMA CHANNEL B: Sine Wave (Right, Channel B) ===== */
    int data_chan_B = dma_claim_unused_channel(true);
    int ctrl_chan_B = dma_claim_unused_channel(true);

    dma_channel_config c_ctrl_B = dma_channel_get_default_config(ctrl_chan_B);
    channel_config_set_transfer_data_size(&c_ctrl_B, DMA_SIZE_32);
    channel_config_set_read_increment(&c_ctrl_B, false);
    channel_config_set_write_increment(&c_ctrl_B, false);
    channel_config_set_chain_to(&c_ctrl_B, data_chan_B);

    dma_channel_configure(
        ctrl_chan_B, &c_ctrl_B,
        &dma_hw->ch[data_chan_B].read_addr,
        &address_pointer_B,
        1, false
    );

    dma_channel_config c_data_B = dma_channel_get_default_config(data_chan_B);
    channel_config_set_transfer_data_size(&c_data_B, DMA_SIZE_16);
    channel_config_set_read_increment(&c_data_B, true);
    channel_config_set_write_increment(&c_data_B, false);
    dma_timer_set_fraction(1, 0x0017, 0xFFFF);  // ~44kHz
    channel_config_set_dreq(&c_data_B, 0x3C);
    channel_config_set_chain_to(&c_data_B, ctrl_chan_B);

    dma_channel_configure(
        data_chan_B, &c_data_B,
        &spi_get_hw(SPI_PORT)->dr,
        DAC_data_B,
        SINE_TABLE_SIZE,
        false
    );
    /* ===== DMA CHANNEL A: Backing Track (Left, Channel A) ===== */
    int data_chan_A = dma_claim_unused_channel(true);
    int ctrl_chan_A = dma_claim_unused_channel(true);

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

    /* ===== START BOTH DMA CHAINS ===== */
    dma_start_channel_mask((1u << ctrl_chan_A) | (1u << ctrl_chan_B));

    // Exit main. No code executing!!

}
