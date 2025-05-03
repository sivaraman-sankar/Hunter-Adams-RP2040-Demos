// constants.h
#ifndef CONSTANTS_H // Include guard to prevent multiple inclusions
#define CONSTANTS_H

#include <stdint.h>


// Galton board constants
#define PI 3.14159
#define X_DIMENSION 640
#define Y_DIMENSION 480
#define FRAME_RATE 33000
#define GRAVITY float2fix15(0.75)
#define BALL_RADIUS 4
#define PEG_RADIUS 6
#define VERTICAL_SEPARATION 19
#define HORIZONTAL_SEPARATION 38
#define BOUNCINESS float2fix15(0.5)
#define MAX_PEGS 136 // 0.5*n*(n-1)

// DDS constants
#define sine_table_size 256
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// SPI constants
#define PIN_MISO 4
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7
#define SPI_PORT spi0

// Number of DMA transfers per event
const uint32_t transfer_count = sine_table_size;

// === the fixed point macros ========================================
typedef signed int fix15;
#define multfix15(a, b) ((fix15)((((signed long long)(a)) * ((signed long long)(b))) >> 15))
#define float2fix15(a) ((fix15)((a) * 32768.0)) // 2^15
#define fix2float15(a) ((float)(a) / 32768.0)
#define absfix15(a) abs(a)
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a, b) (fix15)(div_s64s64((((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b > int2fix15(480))
#define hitTop(b) (b < int2fix15(0))
#define hitLeft(a) (a < int2fix15(0))
#define hitRight(a) (a > int2fix15(640))

// State variables for boid

typedef struct
{
    fix15 vx;
    fix15 vy;
    fix15 x;
    fix15 y;
} Boid;


// keysignature 
typedef enum { A_MAJOR, C_MAJOR, G_MAJOR } KeyState;

// array of key signatures 
KeyState key_signatures[] = { A_MAJOR, C_MAJOR, G_MAJOR };

#endif // CONSTANTS_H