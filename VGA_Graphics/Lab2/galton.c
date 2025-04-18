
/**
 * Hunter Adams (vha3@cornell.edu)
 *
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Core imports
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Other imports
#include "vga16_graphics.h"
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "pt_cornell_rp2040_v1_3.h"
#include "hardware/spi.h"

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
#define MAX_BALLS 100

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

// Ball struct
typedef struct
{
    fix15 vx;
    fix15 vy;
    fix15 x;
    fix15 y;
} Ball;

// DMA stuff

// Number of samples per period in sine table
#define sine_table_size 256

// Sine table
int raw_sin[sine_table_size];

// Table of values to be sent to DAC
unsigned short DAC_data[sine_table_size];

// Pointer to the address of the DAC data table
unsigned short *address_pointer_A = &DAC_data[0];

int ctrl_chan;
int fallen_balls;
int boot_time;

void dma_main()
{

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000);

    // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Build sine table and DAC data table
    int i;

    for (i = 0; i < (sine_table_size); i++)
    {
        raw_sin[i] = (int)(2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047); // 12 bit
        DAC_data[i] = DAC_config_chan_B | (raw_sin[i] & 0x0fff);
    }

    // Select DMA channels
    int data_chan = dma_claim_unused_channel(true);
    ctrl_chan = dma_claim_unused_channel(true);

    // Setup the control channel
    dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
    channel_config_set_read_increment(&c, false);                     // no read incrementing
    channel_config_set_write_increment(&c, false);                    // no write incrementing
    channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

    dma_channel_configure(
        ctrl_chan,                        // Channel to be configured
        &c,                               // The configuration we just created
        &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
        &address_pointer_A,               // Read address (POINTER TO AN ADDRESS)
        1,                                // Number of transfers
        false                             // Don't start immediately
    );

    // Setup the data channel
    dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
    channel_config_set_read_increment(&c2, true);                      // yes read incrementing
    channel_config_set_write_increment(&c2, false);                    // no write incrementing

    // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
    dma_timer_set_fraction(0, 0x0017, 0xffff);

    // 0x3b means timer0 (see SDK manual)
    channel_config_set_dreq(&c2, 0x3b); // DREQ paced by timer 0

    // chain to the controller DMA channel
    // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel

    dma_channel_configure(
        data_chan,                 // Channel to be configured
        &c2,                       // The configuration we just created
        &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
        DAC_data,                  // The initial read address
        sine_table_size,           // Number of transfers
        false                      // Don't start immediately.
    );

    // start the control channel
    // dma_start_channel_mask(1u << ctrl_chan) ;
}

void beep(int ctrl_chan)
{
    dma_channel_start(ctrl_chan);
}

// -- end of DMA ---

// State variables
char ball_color = WHITE;
char peg_color = GREEN;
int positions[MAX_PEGS][2];
int histogram[16];
int pos_count = 0;
int ball_count = 30;
int last_collided_peg = -1;

// Initialize balls array w/ max size
Ball balls[MAX_BALLS];

void spawnBall(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
    *x = int2fix15(X_DIMENSION / 2);
    *y = int2fix15(0);
    *vx = float2fix15(((rand() / (float)RAND_MAX) - 0.5));
    // *vx = ((rand() & 0xffff) >> 16) - float2fix15(0.5);
    *vy = int2fix15(0);
}

// Spawn all balls
void spawnAllBalls()
{
    for (int i = 0; i < ball_count; i++)
    {
        spawnBall(&balls[i].x, &balls[i].y, &balls[i].vx, &balls[i].vy);
    }
}

// Get all peg poisitons
void get_all_peg_positions(int num_rows, int center_x)
{
    for (int row = 0; row < num_rows; row++)
    {
        int row_size = row + 1;
        int start_x = center_x - (((row_size - 1) * HORIZONTAL_SEPARATION) / 2);

        for (int peg = 0; peg < row_size; peg++)
        {
            if (pos_count < MAX_PEGS)
            {
                positions[pos_count][0] = start_x + peg * HORIZONTAL_SEPARATION;
                positions[pos_count][1] = 25 + row * VERTICAL_SEPARATION;
                pos_count++;
            }
        }
    }
}

// Update ball velocity based on collision logic
void update_ball_velocity(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, fix15 peg_x, fix15 peg_y, int current_peg)
{
    // collision detection
    fix15 dx = *x - peg_x;
    fix15 dy = *y - peg_y;

    fix15 delta = int2fix15(BALL_RADIUS) + int2fix15(PEG_RADIUS);

    if (absfix15(dx) < (delta) && absfix15(dy) < (delta))
    {

        fix15 distance = float2fix15(sqrt(fix2float15(multfix15(dx, dx)) + fix2float15(multfix15(dy, dy))));
        fix15 norm_x = divfix(dx, distance);
        fix15 norm_y = divfix(dy, distance);
        fix15 intermediate = multfix15(float2fix15(-2), (multfix15(norm_x, *vx) + multfix15(norm_y, *vy)));

        if (intermediate > 0)
        {
            *x = peg_x + multfix15(norm_x, (distance + int2fix15(1)));
            *y = peg_y + multfix15(norm_y, (distance + int2fix15(1)));

            *vx = *vx + multfix15(norm_x, intermediate);
            *vy = *vy + multfix15(norm_y, intermediate);

            // printf("Current peg: %d", current_peg);

            // make sound
            if (last_collided_peg != current_peg)
            {
                last_collided_peg = current_peg;
                // printf("Last collided: %d \n", last_collided_peg);
                beep(ctrl_chan);
            }
            *vx = multfix15(*vx, BOUNCINESS);
            *vy = multfix15(*vy, BOUNCINESS);
        }
    }
}

int getBucket(int x)
{
    // Use range comparisons to determine the bucket
    switch (x)
    {
    case 0 ... 39:
        return 0; // 0 <= x < 40
    case 40 ... 79:
        return 1; // 40 <= x < 80
    case 80 ... 119:
        return 2; // 80 <= x < 120
    case 120 ... 159:
        return 3; // 120 <= x < 160
    case 160 ... 199:
        return 4; // 160 <= x < 200
    case 200 ... 239:
        return 5; // 200 <= x < 240
    case 240 ... 279:
        return 6; // 240 <= x < 280
    case 280 ... 319:
        return 7; // 280 <= x < 320
    case 320 ... 359:
        return 8; // 320 <= x < 360
    case 360 ... 399:
        return 9; // 360 <= x < 400
    case 400 ... 439:
        return 10; // 400 <= x < 440
    case 440 ... 479:
        return 11; // 440 <= x < 480
    case 480 ... 519:
        return 12; // 480 <= x < 520
    case 520 ... 559:
        return 13; // 520 <= x < 560
    case 560 ... 599:
        return 14; // 560 <= x < 600
    case 600 ... 639:
        return 15; // 600 <= x < 640
    default:
        return -1; // In case something goes wrong (this should not happen)
    }
}

int get_histogram_index(int x)
{
    int bucketNum = getBucket(x);
    // printf("For %d, the bucket is %d", x, bucketNum);
    return bucketNum;
}

// Update ball position based on updated velocity
void update_ball_position(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
    if (hitBottom(*y))
    {
        // Start at top of screen (640 width x 480 height screen)
        // *x = int2fix15(320);
        // *y = int2fix15(0);
        // // Randomized (small) x-velocity between -0.5 and 0.5
        // // *vx = float2fix15(((rand() / (float)RAND_MAX) - 0.5));
        // *vx = float2fix15((rand() & 0xffff) >> 16) - float2fix15(0.5);

        // *vy = int2fix15(0);
        histogram[get_histogram_index(fix2int15(*x))] += 1;
        spawnBall(x, y, vx, vy);
        fallen_balls = fallen_balls + 1;
    }

    //
    *vy += GRAVITY;
    *x += *vx;
    *y += *vy;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy)
{
    // Reverse direction if we've hit a wall
    if (hitTop(*y))
    {
        *vy = (-*vy);
        *y = (*y + int2fix15(5));
    }
    if (hitBottom(*y))
    {
        *vy = (-*vy);
        *y = (*y - int2fix15(5));
    }
    if (hitRight(*x))
    {
        *vx = (-*vx);
        *x = (*x - int2fix15(5));
    }
    if (hitLeft(*x))
    {
        *vx = (-*vx);
        *x = (*x + int2fix15(5));
    }

    // Update position using velocity
    *x = *x + *vx;
    *y = *y + *vy;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000);
    // announce the threader version
    // sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write;
    while (1)
    {
        // print prompt
        // sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
        // non-blocking write
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%d", &user_input);
        // update boid color
        if ((user_input > 0) && (user_input < 16))
        {
            ball_color = (char)user_input;
        }
    } // END WHILE(1)
    PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD(protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;

    // Spawn all the boids
    spawnAllBalls();

    while (1)
    {
        begin_time = time_us_32();

        // erase balls
        // for (int j = 0; j < ball_count; j++)
        // {
        //     fillCircle(fix2int15(balls[j].x), fix2int15(balls[j].y), BALL_RADIUS, BLACK);
        // }

        // DRAW pegs
        for (int i = 0; i < MAX_PEGS; i++)
        {
            fillCircle(positions[i][0], positions[i][1], PEG_RADIUS, peg_color);
        }

        for (int j = 0; j < ball_count; j++)
        {
            fillCircle(fix2int15(balls[j].x), fix2int15(balls[j].y), BALL_RADIUS, BLACK);
            for (int i = 0; i < MAX_PEGS; i++)
            {
                fix15 current_peg_x = int2fix15(positions[i][0]);
                fix15 current_peg_y = int2fix15(positions[i][1]);
                update_ball_velocity(&balls[j].x, &balls[j].y, &balls[j].vx, &balls[j].vy, current_peg_x, current_peg_y, i);
            }
            update_ball_position(&balls[j].x, &balls[j].y, &balls[j].vx, &balls[j].vy);
            fillCircle(fix2int15(balls[j].x), fix2int15(balls[j].y), BALL_RADIUS, ball_color);
        }
        // last_collided_peg = -1;

        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        PT_YIELD_usec(spare_time);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD(protothread_display(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;

    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();

        /*
        The current number of balls being animated.
        The total number that have fallen through the board since reset.
        Values of other tunable parameters
        Time since boot
        */

        char buffer[50];

        // Print Current balls
        setCursor(X_DIMENSION * 0.8, 20);
        setTextSize(1);
        setTextColor2(RED, BLACK);

        sprintf(buffer, "current balls: %d", ball_count);
        writeString(buffer);

        // Print Fallen balls
        setCursor(X_DIMENSION * 0.8, 40);
        setTextSize(1);
        setTextColor2(RED, BLACK);

        sprintf(buffer, "fallen balls: %d", fallen_balls);
        writeString(buffer);

        // Print Time since boot
        setCursor(X_DIMENSION * 0.8, 60);
        setTextSize(1);
        setTextColor2(RED, BLACK);
        float elapsed_time_s = (time_us_32() - boot_time) / 1000000.0f;
        sprintf(buffer, "Elapsed time: %.2f", elapsed_time_s);
        writeString(buffer);

        // delay in accordance with frame rate
        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        // yield for necessary amount of time
        PT_YIELD_usec(spare_time);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

static PT_THREAD(protothread_histogram(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time;
    static int spare_time;
    float normalization_coeff = 1.0f;

    int bar_width = X_DIMENSION / 16; // Width of each bar

    while (1)
    {
        // Measure time at start of thread
        begin_time = time_us_32();

        for (int i = 0; i < 16; i++)
        {
            short x_dim = (i)*X_DIMENSION / 16;
            short y_dim = (Y_DIMENSION - histogram[i]) * normalization_coeff;

            // === untested code - uncomment to see if histogram normalizes ===
            // this is not an actual implementation of norm logic, but a hacky fix
            // if (y_dim > Y_DIMENSION * 0.8)
            // {
            //     normalization_coeff = normalization_coeff - 0.1f;
            //     break;
            // }

            fillRect(x_dim, y_dim, bar_width, histogram[i], GREEN);
        }

        // delay in accordance with frame rate
        spare_time = FRAME_RATE - (time_us_32() - begin_time);
        // yield for necessary amount of time
        PT_YIELD_usec(spare_time);
        // NEVER exit while
    } // END WHILE(1)
    PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
    // Add animation thread
    pt_add_thread(protothread_display);
    pt_add_thread(protothread_histogram);

    // Start the scheduler
    pt_schedule_start;
}

// DMA MAIN
// int dma_main()
// {

//     // Initialize SPI channel (channel, baud rate set to 20MHz)
//     spi_init(SPI_PORT, 20000000);

//     // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
//     spi_set_format(SPI_PORT, 16, 0, 0, 0);

//     // Map SPI signals to GPIO ports, acts like framed SPI with this CS mapping
//     gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
//     gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
//     gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
//     gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

//     // Build sine table and DAC data table
//     int i;
//     for (i = 0; i < (sine_table_size); i++)
//     {
//         raw_sin[i] = (int)(2047 * sin((float)i * 6.283 / (float)sine_table_size) + 2047); // 12 bit
//         DAC_data[i] = DAC_config_chan_B | (raw_sin[i] & 0x0fff);
//     }

//     // Select DMA channels
//     int data_chan = dma_claim_unused_channel(true);
//     ;
//     ctrl_chan = dma_claim_unused_channel(true);
//     ;

//     // Setup the control channel
//     dma_channel_config c = dma_channel_get_default_config(ctrl_chan); // default configs
//     channel_config_set_transfer_data_size(&c, DMA_SIZE_32);           // 32-bit txfers
//     channel_config_set_read_increment(&c, false);                     // no read incrementing
//     channel_config_set_write_increment(&c, false);                    // no write incrementing
//     channel_config_set_chain_to(&c, data_chan);                       // chain to data channel

//     dma_channel_configure(
//         ctrl_chan,                        // Channel to be configured
//         &c,                               // The configuration we just created
//         &dma_hw->ch[data_chan].read_addr, // Write address (data channel read address)
//         &address_pointer_A,               // Read address (POINTER TO AN ADDRESS)
//         1,                                // Number of transfers
//         false                             // Don't start immediately
//     );

//     // Setup the data channel
//     dma_channel_config c2 = dma_channel_get_default_config(data_chan); // Default configs
//     channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);           // 16-bit txfers
//     channel_config_set_read_increment(&c2, true);                      // yes read incrementing
//     channel_config_set_write_increment(&c2, false);                    // no write incrementing
//     // (X/Y)*sys_clk, where X is the first 16 bytes and Y is the second
//     // sys_clk is 125 MHz unless changed in code. Configured to ~44 kHz
//     dma_timer_set_fraction(0, 0x0017, 0xffff);
//     // 0x3b means timer0 (see SDK manual)
//     channel_config_set_dreq(&c2, 0x3b); // DREQ paced by timer 0
//     // chain to the controller DMA channel
//     // channel_config_set_chain_to(&c2, ctrl_chan);                        // Chain to control channel

//     dma_channel_configure(
//         data_chan,                 // Channel to be configured
//         &c2,                       // The configuration we just created
//         &spi_get_hw(SPI_PORT)->dr, // write address (SPI data register)
//         DAC_data,                  // The initial read address
//         sine_table_size,           // Number of transfers
//         false                      // Don't start immediately.
//     );

//     // start the control channel
//     // dma_start_channel_mask(1u << ctrl_chan) ;

//     // Exit main.
//     // No code executing!!
// }

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
    // set boot time
    boot_time = time_us_32();

    // initialize stio
    stdio_init_all();

    // start beep
    dma_main();

    // initialize VGA
    initVGA();

    int center_x = 320;

    // Get peg positions and store them in the global array
    get_all_peg_positions(16, center_x);

    //   // start core 1
    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // add threads
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_anim);

    // start scheduler
    pt_schedule_start;
}
