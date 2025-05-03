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

// Include the VGA grahics library
#include "vga16_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_3.h"

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
#define hitBottom(b) (b > int2fix15(380))
#define hitTop(b) (b < int2fix15(100))
#define hitLeft(a) (a < int2fix15(100))
#define hitRight(a) (a > int2fix15(540))

// uS per frame
#define FRAME_RATE 33000
#define FRAME_RATE2 66000

#define X_DIMENSION 640
#define Y_DIMENSION 480

// the color of the boid
char color = WHITE;

// Boid on core 0
fix15 boid0_x;
fix15 boid0_y;
fix15 boid0_vx;
fix15 boid0_vy;

// Boid on core 1
fix15 boid1_x;
fix15 boid1_y;
fix15 boid1_vx;
fix15 boid1_vy;

int cursor_position = 0;

int active_key = 0;

int get_current_pressed_note()
{
  active_key += 1;
  if (active_key > 8)
  {
    active_key = 0;
  }
  return active_key;
}

int get_color(int color, float intensity)
{
  // Ensure intensity is between 0 and 1
  if (intensity < 0)
    intensity = 0;
  if (intensity > 1)
    intensity = 1;

  // Extract RGB components from the color
  int red = (color >> 16) & 0xFF;
  int green = (color >> 8) & 0xFF;
  int blue = color & 0xFF;

  // Scale each component by the intensity
  red = (int)(red * intensity);
  green = (int)(green * intensity);
  blue = (int)(blue * intensity);

  // Combine the components back into a single color
  return (red << 16) | (green << 8) | blue;
}

// Create a boid
void spawnBoid(fix15 *x, fix15 *y, fix15 *vx, fix15 *vy, int direction)
{
  // Start in center of screen
  *x = int2fix15(320);
  *y = int2fix15(240);
  // Choose left or right
  if (direction)
    *vx = int2fix15(3);
  else
    *vx = int2fix15(-3);
  // Moving down
  *vy = int2fix15(1);
}

// Draw the boundaries
void drawArena()
{
  drawVLine(100, 100, 280, WHITE);
  drawVLine(540, 100, 280, WHITE);
  drawHLine(100, 100, 440, WHITE);
  drawHLine(100, 380, 440, WHITE);
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
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write;
  while (1)
  {
    // print prompt
    sprintf(pt_serial_out_buffer, "input a number in the range 1-15: ");
    // non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%d", &user_input);
    // update boid color
    if ((user_input > 0) && (user_input < 16))
    {
      color = (char)user_input;
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

  // Spawn a boid
  spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();
    // erase boid
    drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, BLACK);
    // update boid's position and velocity
    wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy);
    // draw the boid at its new position
    drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, color);
    // draw the boundaries
    drawArena();
    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

static PT_THREAD(protothread_keys(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  int x_offset = 6 * (X_DIMENSION / 10) - 40;
  int y_offset = 2 * (Y_DIMENSION / 5) - 40;

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();

    // need to print a octave of a standard keyboard starting at C4 at x_offset, y_offset
    // print the white keys - note they are 20 pixels wide and 100 pixels tall
    // however, the black keys are 10 pixels wide and 60 pixels tall and are right aligned with the white keys,
    // hence the white has two parts to it, the top and the bottom, the top is 10 pixels wide and 60 pixels tall and the bottom is 20 pixels wide and 40 pixels tall
    int current_key = get_current_pressed_note();

    // draw the C4 key
    if (current_key == 1)
    {
      fillRect(x_offset, y_offset, 8, 60, ORANGE);
      fillRect(x_offset, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset, y_offset, 8, 60, WHITE);
      fillRect(x_offset, y_offset + 60, 18, 40, WHITE);
    }

    // draw the D4 key
    if (current_key == 2)
    {
      fillRect(x_offset + 20, y_offset, 8, 60, ORANGE);
      fillRect(x_offset + 20, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset + 20, y_offset, 8, 60, WHITE);
      fillRect(x_offset + 20, y_offset + 60, 18, 40, WHITE);
    }

    // draw the E4 key
    if (current_key == 3)
    {
      fillRect(x_offset + 40, y_offset, 18, 60, ORANGE);
      fillRect(x_offset + 40, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset + 40, y_offset, 18, 60, WHITE);
      fillRect(x_offset + 40, y_offset + 60, 18, 40, WHITE);
    }

    // draw the F4 key
    if (current_key == 4)
    {
      fillRect(x_offset + 60, y_offset, 8, 60, ORANGE);
      fillRect(x_offset + 60, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset + 60, y_offset, 8, 60, WHITE);
      fillRect(x_offset + 60, y_offset + 60, 18, 40, WHITE);
    }

    // draw the G4 key
    if (current_key == 5)
    {
      fillRect(x_offset + 80, y_offset, 8, 60, ORANGE);
      fillRect(x_offset + 80, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset + 80, y_offset, 8, 60, WHITE);
      fillRect(x_offset + 80, y_offset + 60, 18, 40, WHITE);
    }
    // draw the A4 key
    if (current_key == 6)
    {
      fillRect(x_offset + 100, y_offset, 8, 60, ORANGE);
      fillRect(x_offset + 100, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset + 100, y_offset, 8, 60, WHITE);
      fillRect(x_offset + 100, y_offset + 60, 18, 40, WHITE);
    }
    // draw the B4 key
    if (current_key == 7)
    {
      fillRect(x_offset + 120, y_offset, 18, 60, ORANGE);
      fillRect(x_offset + 120, y_offset + 60, 18, 40, ORANGE);
    }
    else
    {
      fillRect(x_offset + 120, y_offset, 18, 60, WHITE);
      fillRect(x_offset + 120, y_offset + 60, 18, 40, WHITE);
    }

    // print the black keys
    // draw the C#4 key
    fillRect(x_offset + 10, y_offset, 10, 60, BLACK);
    // draw the D#4 key
    fillRect(x_offset + 30, y_offset, 10, 60, BLACK);
    // draw the F#4 key
    fillRect(x_offset + 70, y_offset, 10, 60, BLACK);
    // draw the G#4 key
    fillRect(x_offset + 90, y_offset, 10, 60, BLACK);
    // draw the A#4 key
    fillRect(x_offset + 110, y_offset, 10, 60, BLACK);

    spare_time = FRAME_RATE2 - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

static PT_THREAD(protothread_vga_drums(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  // x,y offsets for the drums positions
  int x_offset = 2 * (X_DIMENSION / 10) - 40;
  int y_offset = 2 * (Y_DIMENSION / 5) - 40;
  int drum_size = 50;

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();

    // print 2X2 drum set
    setCursor(x_offset, y_offset);

    // erase the drum set 1
    // fillRect(x_offset, y_offset, drum_size, drum_size, BLACK);
    // draw the drum set 1
    fillRect(x_offset, y_offset, drum_size, drum_size, RED);

    // animation to show drum set 1 is pressed - the color becomes darker and then back to normal over the frames
    float intensity = 0.5; // initial intensity

    fillRect(x_offset, y_offset, drum_size, drum_size, RED);
    intensity += 0.1; // increase intensity

    // erase the drum set 2
    // fillRect(x_offset + drum_size + 2, y_offset, drum_size, drum_size, BLACK);
    // draw the drum set 2
    fillRect(x_offset + drum_size + 2, y_offset, drum_size, drum_size, BLUE);

    // draw the drum set 3
    fillRect(x_offset, y_offset + drum_size + 2, drum_size, drum_size, GREEN);
    // draw the drum set 4
    fillRect(x_offset + drum_size + 2, y_offset + drum_size + 2, drum_size, drum_size, YELLOW);

    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 0
static PT_THREAD(protothread_vga_title(struct pt *pt))
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

    setCursor(X_DIMENSION / 2, 0);
    setTextSize(2);
    setTextColor2(WHITE, BLACK);
    // print title
    writeString("VGA Synth Pad v1.0");
    // delay in accordance with frame rate

    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

static PT_THREAD(protothread_vga_state(struct pt *pt))
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
    int x_offset = X_DIMENSION / 10;
    int y_offset = 4 * (Y_DIMENSION / 5);

    // print key signature
    setCursor(x_offset, y_offset);
    setTextSize(1);
    setTextColor2(GREEN, BLACK);
    writeString("Current Key Signature: ");

    // print mode
    setCursor(x_offset, y_offset + 20);
    setTextSize(1);
    setTextColor2(GREEN, BLACK);
    writeString("Mode: ");

    // print backing track name
    setCursor(x_offset, y_offset + 40);
    setTextSize(1);
    setTextColor2(GREEN, BLACK);
    writeString("Backing Track: ");

    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

static PT_THREAD(protothread_track_state(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;
  int x_offset = 8 * (X_DIMENSION / 10) - 40;
  int y_offset = 4 * (Y_DIMENSION / 5);

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();

    // print seek line with position of the cursor
    setCursor(x_offset, y_offset);
    drawHLine(x_offset, y_offset, 160, WHITE);

    setCursor(x_offset + cursor_position, y_offset);
    drawRect(x_offset + cursor_position, y_offset, 2, 2, BLACK);

    cursor_position += 1;
    setCursor(x_offset + cursor_position, y_offset);
    drawRect(x_offset + cursor_position, y_offset, 2, 2, GREEN);

    if (cursor_position >= 160)
    {
      cursor_position = 0;
    }

    // print play, pause, next, previous
    setCursor(x_offset, y_offset + 20);
    setTextSize(1);
    setTextColor2(GREEN, BLACK);
    writeString("<<       Play/Pause     >>");

    // print backing track name
    setCursor(x_offset, y_offset + 40);
    setTextSize(1);
    setTextColor2(GREEN, BLACK);
    writeString("Backing Track: ");

    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Animation on core 1
static PT_THREAD(protothread_anim1(struct pt *pt))
{
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time;
  static int spare_time;

  // Spawn a boid
  spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

  while (1)
  {
    // Measure time at start of thread
    begin_time = time_us_32();
    // erase boid
    drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, BLACK);
    // update boid's position and velocity
    wallsAndEdges(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy);
    // draw the boid at its new position
    drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, color);
    // delay in accordance with frame rate
    spare_time = FRAME_RATE - (time_us_32() - begin_time);
    // yield for necessary amount of time
    PT_YIELD_usec(spare_time);
    // NEVER exit while
  } // END WHILE(1)
  PT_END(pt);
} // animation thread

// Function to get a darker shade of a color

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main()
{
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start;
}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main()
{
  // initialize stio
  stdio_init_all();

  // initialize VGA
  initVGA();

  // start core 1
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_vga_title);
  pt_add_thread(protothread_vga_state);
  pt_add_thread(protothread_track_state);
  pt_add_thread(protothread_vga_drums);
  pt_add_thread(protothread_keys);

  // start scheduler
  pt_schedule_start;
}
