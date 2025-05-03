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

// Shared header file for multicore communication
#include "shared.h"

// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"


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

// Screen dimensions
#define X_DIMENSION 640
#define Y_DIMENSION 480

int cursor_position = 0;
int active_note = 0;

char current_pressed_note = 'C'; // Default to C4
int current_pressed_drums = 0;   // Default to 0, ranges from [0,3]

void send_note_to_vga(MessageType type, char payload)
{
  StateMessage msg;
  msg.type = type;
  msg.payload = payload;
  multicore_fifo_push_blocking(*(uint32_t *)&msg); // Bitcast struct as 32-bit
}

void set_current_key_signature(char key_signature)
{
  // Set the current key signature
  switch (key_signature)
  {
  case 'C':
    active_note = 1; // C Major
    break;
  case 'A':
    active_note = 2; // A Major
    break;
  case 'G':
    active_note = 3; // G Major
    break;
  default:
    active_note = 1; // Default to C Major
    break;
  }
}

void set_current_pressed_note(int note)
{
  int current_key_signature = active_note;

  // pentatonic scale corresponding to the key signature
  // C Major: C4, D4, E4, G4, A4
  // A Major: A4, B4, C#5, E5, F#5
  // G Major: G4, A4, B4, D5, E5

  switch (current_key_signature)
  {
  case 1:
    // C Major
    switch (note)
    {
    case 1:
      current_pressed_note = 'C';
      break;
    case 2:
      current_pressed_note = 'D';
      break;
    case 3:
      current_pressed_note = 'E';
      break;
    case 4:
      current_pressed_note = 'G';
      break;
    case 5:
      current_pressed_note = 'A';
      break;
    }
    break;

  case 2:
    // A Major
    switch (note)
    {
    case 1:
      current_pressed_note = 'A';
      break;
    case 2:
      current_pressed_note = 'B';
      break;
    case 3:
      current_pressed_note = 'c';
      break;
    case 5:
      current_pressed_note = 'E';
      break;
    case 6:
      current_pressed_note = 'f';
      break;
    }
    break;

  case 3:
    // G Major
    switch (note)
    {
    case 1:
      current_pressed_note = 'G';
      break;
    case 2:
      current_pressed_note = 'A';
      break;
    case 3:
      current_pressed_note = 'B';
      break;
    case 5:
      current_pressed_note = 'D';
      break;
    case 6:
      current_pressed_note = 'E';
      break;
    }
    break;
  default:
    // Default to C Major if no valid key signature is selected
    current_pressed_note = 'C';
    break;
  }
}

char get_current_pressed_note()
{
  return current_pressed_note;
}

// Non-blocking, one-step version of poll_queue
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
    case MSG_KEY_CHANGE:
      printf("\n[Core 1] Key signature changed to %c\n", msg.payload);
      set_current_key_signature(msg.payload);
      break;
    case MSG_BACKING_TRACK_CHANGE:
      // Handle other types if needed
      break;
    case MSG_DRUMS_CHANGE:
      printf("\n[Core 1] Playing drum %d\n", msg.payload);
      current_pressed_drums = msg.payload - '0'; // Convert char to int
      break;
    default:
      printf("[Core 1] Unknown message type\n");
    }
  }
}

// Protothread to run poll_queue continuously
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

// ==================================================
// users serial input thread
// ==================================================
// Animation on core 1
// * Renders the keyboard on the screen
// * Reacts to key presses via multicore FIFO method
// static PT_THREAD(thread_pitch_bend(struct pt *pt))

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
    char current_key = get_current_pressed_note();

    // draw the C4 key
    fillRect(x_offset, y_offset, 8, 60, current_key == 'C' ? ORANGE : WHITE);
    fillRect(x_offset, y_offset + 60, 18, 40, current_key == 'C' ? ORANGE : WHITE);

    // draw the D4 key
    fillRect(x_offset + 20, y_offset, 8, 60, current_key == 'D' ? ORANGE : WHITE);
    fillRect(x_offset + 20, y_offset + 60, 18, 40, current_key == 'D' ? ORANGE : WHITE);

    // draw the E4 key
    fillRect(x_offset + 40, y_offset, 18, 60, current_key == 'E' ? ORANGE : WHITE);
    fillRect(x_offset + 40, y_offset + 60, 18, 40, current_key == 'E' ? ORANGE : WHITE);

    // draw the F4 key
    fillRect(x_offset + 60, y_offset, 8, 60, current_key == 'F' ? ORANGE : WHITE);
    fillRect(x_offset + 60, y_offset + 60, 18, 40, current_key == 'F' ? ORANGE : WHITE);

    // draw the G4 key
    fillRect(x_offset + 80, y_offset, 18, 60, current_key == 'G' ? ORANGE : WHITE);
    fillRect(x_offset + 80, y_offset + 60, 18, 40, current_key == 'G' ? ORANGE : WHITE);

    // draw the A4 key
    fillRect(x_offset + 100, y_offset, 8, 60, current_key == 'A' ? ORANGE : WHITE);
    fillRect(x_offset + 100, y_offset + 60, 18, 40, current_key == 'A' ? ORANGE : WHITE);

    // draw the B4 key
    fillRect(x_offset + 120, y_offset, 18, 60, current_key == 'B' ? ORANGE : WHITE);
    fillRect(x_offset + 120, y_offset + 60, 18, 40, current_key == 'B' ? ORANGE : WHITE);

    // print the black keys
    // draw the C#4 key
    fillRect(x_offset + 10, y_offset, 10, 60, current_key == 'c' ? ORANGE : BLACK);
    // draw the D#4 key
    fillRect(x_offset + 30, y_offset, 10, 60, current_key == 'd' ? ORANGE : BLACK);
    // draw the F#4 key
    fillRect(x_offset + 70, y_offset, 10, 60, current_key == 'f' ? ORANGE : BLACK);
    // draw the G#4 key
    fillRect(x_offset + 90, y_offset, 10, 60, current_key == 'g' ? ORANGE : BLACK);
    // draw the A#4 key
    fillRect(x_offset + 110, y_offset, 10, 60, current_key == 'a' ? ORANGE : BLACK);

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
    fillRect(x_offset, y_offset, drum_size, drum_size, current_pressed_drums == 0 ? WHITE : RED);

    // erase the drum set 2
    // fillRect(x_offset + drum_size + 2, y_offset, drum_size, drum_size, BLACK);
    // draw the drum set 2
    fillRect(x_offset + drum_size + 2, y_offset, drum_size, drum_size, current_pressed_drums == 1 ? WHITE : BLUE);

    // draw the drum set 3
    fillRect(x_offset, y_offset + drum_size + 2, drum_size, drum_size, current_pressed_drums == 2 ? BLACK : GREEN);
    // draw the drum set 4
    fillRect(x_offset + drum_size + 2, y_offset + drum_size + 2, drum_size, drum_size, current_pressed_drums == 3 ? BLACK : YELLOW);

    // reset the drum state to -1
    current_pressed_drums = -1;
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
  char screentext[40];

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
    sprintf(screentext, "Current Key signature: %d", active_note);
    writeString(screentext);

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


// ==================================================
// ==== core1 entry point =========================
// ==================================================

void core1_main()
{
  // add threads
  pt_add_thread(thread_poll_queue); // poll queue thread
  pt_add_thread(protothread_vga_title);
  pt_add_thread(protothread_vga_state);
  pt_add_thread(protothread_track_state);
  pt_add_thread(protothread_vga_drums);
  pt_add_thread(protothread_keys);
  // Start the scheduler
  pt_schedule_start;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD(protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
  // stores user input
  static int user_input;
  static char key_signature;
  static int note;
  // wait for 0.1 sec
  PT_YIELD_usec(1000000);
  // announce the threader version
  sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
  // non-blocking write
  serial_write;
  while (1)
  {
    // print prompt
    sprintf(pt_serial_out_buffer, "input a number in the range 1: Key change, 2: Note change \n");
    // non-blocking write
    serial_write;
    // spawn a thread to do the non-blocking serial read
    serial_read;
    // convert input string to number
    sscanf(pt_serial_in_buffer, "%d", &user_input);

    switch (user_input)
    {
    case 1:
      // get input for the key signature
      sprintf(pt_serial_out_buffer, "input a key signature: C, A, G \n");
      // non-blocking write
      serial_write;
      serial_read;
      sscanf(pt_serial_in_buffer, "%c", &key_signature);
      // send the key signature to core

      send_note_to_vga(MSG_KEY_CHANGE, key_signature);
      break;
    case 2:
      // get input for the note
      sprintf(pt_serial_out_buffer, "input a note: 1: C, 2: D, 3: E, 4: F, 5: G, 6: A, 7: B \n");
      // non-blocking write
      serial_write;
      serial_read;
      sscanf(pt_serial_in_buffer, "%d", &note);
      // send the note to core
      send_note_to_vga(MSG_NOTE_CHANGE, note + '0'); // convert to char
      break;
    case 3:
      // get input for the drums
      sprintf(pt_serial_out_buffer, "input a drum: 0: kick, 1: snare, 2: hi-hat, 3: cymbal \n");
      // non-blocking write
      serial_write;
      serial_read;
      sscanf(pt_serial_in_buffer, "%d", &current_pressed_drums);
      // send the note to core
      send_note_to_vga(MSG_DRUMS_CHANGE, current_pressed_drums + '0'); // convert to char
    default:
      break;
    }

    // print the input number to the monitor
    sprintf(pt_serial_out_buffer, "You entered: %d\n\r", user_input);
    // non-blocking write
    serial_write;

  } // END WHILE(1)
  PT_END(pt);
} // timer thread


// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
// int main()
// {
//   // initialize stio
//   stdio_init_all();

//   // initialize VGA
//   initVGA();

//   // start core 1
//   multicore_reset_core1();
//   multicore_launch_core1(&core1_main);

//   // add serial thread
//   pt_add_thread(protothread_serial);

//   // start scheduler
//   pt_schedule_start;
// }
