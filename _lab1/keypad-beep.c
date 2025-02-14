 /**
 * Based on Demos from Hunter Adams (vha3@cornell.edu)
 * Edited by Nicholas Papapanou (ngp37) & Sivaraman, Sankar (ss4362)
 *
 * Keypad w/ Beep
 *
 * KEYPAD CONNECTIONS
 *  - GPIO 9   -->  330 ohms  --> Pin 1 (button row 1)
 *  - GPIO 10  -->  330 ohms  --> Pin 2 (button row 2)
 *  - GPIO 11  -->  330 ohms  --> Pin 3 (button row 3)
 *  - GPIO 12  -->  330 ohms  --> Pin 4 (button row 4)
 *  - GPIO 13  -->     Pin 5 (button col 1)
 *  - GPIO 14  -->     Pin 6 (button col 2)
 *  - GPIO 15  -->     Pin 7 (button col 3)
 *
 * VGA CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #include <string.h>

 #include "pico/stdlib.h"
 #include "pico/multicore.h"

 #include "hardware/pio.h"
 #include "hardware/dma.h"
 #include "hardware/sync.h"
 #include "hardware/spi.h"

 // VGA graphics library
 #include "vga16_graphics.h"
 #include "pt_cornell_rp2040_v1_3.h"

 // ADDED DEFINITIONS FROM BEEP //

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
 // Phase accumulator and phase increment. Increment sets output frequency. Inner sin function for swoop.
 volatile unsigned int phase_accum_swoop;
 volatile unsigned int phase_incr_swoop;
 volatile unsigned int phase_accum_chirp;
 volatile unsigned int phase_incr_chirp;
 volatile float Fout_swoop;
 volatile float Fout_chirp;

 static float m = -0.00048332194;

 // DDS sine table (populated in main())
 #define sine_table_size 256
 fix15 sin_table[sine_table_size] ;

 // Values output to DAC
 int DAC_output_swoop ;
 int DAC_output_chirp ;

 // Amplitude modulation parameters and variables
 fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
 fix15 attack_inc ;                      // rate at which sound ramps up
 fix15 decay_inc ;                       // rate at which sound ramps down
 fix15 current_amplitude_swoop = 0 ;         // current amplitude (modified in ISR)
 fix15 current_amplitude_chirp = 0 ;         // current amplitude (modified in ISR)

 // Timing parameters for beeps (units of interrupts)
 #define ATTACK_TIME             250
 #define DECAY_TIME              250
 #define SUSTAIN_TIME            10000
 #define SWOOP_DURATION          6500
 #define BEEP_REPEAT_INTERVAL    50000

 // Debouncing FSM states
 #define NOT_PRESSED         0
 #define MAYBE_PRESSED       1
 #define PRESSED             2
 #define MAYBE_NOT_PRESSED   3

// swoop statees
# define SWOOP 1
# define CHIRP 2
# define IDLE -1

 // State machine variables
 volatile unsigned int STATE = IDLE ;
 volatile unsigned int count_swoop = 0 ;
 volatile unsigned int count_chirp = 0 ;
 volatile unsigned int BOUNCE_STATE = NOT_PRESSED ;

 volatile int take_action = 0 ;
 volatile int key_pressed = 0 ;

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

 //GPIO for timing the ISR
 #define ISR_GPIO 2

 // END OF ADDED BEEP DEFINITIONS //

 // Keypad pin configurations
 #define BASE_KEYPAD_PIN 9
 #define KEYROWS         4
 #define NUMKEYS         12

 unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                 0x22, 0x42, 0x14, 0x24, 0x44,
                                 0x18, 0x48} ;
 unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
 unsigned int button = 0x70 ;


 char keytext[40];
 int prev_key = 0;


 // This timer ISR is called on core 0
 static void alarm_irq(void) {

     // Assert a GPIO when we enter the interrupt
     gpio_put(ISR_GPIO, 1) ;

     // Clear the alarm irq
     hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

     // Reset the alarm register
     timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

     if (STATE == SWOOP) { // Generate swoop
         // DDS phase and sine table lookup
         Fout_swoop = -260 * sin(m*count_swoop) + 1740;
         phase_incr_swoop = (Fout_swoop*two32)/Fs ;
         phase_accum_swoop += phase_incr_swoop  ;
         DAC_output_swoop = fix2int15(multfix15(current_amplitude_swoop,
             sin_table[phase_accum_swoop>>24])) + 2048 ;

         // Ramp up amplitude
         if (count_swoop < ATTACK_TIME) {
             current_amplitude_swoop = (current_amplitude_swoop + attack_inc) ;
         }
         // Ramp down amplitude
         else if (count_swoop > SWOOP_DURATION - DECAY_TIME) {
             current_amplitude_swoop = (current_amplitude_swoop - decay_inc) ;
         }

         // Mask with DAC control bits
         DAC_data_0 = (DAC_config_chan_B | (DAC_output_swoop & 0xffff))  ;

         // SPI write (no spinlock b/c of SPI buffer)
         spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

         // Increment the counter
         count_swoop += 1 ;

         // State transition?
         if (count_swoop == SWOOP_DURATION) {
             STATE = IDLE ;
             count_swoop = 0 ;
         }
     }

     else if (STATE == CHIRP) { // Generate chirp
        // DDS phase and sine table lookup
        Fout_chirp = 0.00018 * count_chirp * count_chirp + 2000;
        phase_incr_chirp = (Fout_chirp*two32)/Fs ;
        phase_accum_chirp += phase_incr_chirp  ;
        DAC_output_chirp = fix2int15(multfix15(current_amplitude_chirp,
            sin_table[phase_accum_chirp>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_chirp < ATTACK_TIME) {
            current_amplitude_chirp = (current_amplitude_chirp + attack_inc) ;
        }
        // Ramp down amplitude
        else if ( count_chirp > SWOOP_DURATION - DECAY_TIME) {
            current_amplitude_chirp = (current_amplitude_chirp - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_chirp & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_chirp += 1 ;

        // State transition?
        if (count_chirp == SWOOP_DURATION) {
            STATE = IDLE ;
            count_chirp = 0 ;
        }
    }



     // De-assert the GPIO when we leave the interrupt
     gpio_put(ISR_GPIO, 0) ;

 }

 // This thread runs on core 0
 static PT_THREAD (protothread_core_0(struct pt *pt))
 {
     // Indicate thread beginning
     PT_BEGIN(pt) ;

     // Some variables
     static int i ;
     static int possible ;

     static uint32_t keypad ;

     while(1) {

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

         // Debounce FSM implementation
         // i == -1 not a valid press
         // i != -1 valid press
         switch (BOUNCE_STATE) {
            case NOT_PRESSED:
                if (i != -1) {
                    possible = i;
                    BOUNCE_STATE = MAYBE_PRESSED;
                }
                break;

            case MAYBE_PRESSED:
                if (i == possible) {
                    // Only beep when 1 is pressed
                    if (i == 1) {
                        current_amplitude_swoop = 0;
                        STATE = SWOOP;
                        count_swoop = 0;
                    }
                    if (i == 2) {
                        current_amplitude_chirp = 0;
                        STATE = CHIRP;
                        count_chirp = 0;
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

            default:
                // Optionally handle unexpected states
                break;
        }

         // Write key to VGA
         if (i != prev_key) {
             prev_key = i ;
             fillRect(250, 20, 176, 30, RED); // red box
             sprintf(keytext, "%d", i) ;
             setCursor(250, 20) ;
             setTextSize(2) ;
             writeString(keytext) ;
         }

         // Print key to terminal
         printf("\n%d", i) ;

         PT_YIELD_usec(30000) ;
     }
     // Indicate thread end
     PT_END(pt) ;
 }

 void beepBeepMain(){

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

 }

 int main() {

     // Initialize stdio
     stdio_init_all();

     beepBeepMain();

     // Initialize the VGA screen
     initVGA() ;

     // Draw some filled rectangles
     fillRect(64, 0, 176, 50, BLUE); // blue box
     fillRect(250, 0, 176, 50, RED); // red box
     fillRect(435, 0, 176, 50, GREEN); // green box

     // Write some text
     setTextColor(WHITE) ;
     setCursor(65, 0) ;
     setTextSize(1) ;
     writeString("Raspberry Pi Pico") ;
     setCursor(65, 10) ;
     writeString("Keypad demo") ;
     setCursor(65, 20) ;
     writeString("Hunter Adams") ;
     setCursor(65, 30) ;
     writeString("vha3@cornell.edu") ;
     setCursor(250, 0) ;
     setTextSize(2) ;
     writeString("Key Pressed:") ;

     // Map LED to GPIO port, make it low
     gpio_init(LED) ;
     gpio_set_dir(LED, GPIO_OUT) ;
     gpio_put(LED, 0) ;

     ////////////////// KEYPAD INITS ///////////////////////
     // Initialize the keypad GPIO's
     gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
     // Set row-pins to output
     gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
     // Set all output pins to low
     gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
     // Turn on pulldown resistors for column pins (on by default)
     gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
     gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
     gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

     // Add core 0 threads
     pt_add_thread(protothread_core_0) ;

     // Start scheduling core 0 threads
     pt_schedule_start ;

 }
