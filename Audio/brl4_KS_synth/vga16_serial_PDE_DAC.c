/**
 * Bruce Land and Hunter Adams (vha3@cornell.edu)
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green lo-bit to VGA green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green hi_bit to VGA green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1, 2, and 3
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 * Protothreads v1.1.1
 * Serial console on GPIO 0 and 1 for debugging
 * and for synth conrol
 * 
 * DAC :
 * GPIO 5 (pin 7) Chip select
   GPIO 6 (pin 9) SCK/spi0_sclk
   GPIO 7 (pin 10) MOSI/spi0_tx
   3.3v (pin 36) -> VCC on DAC 
   GND (pin 3)  -> GND on DAC
 * 
 * =====================================================
 * 
 * There are two threads running on each core, plus the synthesis ISR running on core1
 * The PDE synthesis woks nicely with s15x16 fixed point, but fails with s1x14, probably
 * due to excessive runcation eror.
 * 
 * ==========
 * Support routines:
 * -- ADC setup
 * -- Define two different fixed point schemes
 *      One for DDS which requires bigger numbers, the other for FFT which needs fast execution.
 * -- Set up a low-level timer alarm which truggers the synthesis ISR
 * -- Define the FFT routine in 16-bit fixed point for speed
 * -- Define routine to extract a fast FFT magnutude specterum
 *      Accuracy is about 2%
 * -- Define routine to extract a VERY low resoultion approximaton to log(mag)
 *      The bit twiddling produces a u4x4 fixed poiint log. Good enough for graphics only.
 * 
 * ==========
 * Core0:
 * 
 * -- Graphics thread which generates some text, then currently does nothing
 * 
 * 
 * ==========
 * Core1:
 * 
 * -- synthesis ISR triggered by a timer alarm
 * ---- runs at 40KHz (25 uSec interval) for audio synthesis.
 * ---- computes fulll string PDE
 * ---- Not reaaly practical, since max length of srin is 20 at 40 KHz.
 * ---- and take 0.6 of a cpu.
 * ---- Pushes output to an SPI channel
 * ---- Fills an array with DDS results for the FFT thread
 * 
 * -- FFT thread which waits for a buffer full signal from the ISR, then draws spectra
 * ---- Does a 2048 point FFT then gets the amplitude of the frequency bins and takes a VERY approx log2
 * ---- plots the amlitude spectrum, log amplitude spectrum, and spectrogram
 * -- A play thread which
 * ---- just repeats a note.
 * 
 * PIO: There are three PIO state machines that generate the video.
 * See: https://vanhunteradams.com/Pico/VGA/VGA.html
 */
// ==========================================
// === VGA graphics library
// ==========================================
#include "vga16_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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

float serial_value ;
char serial_value_input[32] ;
int play_scale = true ;


// ==========================================
// === fixed point s15x16
// ==========================================
// s15x16 fixed point macros ==
// == resolution 2^-16 = 1.5e-5
// == dynamic range is 32767/-32768
typedef signed int s15x16;
#define muls15x16(a,b) ((s15x16)(((( signed long long )(a))*(( signed long long )(b)))>>16)) //multiply two fixed 16:16
#define float_to_s15x16(a) ((s15x16)((a)*65536.0)) // 2^16
#define s15x16_to_float(a) ((float)(a)/65536.0)
#define s15x16_to_int(a)    ((int)((a)>>16))
#define int_to_s15x16(a)    ((s15x16)((a)<<16))
#define divs15x16(a,b) ((s15x16)((((signed long long)(a)<<16)/(b)))) 
#define abss15x16(a) abs(a)

// ==========================================
// === fixed point s1x14 for FFT
// ==========================================
// s1.14 format -- short format is faster
// == resolution 2^-14 = 6.1035e-5
// == dynamic range is +1.9999/-2.0

typedef signed short s1x14 ;
#define muls1x14(a,b) ((s1x14)((((int)(a))*((int)(b)))>>14)) 
#define float_to_s1x14(a) ((s1x14)((a)*16384.0)) // 2^14
#define s1x14_to_float(a) ((float)(a)/16384.0)
#define abss1x14(a) abs(a) 
#define divs1x14(a,b) ((s1x14)((((signed int)(a)<<14)/(b)))) 

// make the integer part of s5x16 (max +/- 2048) map to +/-1 in s1x14  
#define s15x16_to_s1x14(a)  ((s1x14)((((s15x16)(a)>>14)))) 

// string-specific multiply macros
// for _Accum fixed point
#define times0pt5(a) ((a)>>1) 
#define times0pt25(a) ((a)>>2) 
#define times2pt0(a) ((a)<<1) 
#define times4pt0(a) ((a)<<2) 
#define times0pt9998(a) ((a)-((a)>>12)) //>>10
#define times0pt999(a) ((a)-((a)>>10)) //>>10
#define times0pt9999(a) ((a)-((a)>>13))
#define times0pt99999(a) ((a)-((a)>>16))

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

//SPI configurations
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define SPI_PORT spi0

// data for the spi port
uint16_t DAC_data ; 

// ==========================================
// === set up DDS and timer ISR
// ==========================================
// 1/Fs in microseconds
// 40 KSamples/sec
volatile int alarm_period = 25 ;

// inputs
float Fs, Fout, Fmod ;
//              C3     D3     E3     F3     G3     A3     B3     C4
float notes[8]={130.8, 146.8, 164.8, 174.6, 196.0, 220.0, 246.9, 261.6};
//
volatile int note_start = true ;
 
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
// ISR interval will be 10 uSec
//
// the actual ISR
void compute_sample(void);
//
static void alarm_irq(void) {
    // mark ISR entry
    gpio_put(2,1);
    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
    // arm the next interrupt
    // Write the lower 32 bits of the target time to the alarm to arm it
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + alarm_period ;
    //
    // the routine which actually does the PDE
    compute_sample();

    // mark ISR exit
    gpio_put(2,0);
}

// set up the timer alarm ISR
static void alarm_in_us(uint32_t delay_us) {
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
    timer_hw->alarm[ALARM_NUM] = (uint32_t) target;   
}

// ===========================================
// dSP definitions 
// ===========================================
//
// FFT setup
#define N_WAVE          2048 //1024    
#define LOG2_N_WAVE     11 //10     /* log2(N_WAVE) 0 */

s1x14 Sinewave[N_WAVE]; // a table of sines for the FFT
s1x14 window[N_WAVE]; // a table of window values for the FFT
s1x14 fr[N_WAVE], fi[N_WAVE]; // input data
// dds output to FFT
s1x14 dds[N_WAVE] ; // the dds output from ISR
// index into dds buffer
int buffer_index ;
// displa points for graphiics
short display_data[N_WAVE] ;
short fft_display_data[N_WAVE] ;
short log_display_data[N_WAVE] ;


// ==================================
// === Init FFT arrays
//====================================
void FFTinit(void){
// one cycle sine table
  //  required for FFT
  for (int ii = 0; ii < N_WAVE; ii++) {
    // one cycle per window for FFT -- scall amp for number of bits
    Sinewave[ii] = float_to_s1x14(0.5 * sin(6.283 * ((float) ii) / N_WAVE));
    // Raised cos window
    // I think the spectrgraph looks better with flat window
    window[ii] = float_to_s1x14(1.0 ) ;//- cos(6.283 * ((float) ii) / (N_WAVE - 1)));
  }
}

// ==================================
// === FFT
//====================================
void FFTfix(s1x14 fr[], s1x14 fi[], int m){
//Adapted from code by:
//Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
//fr[n],fi[n] are real,imaginary arrays, INPUT AND RESULT.
//size of data = 2**m
// This routine does foward transform only

  int mr,nn,i,j,L,k,istep, n;
  s1x14 qr,qi,tr,ti,wr,wi;

  mr = 0;
  n = 1<<m;   //number of points
  nn = n - 1;

  /* decimation in time - re-order data */
  for(m=1; m<=nn; ++m){
      L = n;
      do L >>= 1; while(mr+L > nn);
      mr = (mr & (L-1)) + L;
      if(mr <= m) continue;
      tr = fr[m];
      fr[m] = fr[mr];
      fr[mr] = tr;
      ti = fi[m];   //for real inputs, don't need this
      fi[m] = fi[mr]; //for real inputs, don't need this
      fi[mr] = ti; //for real inputs, don't need this
  }

  L = 1;
  k = LOG2_N_WAVE-1;
  while(L < n) {
      istep = L << 1;
      for(m=0; m<L; ++m){
          j = m << k;
          wr =  Sinewave[j+N_WAVE/4];
          wi = -Sinewave[j];
          //wr >>= 1; //do need if scale table
          //wi >>= 1;

          for(i=m; i<n; i+=istep){
              j = i + L;
              tr = muls1x14(wr, fr[j]) - muls1x14(wi, fi[j]);
              ti = muls1x14(wr, fi[j]) + muls1x14(wi, fr[j]);
              qr = fr[i] >> 1;
              qi = fi[i] >> 1;
              fr[j] = qr - tr;
              fi[j] = qi - ti;
              fr[i] = qr + tr;
              fi[i] = qi + ti;
          }
      }
      --k;
      L = istep;
  }
}

//====================================
// === magnitude approx good to about +/-2%
// see https://en.wikipedia.org/wiki/Alpha_max_plus_beta_min_algorithm
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
//====================================
void magnitude(s1x14 fr[], s1x14 fi[], int length){
  s1x14 mmax, mmin ;
  s1x14 c1 = float_to_s1x14(0.89820) ;
  s1x14 c2 = float_to_s1x14(0.48597) ;
  for (int ii = 0; ii < length; ii++) {
        mmin = min(abs(fr[ii]), abs(fi[ii])); //>>9
        mmax = max(abs(fr[ii]), abs(fi[ii]));
        // reuse fr to hold magnitude
        fr[ii] = max(mmax, (muls1x14(mmax,c1) + muls1x14(mmin,c2) )); 
        fi[ii] = 0;
      }
}

// ==================================
// === approx log2 for plotting
//====================================
#define log_min 0x00  
// reuse fr to hold log magnitude
// shifting finds most significant bit
// then make approxlog  = ly + (fr-y)./(y) + 0.043;
// BUT for an 8-bit approx (4 bit ly and 4-bit fraction)
// ly 1<=ly<=14
// omit the 0.043 because it is too small for 4-bit fraction
void log2_approx(s1x14 fr[], int length) {
  //
  int sx, y, ly, temp ;
  for (int i = 0; i < length; i++) {
    // interpret bits as integer
    sx = fr[i];
    y=1; ly=0;
    while(sx>1) {
        y=y*2; ly=ly+1; sx=sx>>1;
    }
    // shift ly into upper 4-bits as integer part of log
    // take bits below y and shift into lower 4-bits
    // !!NOTE that fr is no longer in s1x14 format!!
    fr[i] = ((ly)<<4) + ((fr[i]-y)>>(ly-4) ) ;
    // bound the noise at low amp
    if(fr[i]<log_min) fr[i] = log_min;
  }
}

// ==================================================
// === graphics  -- RUNNING on core 0
// ==================================================
static PT_THREAD (protothread_graphics(struct pt *pt)) {
    PT_BEGIN(pt);
    // update count -- for tuning cursor speed
    static int update_count ;
    // the protothreads interval timer
    PT_INTERVAL_INIT() ;

    // draw a big, clunky banner
    #define banner_bottom 50
    // Draw some filled rectangles at top of screen
    fillRect(64, 0, 176, banner_bottom, BLUE); // blue box
    fillRect(250, 0, 176, banner_bottom, YELLOW); // red box
    fillRect(435, 0, 176, banner_bottom, GREEN); // green box

    // Write some intro banner text
    PT_YIELD_usec(10000) ;
    setTextColor(WHITE) ;
    setCursor(65, 0) ;
    setTextSize(1) ;
    writeString("  ") ;
    setCursor(65, 0) ;
    writeString("Pi Pico") ;
    setCursor(65, 10) ;
    writeString("PDE string synthesis demo") ;
    setCursor(65, 20) ;
    writeString("Hunter Adams & Bruce Land") ;
    setCursor(65, 30) ;
    writeString("vha3@cornell.edu") ;
    setCursor(65, 40) ;
    writeString("brl4@cornell.edu") ;
    
    setTextColor(BLACK) ;
    setCursor(300, 6) ;
    writeStringBig("ECE 4760") ;
    setCursor(300, 22) ;
    writeStringBig("PDE string synth") ;

    setCursor(445, 10) ;
    setTextSize(1) ;
    writeString("Protothreads rp2040 v1.3") ;
    setCursor(445, 20) ;
    writeString("VGA - 640x480 - 16 colors ") ;
    setTextColor(WHITE) ;

    while(true) {
        // count passes thru loop
        update_count++ ;

        
        // A brief nap about 16/ec
        PT_YIELD_usec(1000000) ;
   }
   PT_END(pt);
} // graphics thread

// ==================================================
// === play the scale thread on core 1
// ==================================================
static PT_THREAD (protothread_play(struct pt *pt))
{
    PT_BEGIN(pt);
    
     // data structure for interval timer
     PT_INTERVAL_INIT() ;

      while(1) {
        static int i ;
        PT_YIELD_usec(1000) ;
        if(play_scale){

            //mod_attack_inc = div(current_depth, mod_attack_time) ;
            //mod_decay_inc = div(current_depth, mod_decay_time) ;
            for(i=0; i<8; i++){
               // current_main_inc = main_inc[i] ;
                //rho = 
                note_start = true ;
                //PT_YIELD_usec(20000) ;
                // play until too small for DAC
               // PT_YIELD_UNTIL(pt, current_amp<onefix);
                PT_YIELD_usec(2000000) ;
            }
        }
        //
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // play thread

// ==================================================
// === ISR routine -- RUNNING on core 1
// ==================================================
// 
int note_index ;
#define string_size 20
#define copy_len 4*string_size
s15x16 string_n[string_size], string_n_1[string_size], new_string[string_size], new_string_temp;
s15x16 init_string[string_size];
int pluck_pos = 10, pluck_width = 3, output_pos = 15 ;
s15x16 rho = float_to_s15x16(0.0125) ;

void compute_sample(void)
{
    // === 
    // start a burst on new data
    if(note_start) {
      // reset the start flag
      note_start = false ;
      // init the drive amplitude
      //current_amp = attack_inc ;     
      // init pde along string
			// pluck the string
			// this is a set up for zerro initial velocity with
			// the strike as a position
			for (int i=1; i<string_size-2; i++){
				// Gaussian pluck
				string_n[i] = init_string[i];
				string_n_1[i] = string_n[i] ;      	
			}
			string_n[0] = 0;
			string_n_1[0] = 0;
			string_n[string_size-1] = 0;
			string_n_1[string_size-1] = 0;
    } // note start

    // play the PDE
    else {
      for (int i=1; i<string_size-1; i++){          
         // new_string_temp = (string_n[i-1] + string_n[i+1] - times2pt0(string_n[i]))>>1 ;
          new_string_temp = muls15x16(rho, (string_n[i-1] + string_n[i+1] - times2pt0(string_n[i]))) ;
          new_string[i] = times0pt9999(new_string_temp + times2pt0(string_n[i]) - times0pt9999(string_n_1[i])) ;       
      }
      memcpy(string_n_1, string_n, copy_len);
      memcpy(string_n, new_string, copy_len);
      
      int temp = s15x16_to_int(string_n[output_pos]);
      DAC_data = (DAC_config_chan_A | (( temp + 2048) & 0xfff))  ;

      // Write data to DAC
     // spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
      // nonblocking SPI write
      spi0_hw->dr = DAC_data ;
        // move time ahead
        //note_time += onefix ;
        // save in buffer for FFT
      if(buffer_index < N_WAVE){
          dds[buffer_index++] = (short)(temp<<2) ;
      }
    } // current amp > 0   
} // end ISR call


// ==================================================
// === fft thread -- RUNNING on core 1
// ==================================================
//  
static PT_THREAD (protothread_fft(struct pt *pt))
{
    PT_BEGIN(pt);
     static short time_column ;
     static short fr_disp ;
     // green > yellow > red
     static short color_map[10] ={0, 0, 0, 4, 5, 2, 9, 11, 11, 8};
     //static short color_map[20] ={0, 0, 0, 0, 0, 0, 0, 4, 4, 5, 2, 9, 8, 8, 15};
     static short thread_time ;
    
     PT_INTERVAL_INIT() ;

     FFTinit();
        time_column = 110; 
      while(1) {
        //
        //wait for synthesis ISR to fill the FFT buffer
        PT_YIELD_UNTIL(pt, buffer_index == N_WAVE) ;
        // and copy dds to real FFT input
        memcpy(fr, dds, N_WAVE*2) ;
        // start loading next buffer in ISR
        buffer_index = 0 ;
        //
       // for (int i=0; i<N_WAVE/2; i++){
        //    fr[i] = dds_to_s1x14(fr[i]) ;
        //}
        
        // compute FFT on ADC buffer and draw one column
        
        // do the windowing
        for (int i = 0; i < N_WAVE; i++) {
          fr[i] = muls1x14(fr[i], window[i]); 
          fi[i] = 0;
        }

        // do the FFT
        FFTfix(fr, fi, LOG2_N_WAVE);
        //
        // compute power spectrum
        magnitude(fr, fi, N_WAVE);
        // tale log approx
        log2_approx(fr, N_WAVE/2) ;

        
        // plot a vertical slice for spectrogram
        // Spectrogram -- draw and move right
        // wrap the screen
        // at right edge of screen, reset to left edge
        time_column+=2 ;
        if (time_column == 636) time_column = 50;
        // clear two lines ahead of draw point
        drawVLine(time_column+2, 170, 310, BLACK);
        drawVLine(time_column+3, 170, 310, BLACK);
        for(int i=1; i<310; i++){
          //
          // bound log to 0 to 15
          fr_disp = color_map[min(9, max((fr[i]>>2), 0))] ; //4-1
          drawPixel(time_column, 480-i, fr_disp ) ;
          drawPixel(time_column+1, 480-i, fr_disp ) ;
          
        } 

        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // FFT thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){ 

    // fire off synthesis interrupt
    alarm_in_us(alarm_period);



    //  === add threads  ====================
    // for core 1
    //pt_add_thread(protothread_FM) ;
   //pt_add_thread(protothread_fft) ;
    pt_add_thread(protothread_play) ;
    //
    // === initalize the scheduler ==========
    pt_schedule_start ;
    // NEVER exits
    // ======================================
}

// ========================================
// === core 0 main
// ========================================
int main(){

  // dds table for FM synth
  int i = 0 ;
  //while (i<256) {
    // sine table is in naural +1/-1 range
  //  wave_table[i] = float_to_fix(sin(2*3.1416*i/256)) ;
  //  i++ ;
 // }

  // start the serial i/o
  stdio_init_all() ;
  // announce the threader version on system reset
  printf("\n\rProtothreads RP2040 v1.3 two-core, priority\n\r");

  // Initialize SPI channel (channel, baud rate set to 20MHz)
  // connected to spi DAC
spi_init(SPI_PORT, 20000000) ;
// Format (channel, data bits per transfer, polarity, phase, order)
spi_set_format(SPI_PORT, 16, 0, 0, 0);

// Map SPI signals to GPIO ports
//gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
// monitor ISR
 gpio_init(2) ;	
 gpio_set_dir(2, GPIO_OUT) ;

  // Initialize the VGA screen
  initVGA() ;

  for (int i=1; i<string_size-2; i++){
				// Gaussian pluck
				init_string[i] = float_to_s15x16(800 * exp(-(float)(i - pluck_pos)*(i - pluck_pos)/(pluck_width * pluck_width)));    	
			}
     
  // start core 1 threads
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // === config threads ========================
  // for core 0
  pt_add_thread(protothread_fft) ;
  pt_add_thread(protothread_graphics);
  //pt_add_thread(protothread_toggle25);
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;
  // NEVER exits
  // ===========================================
} // end main