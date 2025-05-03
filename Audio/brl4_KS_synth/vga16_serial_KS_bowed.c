/**
 * Bruce Land and Hunter Adams (vha3@cornell.edu)
 * ======================================================
 * Karpus-Strong string synth
 * pluck and bow simulation
 * ======================================================
 * To visualize FFT results you need video:
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
 * 
 * ==========
 * Support routines:
 * -- Define two different fixed point schemes
 *      One for KS which requires bigger numbers, the other for FFT which needs fast execution.
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
 * -- Graphics thread which generates some text, then currently does nothing//
 * 
 * ==========
 * Core1:
 * 
 * -- synthesis ISR triggered by a timer alarm
 * ---- runs at 20KHz (50 uSec interval) for audio synthesis.
 * ---- Worst case execution time is about 3 uSec.
 * ---- Computes Karplus-Strong string sound
 * ---- Pushes output to an SPI channel
 * ---- Fills an array with DDS results for the FFT thread
 * 
 * -- FFT thread which waits for a buffer full signal from the ISR, then draws spectra
 * ---- Does a 2048 point FFT then gets the amplitude of the frequency bins and takes a VERY approx log2
 * ---- plots the amlitude spectrum, log amplitude spectrum, and spectrogram
 * 
 * -- FM synthesis control thread which sequences the synthesis ISR
 * ---- Handles the serial interface for setting parameters
 * ---- precomputes a bunch of fixed point constants to make the ISR faster
 * 
 * -- A play thread which
 * ---- sequences a C scale of 8 notes.
 * 
 * PIO: There are three PIO state machines that generate the video.
 * See: https://vanhunteradams.com/Pico/VGA/VGA.html
 * 
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
// === VGA graphics library
// ==========================================
#include "vga16_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
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

// =========================================
// === KS parameters
// =========================================
// sample rate
 float Fs = 20000 ;
// # of notes in one octave
#define n_note 12 
// index into the note tables
volatile int current_note ;
// max size ~100 Hz -- could be bigger
#define max_string_size 200
s15x16 string[max_string_size] ;
s15x16 init_string[4][max_string_size];

// simulationg input
int bow_type = 0 ;
s15x16 bow_table[4][max_string_size] ;
s15x16 bow_phase_accum ;
s15x16 bow_phase_inc[n_note] ={float_to_s15x16(256/77.0), float_to_s15x16(256/73.0), float_to_s15x16(256/68.0), float_to_s15x16(256/64.0),
                              float_to_s15x16(256/60.0), float_to_s15x16(256/57.0), float_to_s15x16(256/55.00), float_to_s15x16(256/52.00),
                              float_to_s15x16(256/48.0), float_to_s15x16(256/45.0), float_to_s15x16(256/42.0), float_to_s15x16(256/40.0)} ;
// current sring pointers
int ptrin, ptrout ;


//  notes C4	C#	    D	    Eb	    E	    F	   F#	   G	    G#	   A	    Bb	    B4
//        261.6	277.2	293.7	311.1	329.6	349.2	370.0	392.0	415.3	440.0	466.2	493.9
//  matlab length 76.4526   72.1501   68.0967   64.2880   60.6796   57.2738   54.0541   51.0204
//                48.1580   45.4545   42.9000   40.4940

volatile int   string_length[n_note] = {76,  72,  68,  64,  60,  57,  54,  51,  48,  45,  42, 40};
// fractional length
//volatile fixAccum  fine_tune[n_note] = {.45, .15, .10, .29, .68, .27, .05, .02, .26, .45, .90, .49};
// eta = (1-fine_tune)/(1+fine_tune)
volatile s15x16  eta [n_note] = {float_to_s15x16(0.3768), float_to_s15x16(0.739), float_to_s15x16(0.8237), 
         float_to_s15x16(0.5528), float_to_s15x16(0.1908), float_to_s15x16(0.57), float_to_s15x16(0.8974), 
         float_to_s15x16(0.96),float_to_s15x16(0.727), float_to_s15x16(0.375),float_to_s15x16( 0.0526), float_to_s15x16(0.3387) }; 

// 0 is random, 1 is lowpassed random;  2 is gaussian, 3 is sawtooth
int current_pluck = 0 ;
bool pluck_enable = true ;

// gaussian
int pluck_pos = 20, pluck_width = 10, output_pos = 15 ;
//s15x16 pluck_amp = float_to_s15x16(1.0);
// sawtoth
//int saw_length = 10 ;

// filter variables for string
volatile s15x16 tuning_out, last_tune_out, last_tune_in ;
volatile s15x16 lowpass_out  ;
// single pole IIR lowpass
volatile s15x16  low_pass_coeff = float_to_s15x16(0.95) ; // close to classic Karplus Strong

// damping coefficient reduction in amp per synthesis sample
// mmust be <=1.0
volatile s15x16  damping_out, damping_coeff = float_to_s15x16(0.995)  ;
s15x16 final_damping = float_to_s15x16(0.25);

int note_duration = 1000000 ; // 1 sec


// driving the string (as if bowing)
// time since pluck
int sample_number ;
float drive_amp = 0 ;
// rise/fall time of impulse in samples
float drive_rise=1e4, drive_fall=4000 ;
s15x16 drive_rise_inc, drive_fall_inc ;
s15x16 current_drive_amp, string_input ;
// bow frequency relative to string fundamental
float bow_freq = 2.3 ;


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
// for FFT calulation
#define s15x16_to_s1x14(a)  ((s1x14)((((s15x16)(a)>>14)))) 

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
// 20 KSamples/sec
volatile int alarm_period = 50 ;
// signals the ISR to pluck the string
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
#define N_WAVE          1024  ///2048 //1024    
#define LOG2_N_WAVE     10 //10     /* log2(N_WAVE) 0 */

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
    setCursor(300, 5) ;
    writeStringBig("ECE 4760") ;
    setCursor(290, 5+14) ;
    writeStringBig("Karplus_Strong ") ;
    setCursor(300, 5+14*2) ;
    writeStringBig("string synth") ;

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
// === FM parameter setup core1
// ==================================================
// 

int print_col[3] = {50, 250, 450};
int print_row[5] = {60, 80, 100, 120, 140 } ;
char print_str[30] ;

// shoot all the paramaters to the VGA display
void print_all(){
  setTextColor2(WHITE, BLACK);
  setCursor(print_col[0], print_row[0]);
  sprintf(print_str, "pluck (1/0)=%d   ", pluck_enable);
  writeStringBig(print_str) ;

  setCursor(print_col[1], print_row[0]);
  sprintf(print_str, "plucktype (0-3)=%d   ", current_pluck);
  writeStringBig(print_str) ;

  setCursor(print_col[2], print_row[0]);
  sprintf(print_str, "pluckwidth (1-15)=%d   ", pluck_width);
  writeStringBig(print_str) ;

  setCursor(print_col[0], print_row[1]);
  sprintf(print_str, "damping=%5.3f   ", s15x16_to_float(damping_coeff));
  writeStringBig(print_str) ;

  setCursor(print_col[1], print_row[1]);
  sprintf(print_str, "lowpass=%5.3f   ", s15x16_to_float(low_pass_coeff));
  writeStringBig(print_str) ;

  setCursor(print_col[2], print_row[1]);
  sprintf(print_str, "note duration=%5.2f   ", (float)note_duration/1e6);
  writeStringBig(print_str) ;

  setCursor(print_col[0], print_row[2]);
  sprintf(print_str, "bow attack=%5.3f   ", (drive_rise)/Fs);
  writeStringBig(print_str) ;

  setCursor(print_col[1], print_row[2]);
  sprintf(print_str, "bow decay=%5.3f   ", (drive_fall)/Fs);
  writeStringBig(print_str) ;

  setCursor(print_col[2], print_row[2]);
  sprintf(print_str, "bow amp=%5.0f   ", (drive_amp));
  writeStringBig(print_str) ;

  setCursor(print_col[1], print_row[3]);
  sprintf(print_str, "bow frequency=%5.3f   ", bow_freq);
  writeStringBig(print_str) ;

  setCursor(print_col[2], print_row[3]);
  sprintf(print_str, "bow type=%d   ", bow_type);
  writeStringBig(print_str) ;

}


///==============================================

static PT_THREAD (protothread_serial(struct pt *pt))
{
  PT_BEGIN(pt);
    
    static char serial_buffer[40];
    static char cmd[16], arg1[16], arg2[16], arg3[16] ;
    static char* token ;
    float farg ;
    int arg ;

    //
    print_all() ;
    printf("Type 'help' for commands\n\r") ;
    //
    while(1) {

      // print prompt
        sprintf(pt_serial_out_buffer, "KS cmd> ");
        // spawn a thread to do the non-blocking write
        serial_write ;

        // spawn a thread to do the non-blocking serial read
         serial_read ;
        // tokenize
        token = strtok(pt_serial_in_buffer, "  ");
        strcpy(cmd, token) ;
        token = strtok(NULL, "  ");
        strcpy(arg1, token) ;
        token = strtok(NULL, "  ");
        strcpy(arg2, token) ;
        token = strtok(NULL, "  ");
        strcpy(arg3, token) ;
      // conversion to intrnal units
      // increment = Fout/Fs * 2^32
      // octave number is based on a C3 to C4 table
      // parse by command
        if(strcmp(cmd,"help")==0){
            // filter design commands
            printf("***\n\rplay -- play scale\n\r");          
            printf("stop -- playng\n\r");
            //
            printf("pluck 1/0 -- enable initial pluck\n\r") ; 
            printf("plucktype 0-3 -- noise, lp noise, gaussian, sawtooth\n\r") ;  
            printf("pluckwidth -- 1 to 20 for type 2 gaussian \n\r") ;  
            //
            printf("bowattack time -- rise time in seconds\n\r");
            printf("bowdecay time -- fall time in seconds\n\r"); 
            printf("bowamp amplitude -- integer 0 to 1000 \n\r"); 
            printf("bowtype 0-3 -- integer \n\r"); 
            printf("bowfreq relative_freq -- relative to string fundamental\n\r");
            //
            printf("damping coeff -- 0.8 to 1.0\n\r");
            printf("lowpass coeff -- 0.5 to 1.0  \n\r");
            printf("tempo time -- seconds  \n\r");
          
            printf("***\n\r");
        }
        //
        else if(strcmp(cmd,"play")==0){
            play_scale = true ;
        }

        else if(strcmp(cmd,"stop")==0){
            play_scale = false ;
        }

        else if(strcmp(cmd,"pluck")==0){
            sscanf(arg1,"%d", &pluck_enable) ;
        }

        else if(strcmp(cmd,"plucktype")==0){
            sscanf(arg1,"%d", &current_pluck) ;
        }

        else if(strcmp(cmd,"pluckwidth")==0){
            sscanf(arg1,"%d", &pluck_width) ;
            for (int i=0; i<max_string_size; i++){
            // Gaussian pluck
              init_string[2][i] = float_to_s15x16(2000 * exp(-(float)(i - pluck_pos)*(i - pluck_pos)/(pluck_width * pluck_width)));  
            }
        }

        else if(strcmp(cmd,"bowattack")==0){
            sscanf(arg1, "%f", &farg);
            drive_rise = (farg * Fs) ;
            drive_rise_inc = float_to_s15x16(drive_amp / drive_rise) ;
        }

        else if(strcmp(cmd,"bowdecay")==0){
            sscanf(arg1, "%f", &farg);
             drive_fall = (farg * Fs) ;
            drive_fall_inc = float_to_s15x16(drive_amp / drive_fall) ;
        }

         else if(strcmp(cmd,"bowamp")==0){
            sscanf(arg1, "%f", &farg);
            drive_amp = (farg) ;
            drive_fall_inc = float_to_s15x16(drive_amp / drive_fall) ;
            drive_rise_inc = float_to_s15x16(drive_amp / drive_rise) ;
        }

         else if(strcmp(cmd,"bowfreq")==0){
            sscanf(arg1, "%f", &farg);
            bow_freq = (farg) ;
            // bowing tables for DDS
            for (int i=0; i<256; i++){
              bow_table[0][i] = float_to_s15x16(1000 * sin(bow_freq*6.28*(float)i/256.)) ;
              //bow_table[1][i] = float_to_s15x16(1000 * sin(4.2*3.14149*(float)i/256.)) ;
              bow_table[1][i] = float_to_s15x16(1000 * 2*(sin(bow_freq*6.28*(float)i/256.)>0)-1) ;
              //bow_table[3][i] = int_to_s15x16((rand() % 2048) - 1024) ;  
            }
        }

        else if(strcmp(cmd,"bowtype")==0){
            sscanf(arg1,"%d", &bow_type) ;
        }

         else if(strcmp(cmd,"damping")==0){
            sscanf(arg1, "%f", &farg);
            damping_coeff = float_to_s15x16(farg) ;
        }

         else if(strcmp(cmd,"lowpass")==0){
            sscanf(arg1, "%f", &farg);
            low_pass_coeff = float_to_s15x16(farg) ;
        }

         else if(strcmp(cmd,"tempo")==0){
            sscanf(arg1, "%f", &farg);
            note_duration = (int)(farg * 1e6) ;
        }

        else printf("***huh? bad command*** \n\r");
        //
        print_all();
      

      // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // timer thread

// ==================================================
// === play the scale thread on core 1
// ==================================================


static PT_THREAD (protothread_play(struct pt *pt))
{
    PT_BEGIN(pt);
    
     // data structure for interval timer
     PT_INTERVAL_INIT() ;
     // compute rise/fall of drive
     drive_rise_inc = float_to_s15x16(drive_amp / drive_rise) ;
     drive_fall_inc = float_to_s15x16(drive_amp / drive_fall) ;

      while(1) {
        static int i ;
        PT_YIELD_usec(1000) ;
        if(play_scale){

            //mod_attack_inc = div(current_depth, mod_attack_time) ;
            //mod_decay_inc = div(current_depth, mod_decay_time) ;
            for(i=0; i<n_note; i++){
               // current_main_inc = main_inc[i] ;
                //rho = 
                current_note = i ;
                note_start = true ;
                //PT_YIELD_usec(20000) ;
                // play until too small for DAC
               // PT_YIELD_UNTIL(pt, current_amp<onefix);
                PT_YIELD_usec(note_duration) ;
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


void compute_sample(void)
{
    
    // ==== start a string with new data ===
    if(note_start) {
      // reset the start flag
      note_start = false ;
      // load the string
      for (int i=0; i<=string_length[current_note]; i++) {
         if (pluck_enable) string[i] =  init_string[current_pluck][i] ; 
         else string[i] = 0. ;
      } 
      // reset the pointers
      ptrin = 1;
      ptrout = 2;
      // pluck time marker
      sample_number = 0 ;
      // drive starts at zero
      current_drive_amp = 0 ;
      lowpass_out = 0 ;
      tuning_out = 0 ;
      buffer_index = 0 ;
      bow_phase_accum = 0 ;
      //
    } // note start

    // ==== play the KS ===
    //else if(play_scale) {
      

      bow_phase_accum += bow_phase_inc[current_note] ;
      if (bow_phase_accum>int_to_s15x16(255)) bow_phase_accum = 0 ;
      // === low pass filter ===
      // the following is the classic KS lowpass
      //lowpass_out =  ((string[ptrin] + string[ptrout])>>1) ;

      //this is a 1 pole IIR lowpass, which i prefer
      //lowpass_out = muls15x16 (damping_coeff , (((string[ptrin] - string[ptrout])>>low_pass_coeff) + string[ptrout]));
      lowpass_out = muls15x16(low_pass_coeff, (string[ptrin] - lowpass_out)) + lowpass_out ;
      damping_out = muls15x16(damping_coeff, lowpass_out) ;
      // kill the energy at the end of the note
      if(((50*sample_number) > (note_duration-20000) && pluck_enable==0)) damping_out = muls15x16(final_damping, damping_out) ;

      // === tuning all-pass filter ===
      tuning_out= muls15x16 (eta[current_note] , (damping_out - last_tune_out)) + last_tune_in ;
      // all-pass state vars
      last_tune_out = tuning_out;
      last_tune_in = damping_out ; //lowpass_out;

      // input calculations
    if ((sample_number)<(int)(drive_rise + drive_fall)){
      // adjust scale so that drive  ~1
        // and sample number is periodic with the length of the sting
        string_input = muls15x16((current_drive_amp)>>12 , bow_table[bow_type][s15x16_to_int(bow_phase_accum)] );

        current_drive_amp = ((sample_number) < (int)(drive_rise))? 
            current_drive_amp + drive_rise_inc : 
            current_drive_amp - drive_fall_inc ;
    }
    else {
        string_input = 0 ;
        current_drive_amp = 0 ;
    }

      // === string feedback ===
      // depending on the damping, need to attenuate input
      string[ptrin] = tuning_out + (string_input>>2) ; ///(ptrin==20)?(string_input) : 0 ;(sample_number)?
      // could add input here

      // === DAC sscaling and output ===
      int temp = s15x16_to_int(string[ptrin]);
      DAC_data = (DAC_config_chan_A | (( temp + 2048) & 0xfff))  ;
              
     // === update and wrap pointers ===
      if (ptrin==string_length[current_note]) ptrin=1;
      else ptrin=ptrin+1;
     //
      if (ptrout==string_length[current_note]) ptrout=1; 
      else ptrout=ptrout+1;  

      // Write data to DAC
      // spi_write16_blocking(SPI_PORT, &DAC_data, 1) ;
      // NOTE === nonblocking SPI write ===tempo 4

      spi0_hw->dr = DAC_data ;
      
      // keep track of samples for input drive
      sample_number++;

        // === save in buffer for FFT ===
      if(buffer_index < N_WAVE){
          dds[buffer_index++] = (short)(temp<<2) ;
      }
    //} // current amp > 0    
} // end ISR  


// ==================================================
// === fft thread -- RUNNING on core 1
// ==================================================
//  draw the screen 
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
        time_column+=1 ;
        if (time_column == 636) time_column = 50;
        // clear two lines ahead of draw point
        drawVLine(time_column+2, 170, 310, BLACK);
        drawVLine(time_column+3, 170, 310, BLACK);
        for(int i=1; i<310; i++){
          //
          // bound log to 0 to 15
          fr_disp = color_map[min(9, max((fr[i]>>3), 0))] ; //4-1
          drawPixel(time_column, 480-i, fr_disp ) ;
          //drawPixel(time_column+1, 480-i, fr_disp ) ;
          
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
    pt_add_thread(protothread_serial) ;
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

  // set up the sring initial conditions arrays
  for (int i=0; i<max_string_size; i++){
    // random pluck
    init_string[0][i] = int_to_s15x16((rand() % 2048) - 1024);  
    // slghgtly lowpassed random drive to soften pluck transient
    if(i>3){
      init_string[1][i] = (init_string[0][i-3] + init_string[0][i-2] + init_string[0][i-1] + init_string[0][i])>>2; 
    }
   // Gaussian pluck
    init_string[2][i] = float_to_s15x16(800 * exp(-(float)(i - pluck_pos)*(i - pluck_pos)/(pluck_width * pluck_width)));  
    // sawtooth
    init_string[3][i] = (i<11)?int_to_s15x16(i * 100) : (i<22)? int_to_s15x16(1000 - i * 100) : 0 ; //int_to_s15x16(1000-(i-10)*2);
	}

  // bowing tables for DDS
  for (int i=0; i<256; i++){
    bow_table[0][i] = float_to_s15x16(1000 * sin(bow_freq*6.28*(float)i/256.)) ;
    //bow_table[1][i] = float_to_s15x16(1000 * sin(4.2*3.14149*(float)i/256.)) ;
    bow_table[1][i] = float_to_s15x16(1000 * 2*(sin(bow_freq*6.28*(float)i/256.)>0)-1) ;
    //bow_table[3][i] = int_to_s15x16((rand() % 2048) - 1024) ;  
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