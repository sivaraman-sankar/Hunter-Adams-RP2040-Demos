/**
 * V. Hunter Adams (vha3@cornell.edu)
 * PWM demo code with serial input
 *
 * This demonstration sets a PWM duty cycle to a
 * user-specified value.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA (blue)
 *  - GPIO 9 ---> MPU6050 SCL (yellow)
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *
// ======================================================
//  Below section are notes from class (Ignore!!) Jump to line : 100 for code
// ======================================================
 *
 * Two processes -
 * 1. Controller (high priority)
 * 2. User interface (low priority)
 *
 * =================================================================
 * Closing the loop
 *
 * 1. Read from the IMU sensor (I2C)
 *  - Gives accelartion
 *  - Angular rate
 *  - Use that for estimating the arm angle
 *
 * Complimentary filter
 *
 * |     /
 * |    /
 * |   /
 * |  /
 * | /
 * |/
 * ___
 *
 * theta = arctan(a_x/a_y)
 *
 * Why gyro ?
 * Noise -- accelerometer is noisy; when motor is running causes vibrations and other pertrubations. Doesn't enable closed loop control.
 *
 * Output = true noise + noise (Zero mean gaussian white noise )
 *
 * Gyro --
 * Angular rate, how quickly its rotating. (w) degrees/second
 * They develop a bias, even if low noise - nice clean output, but they will drift
 *
 * Complimentary filter = retain good properties of both sensors and eliminate the others
 *
 * - This is time sensitive, so move it into ISR
 *
 * - Algo
 *
 * - Gather ax, ay, az
 * - Use 2 of the three, weighted accel angle estimate ( theta = ax/ay*180/pi) -- depends how you mount the IMU sensor
 * ay, az --> point the imu facing down slide (5)
 * - accel weight
 * - Low passing
 *
 * - Gyro
 * - Take gyro reading
 * - Multiply by dt, 1khz = 0.001 second
 * - Gyro weight  ( usually higher than accel weight )
 * - High passing
 *
 * Sum both to get complimentary angle which will be given to PID controller
 *
 * ======================
 * PID
 *  P - proportional to current and desired state (current)
 *  I - current and history of error (past)
 *  D - rate of change and scales it (future)
 *
 *
 * Only P --
 *  - Oscillations (high Kp)
 *  - Steady state error will always occur
 *  - kp.e  ==> gain of the system
 *
 * I -
 *
 * Ki. sigma(e. dt)
 *
 * Mitigates some issues --
 * - allows for controller to notice that steady state error and solve it
 * - oscillations still occur
 * Notes - cap the integral term to some maximal value
 *
 * D -
 * kd. de/dt
 *
 *
 */

// ======================================================
//  Program starts from below
// ======================================================
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

// Custom libaries
#include "vga16_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_3.h"

// ==========================================================
// defines
// ==========================================================

// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives
// a PWM frequency of 1 kHz. ie. 125Mhz/25/5000 = 1KHz
#define WRAPVAL 5000
#define CLKDIV 25.0f

// GPIO we're using for PWM
#define PWM_OUT 4

// ==========================================================
// state variables
// ==========================================================

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 accel_angle;
fix15 gyro_angle_delta;
fix15 complementary_angle;

// character array
char screentext[40];

// draw speed
int threshold = 10;

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

// semaphore
static struct pt_sem vga_semaphore;

// Variable to hold PWM slice number
uint slice_num;

// PWM duty cycle - note "control" implies duty cylce within this file.
volatile int control;
volatile int old_control;

// PID control parameters
// Todo!: Need to identify the initial value of kp
volatile int kp;
volatile int ki;
volatile int kd;

// angle parameters
volatile int desired_angle;
volatile int error;

// PWM interrupt service routine
void on_pwm_wrap()
{
    // 0. Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));

    // 1. Reads the IMU to get raw accelerometer/gyro measurements
    mpu6050_read_raw(acceleration, gyro);

    // 1.1 Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

    // 2. Estimates angle by means of a complementary filter of accel/gyro measurements
    //  Todo! -- I need to determine which parameters of accel and gyro will be useful here
    //  Todo! - need to figure out if small angle approximation is required or not.

    fix15 filtered_ay = acceleration[1] >> 2;
    fix15 filtered_az = acceleration[1] >> 2;

    accel_angle = multfix15(float2fix15(atan2(-filtered_ay, filtered_az) + 3.14f), oneeightyoverpi);
    gyro_angle_delta = multfix15(gyro[2], zeropt001);
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // 3. Runs the PID control loop at 1000/sec using the angle estimates from the complementary filter
    error = desired_angle - complementary_angle;

    // Compute the proportional
    // control = kp * error;

    // 4. Sets a hardware PWM signal using output-compare unit to control the motor using the command

    // Update duty cycle
    if (control != old_control)
    {
        old_control = control;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, WRAPVAL - control);
    }
}

// Thread that draws to VGA display
// Note - Executed on Core 0 for now; Multicore optimization can be done later.
static PT_THREAD(protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt);

    // We will start drawing at column 81
    static int xcoord = 81;

    // Rescale the measurements for display
    static float OldRange = 500.; // (+/- 250)
    static float NewRange = 150.; // (looks nice on VGA)
    static float OldMin = -250.;
    static float OldMax = 250.;

    // Control rate of drawing
    static int throttle;

    // Draw the static aspects of the display
    setTextSize(1);
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN);
    drawHLine(75, 355, 5, CYAN);
    drawHLine(75, 280, 5, CYAN);
    drawVLine(80, 280, 150, CYAN);
    sprintf(screentext, "0");
    setCursor(50, 350);
    writeString(screentext);
    sprintf(screentext, "+2");
    setCursor(50, 280);
    writeString(screentext);
    sprintf(screentext, "-2");
    setCursor(50, 425);
    writeString(screentext);

    // Draw top plot
    drawHLine(75, 230, 5, CYAN);
    drawHLine(75, 155, 5, CYAN);
    drawHLine(75, 80, 5, CYAN);
    drawVLine(80, 80, 150, CYAN);
    sprintf(screentext, "0");
    setCursor(50, 150);
    writeString(screentext);
    sprintf(screentext, "+250");
    setCursor(45, 75);
    writeString(screentext);
    sprintf(screentext, "-250");
    setCursor(45, 225);
    writeString(screentext);

    while (true)
    {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold)
        {
            // Zero drawspeed controller
            throttle = 0;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK);

            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[0]) * 120.0) - OldMin) / OldRange)), WHITE);
            drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[1]) * 120.0) - OldMin) / OldRange)), RED);
            drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[2]) * 120.0) - OldMin) / OldRange)), GREEN);

            // Draw top plot
            drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[0]))-OldMin) / OldRange)), WHITE);
            drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[1]))-OldMin) / OldRange)), RED);
            drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[2]))-OldMin) / OldRange)), GREEN);

            // Update horizontal cursor
            if (xcoord < 609)
            {
                xcoord += 1;
            }
            else
            {
                xcoord = 81;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread for PID parameters
// Accept only k_p for Week 2
static PT_THREAD(protothread_serial_pid(struct pt *pt))
{
    PT_BEGIN(pt);
    static int test_in_angle, test_in_kp;
    while (1)
    {
        sprintf(pt_serial_out_buffer, "accel_angle: %d", accel_angle);
        sprintf(pt_serial_out_buffer, "gyro_angle: %d", gyro);
        sprintf(pt_serial_out_buffer, "comp_angle: %d", complementary_angle);

        // Accept desired angle
        sprintf(pt_serial_out_buffer, "input desired angle: ");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%d", &test_in_angle);
        if (test_in_angle != desired_angle)
        {
            desired_angle = test_in_angle;
        }

        // Accept k_p
        sprintf(pt_serial_out_buffer, "input kp value: ");
        serial_write;
        // spawn a thread to do the non-blocking serial read
        serial_read;
        // convert input string to number
        sscanf(pt_serial_in_buffer, "%d", &test_in_kp);
        kp = test_in_kp;
    }
    PT_END(pt);
}

int main()
{
    sprintf(pt_serial_out_buffer, "Testing the PWM example ...\n");

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA();

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO PWM_OUT that it is allocated to the PWM
    gpio_set_function(PWM_OUT, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO PWM_OUT (it's slice 2)
    slice_num = pwm_gpio_to_slice_num(PWM_OUT);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);                 // enable the slice associated with our gpio channel
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap); // every time it fires, call the on_pwn_wrap funciton
    irq_set_enabled(PWM_IRQ_WRAP, true);                  // enable the irq

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL);
    pwm_set_clkdiv(slice_num, CLKDIV);

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 3125); // 3125 is going to be modifiable by the user.

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    pt_add_thread(protothread_serial_pid);
    pt_add_thread(protothread_vga);
    pt_schedule_start;
}
