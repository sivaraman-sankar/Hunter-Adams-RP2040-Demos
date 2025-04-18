/**
 * Lab 3: PID Control of a 1D Helicopter
 *
 * Adapted from PWM_Demo code and MPU6050_IMI_Demo code by V. Hunter Adams (vha3@cornell.edu)
 *
 * HARDWARE CONNECTIONS
 * -  * - GPIO 15 ---> Pushbutton switch
`
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
 *  - GPIO 4 ---> PWM from MCU circuit input
 *
 **/

/* ===== IMPORTS ===== */

// Standard libraries
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

/* ===== DEFINITIONS ===== */

// PWM wrap value and clock divide value
// For a CPU rate of 125 MHz, this gives a PWM frequency of 1 kHz. ie. 125Mhz/25/5000 = 1KHz
#define WRAPVAL 5000
#define CLKDIV 25.0f
#define PWM_OUT 4 // GPIO we're using for PWM
#define X_DIMENSION 640
#define Y_DIMENSION 480
int pid_display_x = X_DIMENSION - 100;
#define BUTTON_PIN 15

// Debounce FSM States
#define NOT_STARTED -1
#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define PRESSED 2
#define MAYBE_NOT_PRESSED 3
#define FIVE_SEC 167
#define TEN_SEC 334
#define FIFTEEN_SEC 501

// Some macros for max/min/abs
#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a < b) ? b : a)
#define abs(a) ((a > 0) ? a : -a)

/* ===== VGA SETUP ===== */

// Semaphore for vga display thread
static struct pt_sem vga_semaphore;
// character array
char screentext[40];
// draw speed
int threshold = 10;

/* ===== IMU SETUP ===== */

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 accel_angle;
fix15 gyro_angle_delta;
fix15 complementary_angle;

// PID control parameters
// Todo!: Need to identify the initial value of kp
// they are volatile because they are modified in the ISR
volatile fix15 K_p = float2fix15(30.0f);
volatile fix15 K_i = float2fix15(1.0f);
volatile fix15 K_d = float2fix15(10000.0f);
volatile fix15 motor_disp;

// angle parameters
volatile fix15 desired_angle = int2fix15(0); // Initial desired angle is 90 degrees
fix15 thousand_f = int2fix15(1000);
fix15 error_angle;
fix15 filtered_ay;
fix15 filtered_az;

/* ===== PWM SETUP ===== */

// Variable to hold PWM slice number
uint slice_num;

// PWM duty cycle - note "control" implies duty cylce within this file.
volatile int control;
volatile int old_control;

volatile fix15 proportional_control;
volatile fix15 integral_control;
volatile fix15 derivative_control;

// angles array to store the past 20 angles
fix15 angles[5];

// Initialize to button pressed, so that inital angle is 0
volatile int button_state = NOT_PRESSED;

void setupButton()
{
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN); // Enable internal pull-up resistor
}

/* ===== BUTTON THREAD ===== */

static PT_THREAD(protothread_button(struct pt *pt))
{
    PT_BEGIN(pt);
    static int possible, elapsed_time, button_released = 0;
    while (1)
    {
        elapsed_time += 1;
        int button_read = !gpio_get(BUTTON_PIN); // Active LOW button
        switch (button_state)
        {
        case NOT_PRESSED:
            if (button_read)
            {
                possible = button_read;
                button_state = MAYBE_PRESSED;
            }

            if (button_released)
            {
                // Button has been released: angle change sequence:
                // 90 for 5 seconds, 120 for 5 seconds, 60 for 5 seconds, 90 for 5 seconds
                if (elapsed_time < FIVE_SEC)
                {
                    desired_angle = int2fix15(90);
                }
                else if (elapsed_time < TEN_SEC)
                {
                    desired_angle = int2fix15(120);
                }
                else if (elapsed_time < FIFTEEN_SEC)
                {
                    desired_angle = int2fix15(60);
                }
                else
                {
                    // elapsed_time = 0;
                    desired_angle = int2fix15(90);
                }
            }

            break;
        case MAYBE_PRESSED:
            if (button_read == possible)
            {
                button_state = PRESSED;
            }
            else
            {
                button_state = NOT_PRESSED;
            }

            break;
        case PRESSED:
            if (!button_read)
            {
                button_state = MAYBE_NOT_PRESSED;
            }

            // set angle to 0 degrees
            desired_angle = int2fix15(0);
            elapsed_time = 0;
            break;
        case MAYBE_NOT_PRESSED:
            if (!button_read)
            {
                button_state = NOT_PRESSED;
            }
            else
            {
                button_state = PRESSED;
            }
            button_released = 1;
            break;
        }
        PT_YIELD_usec(30000); // Debounce delay
    }
    PT_END(pt);
}

/* ====== PID ISR ===== */

// PID interrupt service routine
void on_pwm_wrap()
{
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));

    // Reads the IMU to get raw accelerometer/gyro measurements
    // Note - this is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // print acceleartion array
    // printf("Accel X: %f \t", fix2float15(acceleration[0]));
    // printf("Accel Y: %f \t", fix2float15(acceleration[1]));
    // printf("Accel Z: %f \n", fix2float15(acceleration[2]));

    // Low pass filter the accelerometer data to eliminate noise
    filtered_ay = filtered_ay + ((acceleration[1] - filtered_ay) >> 4);
    filtered_az = filtered_az + ((acceleration[2] - filtered_az) >> 4);

    accel_angle = multfix15(float2fix15(atan2(-filtered_ay, filtered_az) + 3.14f), oneeightyoverpi);

    // subtract 270 degrees to get the angle in the correct orientation
    accel_angle = int2fix15(270) - accel_angle;

    gyro_angle_delta = multfix15(gyro[0], zeropt001);
    complementary_angle = multfix15(complementary_angle + gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // print the state for debugging
    // printf("Accel Angle: %f \n", fix2float15(accel_angle));
    // printf("Gyro Angle Delta: %f \n", fix2float15(gyro_angle_delta));
    // printf("Complementary Angle: %f \n", fix2float15(complementary_angle));

    // Runs the PID control loop at 1000/sec using the angle estimates from the complementary filter
    error_angle = desired_angle - complementary_angle;

    // update the angles array with the new angle
    for (int i = 0; i < 4; i++)
    {
        angles[i] = angles[i + 1];
    }
    angles[4] = complementary_angle;

    // Apply proportional gain to compute new control input
    proportional_control = multfix15(K_p, error_angle);

    // Apply integral gain to compute new control input
    integral_control = integral_control + error_angle;

    // bound the intergral gain to prevent windup
    if (integral_control > int2fix15(2500))
    {
        integral_control = int2fix15(2500);
    }
    else if (integral_control < int2fix15(-2500))
    {
        integral_control = int2fix15(-2500);
    }

    // Apply derivative gain to compute new control input
    derivative_control = multfix15(K_d, angles[0] - angles[4]);

    // control is the sum of the proportional, integral, and derivative control inputs
    control = fix2int15(proportional_control) + fix2int15(multfix15(K_i, integral_control)) + fix2int15(derivative_control);

    if (control > 5000)
    {
        control = 5000;
    }
    else if (control < 0)
    {
        control = 0;
    }

    // Update duty cycle using newly computed control input
    if (control != old_control)
    {
        old_control = control;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, WRAPVAL - control); // Invert output
    }

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

void config_PWM()
{
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
}

/* ===== USER INPUT THREAD ===== */

// Takes user input from the serial interface to setup PID parameters and the desired angle.
static PT_THREAD(protothread_serial_pid(struct pt *pt))
{
    PT_BEGIN(pt);
    static int test_in, user_angle;
    static float user_kp, user_ki, user_kd;
    static int choice;
    while (1)
    {

        // Get command from user
        sprintf(pt_serial_out_buffer, "Enter command, '1' for desired angle, '2' for K_p:, '3' for K_i, '4' for K_d:");
        serial_write;
        serial_read;
        sscanf(pt_serial_in_buffer, "%d", &choice);
        printf("Entered Choice: %d\n", choice);
        switch (choice)
        {
        case 1:
            // Get desired angle from user:
            sprintf(pt_serial_out_buffer, "Input desired angle: ");
            serial_write;
            serial_read;
            // Convert input string to number
            sscanf(pt_serial_in_buffer, "%d", &user_angle);
            if (user_angle != desired_angle)
            {
                desired_angle = int2fix15(user_angle);
            }
            break;
        case 2:
            // Get K_p from user
            sprintf(pt_serial_out_buffer, "Input K_p: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%f", &user_kp);
            K_p = float2fix15(user_kp);
            break;
        case 3:
            // Get K_i from user
            sprintf(pt_serial_out_buffer, "Input K_i: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%f", &user_ki);
            K_i = float2fix15(user_ki);
            break;
        case 4:
            // Get K_d from user
            sprintf(pt_serial_out_buffer, "Input K_d: ");
            serial_write;
            serial_read;
            // convert input string to number
            sscanf(pt_serial_in_buffer, "%f", &user_kd);
            K_d = float2fix15(user_kd);
            break;
        default:
            sprintf(pt_serial_out_buffer, "Invalid command. Please try again.\n");
            serial_write;
            break;
        }
    }
    PT_END(pt);
}

/* ===== VGA THREAD ===== */

// Diplays measured angle and low-passed control input on VGA display.

static PT_THREAD(protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt);

    // We will start drawing at column 81
    static int xcoord = 81;

    // Rescale the measurements for display
    static float OldRange = 145.; // (+/- 250)
    static float NewRange = 150.; // (looks nice on VGA)
    static float OldMin = 0.;
    static float OldMax = 145.;

    // Rescale for bottom plot 
    static float OldRange2 = 5000.; // (+/- 250)
    static float NewRange2 = 150.;   // (looks nice on VGA)
    static float OldMin2 = 0.;
    static float OldMax2 = 5000.;

    // Control rate of drawing
    static int throttle;

    // Draw the static aspects of the display
    setTextSize(1);
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN);
    drawHLine(75, 280, 5, CYAN);
    drawVLine(80, 280, 150, CYAN);
    sprintf(screentext, "5000");
    setCursor(50, 280);
    writeString(screentext);
    sprintf(screentext, "0");
    setCursor(50, 425);
    writeString(screentext);

    // Draw top plot
    drawHLine(75, 230, 5, CYAN);
    // drawHLine(75, 155, 5, CYAN);
    drawHLine(75, 80, 5, CYAN);
    drawVLine(80, 80, 150, CYAN);
    setCursor(50, 150);
    sprintf(screentext, "+145");
    setCursor(45, 75);
    writeString(screentext);
    sprintf(screentext, "0");
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

            // Draw plot of error angle
            // drawPixel(xcoord, 380 - (int)(NewRange * ((float)(fix2float15(error_angle) - OldMin) / OldRange)), YELLOW);

            // Draw plot of the complementary angle
            drawPixel(xcoord, 230 - (int)(NewRange * ((float)(fix2float15(complementary_angle) - OldMin) / OldRange)), WHITE);


            // Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[0]) * 120.0) - OldMin) / OldRange)), WHITE);
            // drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[1]) * 120.0) - OldMin) / OldRange)), RED);
            // drawPixel(xcoord, 430 - (int)(NewRange * ((float)((fix2float15(acceleration[2]) * 120.0) - OldMin) / OldRange)), GREEN);

            // Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[0]))-OldMin) / OldRange)), WHITE);
            // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[1]))-OldMin) / OldRange)), RED);
            // drawPixel(xcoord, 230 - (int)(NewRange * ((float)((fix2float15(gyro[2]))-OldMin) / OldRange)), GREEN);

            // plot the control input 
            motor_disp = motor_disp + ((int2fix15(control) - motor_disp) >> 6);
            drawPixel(xcoord, 430 - (int)((NewRange2 * fix2float15(motor_disp)) / 5000), WHITE);

            // Display current angle and desired angle on VGA
            setCursor(0, 0);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "Current Angle: %0.2f", fix2float15(complementary_angle));
            writeString(screentext);

            setCursor(0, 10);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "Desired Angle: %0.2f", fix2float15(desired_angle));
            writeString(screentext);

            // print error
            setCursor(0, 20);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "Error: %0.2f", fix2float15(error_angle));
            writeString(screentext);

            // print control
            setCursor(0, 30);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "Control: %d      \n", control);
            writeString(screentext);

            // Print PID paramerts on the right side of the screen
            setCursor(pid_display_x, 0);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "K_p: %0.2f     ", fix2float15(K_p));
            writeString(screentext);

            setCursor(pid_display_x, 10);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "K_i: %0.2f     ", fix2float15(K_i));
            writeString(screentext);

            setCursor(pid_display_x, 20);

            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "K_d: %0.2f      ", fix2float15(K_d));

            writeString(screentext);

            // print the proportional control
            setCursor(pid_display_x, 30);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "P: %d     ", fix2int15(proportional_control));
            writeString(screentext);

            // print the integral control
            setCursor(pid_display_x, 40);
            setTextSize(1);
            setTextColor2(RED, BLACK);

            sprintf(screentext, "I: %d       ", fix2int15(multfix15(K_i, integral_control)));

            writeString(screentext);

            // print the derivative control

            setCursor(pid_display_x, 50);
            setTextSize(1);

            sprintf(screentext, "D: %d      ", fix2int15(derivative_control));
            writeString(screentext);

            // print button state
            setCursor(pid_display_x, 60);
            setTextSize(1);
            setTextColor2(RED, BLACK);
            sprintf(screentext, "Button State: %d      ", button_state);
            writeString(screentext);

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

/* ===== MAIN FUNCTION ===== */

// core 1 setup
void core1_main()
{
    // Add animation thread
    pt_add_thread(protothread_vga);
    pt_add_thread(protothread_button);

    // Start the scheduler
    pt_schedule_start;
}

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA();

    // setup button
    setupButton();

    // I2C Configuration
    i2c_init(I2C_CHAN, I2C_BAUD_RATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    // PWM configuration
    config_PWM();

    multicore_reset_core1();
    multicore_launch_core1(&core1_main);

    // Add threads and start scheduler
    pt_add_thread(protothread_serial_pid);
    // pt_add_thread(protothread_vga);
    // pt_add_thread(protothread_button);
    pt_schedule_start;
}
