add_executable(MPU6050_IMU)

# must match with pio filename and executable name from above
pico_generate_pio_header(MPU6050_IMU hsync.pio)
pico_generate_pio_header(MPU6050_IMU vsync.pio)
pico_generate_pio_header(MPU6050_IMU rgb.pio)

# must match with executable name and source file names
target_sources(MPU6050_IMU PRIVATE imu_demo.c vga16_graphics.c mpu6050.c)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(MPU6050_IMU pico_stdlib pico_bootsel_via_double_reset pico_multicore hardware_pwm hardware_dma hardware_irq hardware_adc hardware_pio hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(MPU6050_IMU)