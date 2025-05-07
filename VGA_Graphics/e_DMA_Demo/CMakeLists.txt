add_executable(Audio_DMA_Demo)

add_compile_options(-Ofast)

# must match with pio filename and executable name from above
pico_generate_pio_header(Audio_DMA_Demo ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(Audio_DMA_Demo ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(Audio_DMA_Demo ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
# target_sources(Audio_DMA_Demo PRIVATE dj.c karplus_strong vga16_graphics)
target_sources(Audio_DMA_Demo PRIVATE dj.c karplus_strong vga16_graphics)

target_link_libraries(Audio_DMA_Demo pico_stdlib 
hardware_dma 
hardware_spi 
pico_divider 
pico_multicore 
pico_bootsel_via_double_reset 
hardware_pio 
hardware_adc 
hardware_irq 
hardware_clocks 
hardware_pll )

# create map/bin/hex file etc.
pico_add_extra_outputs(Audio_DMA_Demo)

