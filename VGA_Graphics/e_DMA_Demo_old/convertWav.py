#!/usr/bin/env python3

import numpy as np
import scipy.io.wavfile as wav
import argparse
import os

def convert_wav_to_c_array(input_wav, output_file, channel='B'):
    # Read WAV file
    Fs, data = wav.read(input_wav)

    if data.ndim > 1:
        data = data[:, 0]  # Use one channel if stereo

    # Normalize to range -1.0 to 1.0
    data = data.astype(np.float32)
    data = data / np.max(np.abs(data))

    # Scale to DAC range (0–4095)
    dac_vals = ((data * 2047) + 2048).clip(0, 4095).astype(np.uint16)

    # Prepend DAC config bits
    if channel.upper() == 'A':
        header = 0b0011  # Channel A
    elif channel.upper() == 'B':
        header = 0b1011  # Channel B
    else:
        raise ValueError("Invalid channel: must be 'A' or 'B'")

    # Final values with DAC header
    dac_out = [(header << 12) | val for val in dac_vals]

    # Get base variable name from output file
    varname = os.path.splitext(os.path.basename(output_file))[0]

    # Write to .c file
    with open(output_file, "w") as f:
        f.write("#include <stdint.h>\n\n")
        f.write(f"const unsigned short {varname}[] = {{\n")
        for i, val in enumerate(dac_out):
            f.write(f"    0x{val:04x},\n" if i % 12 == 0 else f"0x{val:04x}, ")
        f.write("};\n\n")
        f.write(f"const unsigned int {varname}_len = {len(dac_out)};\n")

    print(f"Converted '{input_wav}' -> '{output_file}' ({len(dac_out)} samples)")

# CLI Interface 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert .wav file to C array for DAC DMA playback.")
    parser.add_argument("input", help="Input .wav file")
    parser.add_argument("-o", "--output", default="track.c", help="Output .c file")
    parser.add_argument("-c", "--channel", choices=["A", "B"], default="B", help="DAC channel to use (A or B)")

    args = parser.parse_args()
    convert_wav_to_c_array(args.input, args.output, args.channel)
