#!/bin/bash
for file in *.wav; do
    ffmpeg -i "$file" -filter:a "volume=2.0" "output_${file}"
done
