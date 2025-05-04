#!/bin/bash
set -e

echo "Building code..."
pio run

echo "Copying to Windows path..."
cp .pio/build/teensy41/firmware.hex /mnt/c/temp/firmware.hex

echo "Uploading firmware..."
/mnt/c/tools/teensy_loader_cli/teensy_loader_cli.exe \
  --mcu=TEENSY41 -w -v C:\\temp\\firmware.hex
