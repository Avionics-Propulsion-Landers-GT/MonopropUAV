#!/bin/bash
set -e

echo "Building code..."
pio run

echo "Copying to Windows path..."
cp .pio/build/teensy41/firmware.hex /mnt/c/temp/firmware.hex

echo "Uploading firmware via PowerShell..."
/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -Command "& 'C:\\tools\\teensy_loader_cli\\teensy_loader_cli.exe' --mcu=TEENSY41 -v -w 'C:\\temp\\firmware.hex'"
