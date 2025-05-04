#!/bin/bash
set -e

echo ""
echo "Binding and attaching USB device..."

# Full path to PowerShell
/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -Command "Start-Process usbipd -ArgumentList 'bind --busid 2-2' -Verb RunAs"
/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe -Command "usbipd attach --busid 2-2 --wsl"
