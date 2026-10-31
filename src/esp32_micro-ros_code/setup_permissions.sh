#!/bin/bash
echo "=== ESP32 Serial Permissions Setup ==="
echo "Adding user '$USER' to the 'dialout' group..."
sudo usermod -aG dialout "$USER"

echo "Temporarily granting read/write access to currently connected serial ports..."
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        sudo chmod 666 "$dev"
        echo "Granted rw permissions on $dev"
    fi
done

echo ""
echo "Done! Note: For permanent group membership to take effect across all shells,"
echo "you may need to log out and log back in, or run 'newgrp dialout' in your current terminal."
