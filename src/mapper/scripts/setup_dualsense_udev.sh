#!/usr/bin/env bash
# setup_dualsense_udev.sh
# Configures an isolated udev rule for the Sony PS5 DualSense controller
# (Vendor ID: 054c, Product ID: 0ce6) to grant user-level write permissions
# exclusively to the DualSense LEDs (sysfs interface).
#
# NOTE: We explicitly keep hidraw restricted to 0600 (root only).
# If hidraw has user write permissions, SDL2 (in ROS joy_node) automatically
# switches to its internal HIDAPI driver, which maps D-Pad to buttons instead of
# axes 6 & 7, reducing axes from 8 to 6 and breaking ps5_mapper!
# Keeping hidraw at 0600 ensures joy_node always operates in standard 8-axis mode.

set -e

RULE_FILE="/etc/udev/rules.d/70-dualsense.rules"

echo "Configuring DualSense LED udev rules at ${RULE_FILE}..."

sudo tee "${RULE_FILE}" > /dev/null << 'EOF'
# 1. Explicitly keep hidraw at 0600 so SDL2 / joy_node stays in standard 8-axis evdev mode
KERNEL=="hidraw*", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", MODE="0600"
KERNEL=="hidraw*", KERNELS=="*054C:0CE6*", MODE="0600"

# 2. Sony DualSense RGB Lightbar & Player LEDs sysfs attributes (matches parent HID device 054C:0CE6)
# Grants non-root write access exclusively to the LED brightness and RGB intensity files.
SUBSYSTEM=="leds", KERNELS=="*054C:0CE6*", KERNEL=="*:rgb:indicator*", ACTION=="add|change", RUN+="/bin/chmod 0666 /sys%p/brightness /sys%p/multi_intensity"
SUBSYSTEM=="leds", KERNELS=="*054C:0CE6*", KERNEL=="*:white:player-*", ACTION=="add|change", RUN+="/bin/chmod 0666 /sys%p/brightness"
EOF

echo "Reloading udev daemon rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reset any active hidraw devices back to 0600
for h in /dev/hidraw*; do
    if [ -e "$h" ]; then
        sudo chmod 0600 "$h" 2>/dev/null || true
    fi
done

# Immediately grant write permissions on any currently connected DualSense LEDs
for p in /sys/class/leds/*:rgb:indicator*; do
    if [ -d "$p" ] && readlink "$p" | grep -qi "054c:0ce6"; then
        echo "Updating permissions on active DualSense RGB LED: $p"
        sudo chmod 0666 "$p/brightness" "$p/multi_intensity" 2>/dev/null || true
    fi
done
for p in /sys/class/leds/*:white:player-*; do
    if [ -d "$p" ] && readlink "$p" | grep -qi "054c:0ce6"; then
        echo "Updating permissions on active DualSense Player LED: $p"
        sudo chmod 0666 "$p/brightness" 2>/dev/null || true
    fi
done

echo "Done! PS5 DualSense permissions configured: LEDs writable (0666), hidraw safe (0600)."
