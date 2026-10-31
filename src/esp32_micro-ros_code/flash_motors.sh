#!/bin/bash
set -e

PORT=${1:-/dev/ttyUSB0}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$SCRIPT_DIR/esp32_microros_motor_controller"
FQBN="esp32:esp32:esp32da:PartitionScheme=huge_app"
ARDUINO_CLI="$(command -v arduino-cli || echo "$HOME/.local/bin/arduino-cli")"

echo "=== Compiling and Flashing 5-Motor + Gripper PWM Controller ==="
echo "Port:   $PORT"
echo "Board:  $FQBN"
echo "Sketch: $SKETCH_DIR"
echo ""

if [ ! -e "$PORT" ]; then
    echo "ERROR: Port $PORT not found! Check connection with 'ls -l /dev/ttyUSB*'"
    exit 1
fi

# Ensure port is not held open by a running agent
if fuser "$PORT" >/dev/null 2>&1; then
    echo "WARNING: $PORT is currently being used by another process (likely micro_ros_agent)."
    echo "Stopping background processes holding $PORT..."
    fuser -k "$PORT" 2>/dev/null || true
    sleep 1
fi

if [ ! -r "$PORT" ] || [ ! -w "$PORT" ]; then
    echo "WARNING: Insufficient permissions for $PORT."
    echo "Run './setup_permissions.sh' or 'sudo chmod 666 $PORT' first."
fi

echo "--- 1. Compiling sketch ---"
"$ARDUINO_CLI" compile --fqbn "$FQBN" "$SKETCH_DIR"

echo ""
echo "--- 2. Uploading sketch to $PORT ---"
echo "TIP: If the upload gets stuck on 'Connecting...', press and hold the 'BOOT' button on the ESP32 until flashing starts."
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"

echo ""
echo "=== SUCCESS! Firmware uploaded successfully. ==="
echo "Next step: Run './run_agent.sh' in one terminal, then './run_motor_talker.sh' in another."
