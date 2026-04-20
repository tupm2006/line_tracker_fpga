#!/bin/bash

# ==============================================================================
# Tang Nano 9K Robust Flash Script (v2)
# ==============================================================================
# Captures output to detect CRC/FAIL errors and automatically retries.
# ==============================================================================

BITSTREAM="line_tracker.fs"
BOARD="tangnano9k"
FREQ="2000000"
LOADER="/usr/bin/openFPGALoader"

# Permission check
if ! groups | grep -q 'uaccess' && [ ! -w /dev/bus/usb ]; then
    echo "WARNING: Permission issue. Check udev rules."
fi

if [ ! -f "$BITSTREAM" ]; then
    echo "Error: $BITSTREAM not found. Run 'make' first."
    exit 1
fi

MAX_ATTEMPTS=3
ATTEMPT=1

while [ $ATTEMPT -le $MAX_ATTEMPTS ]; do
    echo "--- Flash Attempt $ATTEMPT of $MAX_ATTEMPTS ---"
    
    # Run the flash command and capture both stdout and stderr
    # We use 'make flash' if you prefer, but calling direct allows freq control
    OUTPUT=$($LOADER -b $BOARD --freq $FREQ -f "$BITSTREAM" 2>&1)
    RET_CODE=$?
    
    echo "$OUTPUT"
    
    # Check for success: Exit code 0 AND no "FAIL" or "CRC" in output
    if [ $RET_CODE -eq 0 ] && ! echo "$OUTPUT" | grep -qi "FAIL" && ! echo "$OUTPUT" | grep -qi "CRC"; then
        echo "SUCCESS: Bitstream verified and loaded."
        $LOADER -b $BOARD --reset
        exit 0
    else
        echo "------------------------------------------------"
        if [ $ATTEMPT -lt $MAX_ATTEMPTS ]; then
            echo "DETECTION: CRC/FAIL or Busy state detected."
            echo "Resetting JTAG state and retrying in 200ms..."
            $LOADER -b $BOARD --detect > /dev/null 2>&1
            sleep 0.2
        else
            echo "FATAL: Flash failed after $MAX_ATTEMPTS attempts."
            exit 1
        fi
    fi
    ATTEMPT=$((ATTEMPT + 1))
done
