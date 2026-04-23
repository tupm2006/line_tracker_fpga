#!/bin/bash

# ==============================================================================
# Tang Nano 9K Pro Flash Script
# ==============================================================================
# Usage: 
#   ./flash_fpga.sh          -> Fast SRAM load (lost on power-off)
#   ./flash_fpga.sh --flash  -> Permanent Flash load (slow erase/verify)
# ==============================================================================

BITSTREAM="line_tracker.fs"
BOARD="tangnano9k"
FREQ="2500000"
LOADER="openFPGALoader"

# 1. Configuration
FLASH_MODE=false
if [[ "$1" == "--flash" ]]; then
    FLASH_MODE=true
    echo "--- PERMANENT FLASH MODE ENABLED ---"
else
    echo "--- FAST SRAM MODE (Default) ---"
fi

# 2. Check for Bitstream
if [ ! -f "$BITSTREAM" ]; then
    echo "ERROR: Bitstream '$BITSTREAM' not found."
    echo "Run 'make' or check your file names."
    exit 1
fi

# 3. Programming Logic
flash_once() {
    if [ "$FLASH_MODE" = true ]; then
        # Flash mode (Slow, Permanent)
        $LOADER -b $BOARD --freq $FREQ -f "$BITSTREAM"
    else
        # SRAM mode (Fast, Volatile)
        $LOADER -b $BOARD --freq $FREQ "$BITSTREAM"
    fi
}

# 4. Attempt Programming with Retry
MAX_RETRIES=2
COUNT=0
SUCCESS=false

while [ $COUNT -le $MAX_RETRIES ]; do
    echo "Attempting to load bitstream (Try $((COUNT+1)))..."
    
    # Execute and capture output
    flash_once
    if [ $? -eq 0 ]; then
        SUCCESS=true
        break
    fi
    
    echo "WARNING: Load failed. Resetting cable and retrying..."
    $LOADER -b $BOARD --detect > /dev/null 2>&1
    sleep 0.5
    ((COUNT++))
done

# 5. Final Status
if [ "$SUCCESS" = true ]; then
    echo "----------------------------------------"
    echo "SUCCESS: FPGA Programmed Successfully."
    echo "----------------------------------------"
    # Optional Reset
    $LOADER -b $BOARD --reset > /dev/null 2>&1
    exit 0
else
    echo "----------------------------------------"
    echo "FATAL ERROR: Failed to program FPGA after $MAX_RETRIES retries."
    echo "Check USB connection and Tang Nano 9K lights."
    echo "----------------------------------------"
    exit 1
fi
