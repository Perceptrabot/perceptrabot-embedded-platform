#!/bin/bash

# Defaults
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BOARD="arduino:avr:uno"

# Usage info
usage() {
    echo "Usage: $0 <path_to_ino> [-p port] [-b board]"
    echo ""
    echo "  path_to_ino   Path to the .ino sketch file (required)"
    echo "  -p port       Serial port (default: $DEFAULT_PORT)"
    echo "  -b board      Board FQBN   (default: $DEFAULT_BOARD)"
    exit 1
}

# Require at least one argument
if [ $# -lt 1 ]; then
    usage
fi

INO_PATH="$1"
shift

PORT="$DEFAULT_PORT"
BOARD="$DEFAULT_BOARD"

# Parse optional flags
while getopts "p:b:" opt; do
    case $opt in
        p) PORT="$OPTARG" ;;
        b) BOARD="$OPTARG" ;;
        *) usage ;;
    esac
done

# Validate the .ino file exists
if [ ! -f "$INO_PATH" ]; then
    echo "Error: File not found: $INO_PATH"
    exit 1
fi

echo ">>> Compiling $INO_PATH for board $BOARD ..."
arduino-cli compile -b "$BOARD" "$INO_PATH"

if [ $? -ne 0 ]; then
    echo "Compilation failed. Aborting upload."
    exit 1
fi

echo ">>> Uploading to $PORT ..."
arduino-cli upload "$INO_PATH" -p "$PORT" -b "$BOARD"

if [ $? -ne 0 ]; then
    echo "Upload failed."
    exit 1
fi

echo ">>> Done!"