#!/usr/bin/env bash
set -e

# Set Python version
PYTHON=python3

echo "=================================="
echo "Project Setup"
echo "=================================="
echo

# Check if the specified Python version is installed
if ! command -v "$PYTHON" &>/dev/null; then
    echo "ERROR: $PYTHON is not installed."
    echo "Please install it, e.g.: sudo apt install python3.12"
    echo
    exit 1
fi

echo "$PYTHON detected:"
"$PYTHON" --version
echo

# Loop through all subdirectories that contain a requirements.txt
dir="./"
if [ -f "$dir/requirements.txt" ]; then
    echo

    pushd "$dir" > /dev/null

    # Create virtual environment if it doesn't exist
    if [ ! -d ".venv" ]; then
        echo "Creating virtual environment with $PYTHON"
        "$PYTHON" -m venv ".venv"
    fi

    # Upgrade pip
    ".venv/bin/python" -m pip install --upgrade pip

    # Install dependencies
    ".venv/bin/python" -m pip install -r "requirements.txt"

    popd > /dev/null

    echo "Finished $dir"
fi

echo
echo "=================================="
echo "Setup complete"
echo "=================================="
