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

# Check if requirements.txt exists in the current directory
if [ -f "requirements.txt" ]; then
    echo

    # Create virtual environment if it doesn't exist
    if [ ! -d ".venv" ]; then
        echo "Creating virtual environment with $PYTHON"
        "$PYTHON" -m venv ".venv"
    fi

    # Upgrade pip
    ".venv/bin/python" -m pip install --upgrade pip

    # Install dependencies
    ".venv/bin/python" -m pip install -r "requirements.txt"

    echo "Finished setting up virtual environment"
fi

echo
echo "=================================="
echo "Setup complete"
echo "=================================="
