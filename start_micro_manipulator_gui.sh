#!/bin/bash

# Activate the virtual environment
source ./.venv/bin/activate

# Run the Python script
cd source
python3 main.py &
