#!/usr/bin/env bash

# This script runs two steps:
# 1. Executes generate_reachability.py with Python 3
# 2. Launches Blender 4.3 in background mode with reduce.py
#
# Usage:
#   ./run_pipeline.sh /path/to/blender [args...]
#
# Arguments:
#   /path/to/blender   Path to the Blender executable (first argument, required)
#   [args...]          Additional arguments passed to generate_reachability.py
#
# Example:
#   ./run_pipeline.sh /media/data/blender-2.82-linux64/blender input1 input2
#
# This will run:
#   python3 generate_reachability.py input1 input2
#   /path/to/blender --background --python reduce.py

#/media/stonneau/data/dev/linux/soft/blender-4.3.2-linux-x64/blender

# Check if at least one argument was provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 /path/to/blender [arguments for generate_reachability.py]"
    exit 1
fi

# First argument = Blender executable
BLENDER_EXEC="$1"
shift  # Remove the first argument, leaving the rest for generate_reachability.py

# Step 1: Run generate_reachability.py with Python 3
python3 generate_reachability.py "$@"

# Step 2: Run Blender in background mode with reduce.py
"$BLENDER_EXEC" --background --python reduce_blender4.py

rm ./*.obj
mv output/*.obj ../data/reachability_constraints/
rm -r ./output
