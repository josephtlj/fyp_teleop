#!/usr/bin/env bash
# Run on the Mac. Copies the project and ~/fyp_data from the Pi into a local folder.
# Files deleted on the Pi are kept in the backup on purpose.
set -euo pipefail
DEST="${1:-$HOME/fyp_backup}"
mkdir -p "$DEST"
rsync -az yahboom:colcon_ws/src/fyp_teleop/ "$DEST/fyp_teleop/"
rsync -az yahboom:fyp_data/ "$DEST/fyp_data/"
echo "Backed up to $DEST"
