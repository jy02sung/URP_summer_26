#!/usr/bin/env bash

set -euo pipefail

# Diagnostic model: preserve wrist mass and EE placement, but turn the two
# wrist hinges into fixed joints.  It separates hinge dynamics from geometry.
"$(dirname "$0")/generate_no_wrist_transmissions_urdf.sh" |
  sed \
    -e 's/<joint name="L_wrist_yaw_joint" type="revolute">/<joint name="L_wrist_yaw_joint" type="fixed">/' \
    -e 's/<joint name="R_wrist_yaw_joint" type="revolute">/<joint name="R_wrist_yaw_joint" type="fixed">/'
