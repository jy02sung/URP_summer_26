#!/usr/bin/env bash

set -euo pipefail

# Keep the wrist links in Gazebo while removing only their ros_control
# transmissions.  This is used to isolate hardware-interface initialization.
xacro --inorder "$(dirname "$0")/../urdf/dual_arm.xacro" |
  awk '
    /<!-- Effort Control -->/ { effort_control = 1 }
    effort_control && (/<transmission name="tran6">/ || /<transmission name="tran11">/) { skip = 1 }
    !skip { print }
    skip && /<\/transmission>/ { skip = 0 }
  '
