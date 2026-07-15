#!/usr/bin/env bash

set -euo pipefail

spawn_delay="${1:-8.0}"
model_name="${2:-dual_arm}"
urdf_file="${3:-}"

if [[ -z "${urdf_file}" ]]; then
  echo "usage: $0 <spawn_delay> <model_name> <urdf_file>" >&2
  exit 2
fi

sleep "${spawn_delay}"

source /home/jys/catkin_ws/devel/setup.bash

until rosservice info /gazebo/spawn_urdf_model >/dev/null 2>&1; do
  sleep 0.5
done

exec rosrun gazebo_ros spawn_model -urdf -model "${model_name}" -file "${urdf_file}"
