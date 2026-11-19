#!/usr/bin/env bash
set -euo pipefail

readonly project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly container_project_dir="/workspace/DART"

docker_args=(
  --rm
  --interactive
  --name dart-compute
  --network host
  --ipc host
  --privileged
  --user "$(id -u):$(id -g)"
  --env HOME=/tmp/dart-home
  --volume "${project_dir}:${container_project_dir}"
  --workdir "${container_project_dir}"
)

if [[ -t 0 && -t 1 ]]; then
  docker_args+=(--tty)
fi

while read -r group_id; do
  docker_args+=(--group-add "${group_id}")
done < <(id -G | tr ' ' '\n')

exec docker run "${docker_args[@]}" sp_vpie:dart bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  if [[ ! -f install/setup.bash ]]; then
    colcon build --symlink-install --event-handlers console_cohesion+
  fi
  source install/setup.bash
  exec ros2 launch dart_bringup dart.launch.py \
    config:=/workspace/DART/config/standard3.yaml
'
