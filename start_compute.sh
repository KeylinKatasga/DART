#!/usr/bin/env bash
set -euo pipefail

readonly project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly container_project_dir="/workspace/DART"
entry_mode="compute"

case "${1:-}" in
  "") ;;
  --servo-test) entry_mode="servo-test" ;;
  --servo-test-dry-run) entry_mode="servo-test-dry-run" ;;
  *)
    echo "usage: $0 [--servo-test|--servo-test-dry-run]" >&2
    exit 2
    ;;
esac

docker_args=(
  --rm
  --interactive
  --name dart-compute
  --network host
  --ipc host
  --privileged
  --user "$(id -u):$(id -g)"
  --env HOME=/tmp/dart-home
  --env "DART_ENTRY_MODE=${entry_mode}"
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
  if [[ "${DART_ENTRY_MODE}" == "servo-test" ]]; then
    exec python3 tools/green_light_servo_sweep.py --arm
  fi
  if [[ "${DART_ENTRY_MODE}" == "servo-test-dry-run" ]]; then
    exec python3 tools/green_light_servo_sweep.py
  fi
  exec ros2 launch dart_bringup dart.launch.py \
    config:=/workspace/DART/config/standard3.yaml
'
