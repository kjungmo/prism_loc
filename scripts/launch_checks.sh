#!/usr/bin/env bash
# Headless launch checks: start each shipped launch file without RViz on generated
# fixture maps, then verify that every key of its params YAML is declared and applied
# by the running node and that the node publishes its /diagnostics status.
# Run from a sourced colcon workspace:  bash scripts/launch_checks.sh [laser2d|ndt3d|fusion3d ...]
set -u
here="$(cd "$(dirname "$0")" && pwd)"
fix="${FIXTURES:-$(mktemp -d)}"
python3 "$here/make_fixtures.py" "$fix"
share_loc="$(ros2 pkg prefix prism_loc)/share/prism_loc"
share_fus="$(ros2 pkg prefix prism_loc_fusion_ros)/share/prism_loc_fusion_ros"
backends=("$@")
[ ${#backends[@]} -eq 0 ] && backends=(laser2d ndt3d fusion3d)

stop() {  # SIGTERM the launch process group, SIGKILL if it lingers
  kill -TERM -"$1" 2>/dev/null
  for _ in $(seq 50); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.2; done
  kill -KILL -"$1" 2>/dev/null
}

failed=0
for b in "${backends[@]}"; do
  case "$b" in
    laser2d)  cmd=(ros2 launch prism_loc laser2d.launch.py rviz:=false "map:=$fix/map.yaml")
              args=("$share_loc/params/laser2d.yaml" --node prism_loc --diagnostics "prism_loc: localization") ;;
    ndt3d)    cmd=(ros2 launch prism_loc ndt3d.launch.py rviz:=false "map_pcd_path:=$fix/map.pcd")
              args=("$share_loc/params/ndt3d.yaml" --node prism_loc --allow-override map_pcd_path
                    --diagnostics "prism_loc: localization") ;;
    fusion3d) cmd=(ros2 launch prism_loc_fusion_ros fusion3d.launch.py rviz:=false "map_pcd_path:=$fix/map.pcd")
              args=("$share_fus/params/fusion3d.yaml" --node prism_loc_fusion --allow-override map_pcd_path
                    --diagnostics "prism_loc_fusion: fusion") ;;
    *) echo "unknown backend $b"; exit 2 ;;
  esac
  echo "=== $b: ${cmd[*]}"
  setsid "${cmd[@]}" > "launch_$b.log" 2>&1 &
  pid=$!
  status=0
  timeout 90 python3 "$here/check_param_binding.py" "${args[@]}" || status=$?
  stop "$pid"
  if [ "$status" -ne 0 ]; then echo "--- launch_$b.log"; cat "launch_$b.log"; failed=1; fi
done
exit $failed
