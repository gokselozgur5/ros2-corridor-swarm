# Process hygiene for the proof scripts. Source right after env.sh, before anything is launched:
#   source ./env.sh; source ./steps/lib.sh
# Thor is shared: other users run Gazebo, robot_state_publisher, parameter_bridge and foxglove_bridge
# on the same host, so nothing here is killed by executable name. Everything a proof starts inherits
# CORRIDOR_PROOF=<pid of the proof script>, and kill_ours stops the live descendants of this script plus
# the leftovers of proofs whose script is gone (a run that died without reaching its EXIT trap). A proof
# that is still running keeps its processes: two proofs may share the host on different domains.
export CORRIDOR_PROOF="${CORRIDOR_PROOF:-$$}"

_proof_descendants() {   # the given pids and everything below them
  local pid kids
  for pid in "$@"; do
    echo "$pid"
    kids=$(pgrep -P "$pid")
    [ -n "$kids" ] && _proof_descendants $kids
  done
}

_proof_marked() {   # this user's processes carrying our marker, or the marker of a proof script that no longer runs
  local pid mark
  for pid in $(pgrep -u "$(id -u)"); do
    mark=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^CORRIDOR_PROOF=//p' | head -1)
    [ -z "$mark" ] && continue
    if [ "$mark" = "$CORRIDOR_PROOF" ] || ! kill -0 "$mark" 2>/dev/null; then echo "$pid"; fi
  done
}

_proof_targets() {   # both sets, minus this shell and its subshells, the log tee and ssh/tmux plumbing
  local pid comm
  for pid in $( { _proof_descendants $(pgrep -P $$); _proof_marked; } | sort -un ); do
    [ "$pid" = "$$" ] && continue
    comm=$(cat "/proc/$pid/comm" 2>/dev/null) || continue
    case "$comm" in bash|sh|dash|zsh|tee|sshd|ssh|tmux*) continue;; esac
    echo "$pid"
  done
}

_proof_alive() {   # the given pids that still run; zombies count as gone
  local pid state
  for pid in "$@"; do
    state=$(sed -E 's/.*\) //' "/proc/$pid/stat" 2>/dev/null | cut -d' ' -f1)
    [ -n "$state" ] && [ "$state" != "Z" ] && echo "$pid"
  done
}

kill_ours() {   # TERM everything that is ours, give it three seconds, KILL what is left
  local pids alive i
  pids=$(_proof_targets)
  [ -z "$pids" ] && return 0
  kill -TERM $pids 2>/dev/null
  for i in 1 2 3 4 5 6 7 8 9 10; do
    alive=$(_proof_alive $pids)
    [ -z "$alive" ] && return 0
    sleep 0.3
  done
  kill -KILL $alive 2>/dev/null
}
