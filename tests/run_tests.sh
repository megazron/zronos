#!/usr/bin/env bash
# zronOS test suite. Exits nonzero on any failure.
set -u
cd "$(dirname "$0")/.."
make >/dev/null 2>&1 || { echo "BUILD FAILED"; exit 1; }
fail=0
ck(){ if echo "$OUT" | grep -q -- "$1"; then echo "  PASS  $2"; else echo "  FAIL  $2 (missing: $1)"; fail=1; fi; }
nck(){ if echo "$OUT" | grep -q -- "$1"; then echo "  FAIL  $2 (unexpected: $1)"; fail=1; else echo "  PASS  $2"; fi; }

echo "== boot / lifecycle / IPC / e-stop / halt =="
OUT="$(./zron demo --seconds 8 2>&1)"
ck "boot complete -- entering scheduler" "boots and enters scheduler"
ck "'bringup' is RUNNING"   "run-once service starts"
ck "'heartbeat' is RUNNING" "periodic service starts"
ck "'watchdog' is RUNNING"  "dependency-ordered start"
ck "\[hal\] wheels drove"   "HAL actuator driven by a service"
ck "battery        76"      "blackboard: battery topic published"
ck "detection      cone"    "blackboard: detection topic published"
ck "E-STOP asserted by service 'watchdog'" "service trips the e-stop"
ck "\[hal\] BLOCKED -- actuators are in e-stop" "HAL respects e-stop after trip"
ck "zronOS band. Alvida." "clean halt"
ck "summary: UNHEALTHY" "an e-stopped HAL makes the health summary UNHEALTHY"

echo "== supervisor restart policy =="
OUT="$(./zron demo --seconds 3 tests/restart.manifest 2>&1)"
ck "FAILED" "a crashing service is marked FAILED"
ck "restart policy 'always'" "restart policy kicks in"
ck "restarting 'crasher'" "supervisor restarts the service"

echo "== boot order (bringup before its dependents) =="
OUT="$(./zron demo --seconds 1 2>&1)"
BR=$(echo "$OUT" | grep -n "starting 'bringup'" | head -1 | cut -d: -f1)
HB=$(echo "$OUT" | grep -n "starting 'heartbeat'" | head -1 | cut -d: -f1)
if [ -n "$BR" ] && [ -n "$HB" ] && [ "$BR" -lt "$HB" ]; then echo "  PASS  bringup starts before heartbeat (dependency)"; else echo "  FAIL  boot order"; fail=1; fi

echo "== parameter server (shared across services) =="
OUT="$(./zron demo --seconds 0.5 examples/capabilities.manifest 2>&1)"
ck "\[param\] max_speed = 0.5" "a service sets a parameter"
ck "max_speed = 0.5  robot = atlas" "another service reads the same parameters"

echo "== TF transform tree (correct 3D composition) =="
ck "arm frame odom mein: Pose(x=1.000, y=0.000, z=0.500" "TF lookup composes the chain (translation)"
ck "yaw=1.571" "TF lookup composes rotation (yaw)"

echo "== inter-service RPC =="
ck "advertised service 'plan_speed'" "a service advertises an RPC endpoint"
ck "plan_speed(0.4) ne diya: 40" "another service calls it and gets the result"

echo "== health / diagnostics =="
ck "\[sehat\] OK" "a service reports its health"
ck "summary: HEALTHY\|summary: DEGRADED\|summary: UNHEALTHY" "aggregate health report renders"

echo "== zron doctor (preflight, no boot) =="
OUT="$(./zron doctor system/system.manifest 2>&1)"
ck "READY TO BOOT" "a valid manifest passes preflight"
ck "dependency graph is acyclic" "cycle check runs"
OUT="$(./zron doctor tests/bad.manifest 2>&1)"
ck "NOT READY" "an invalid manifest fails preflight"
ck "script missing\|parse error\|unknown" "preflight explains the failure"

echo "== zron new (scaffold) =="
ZRON="$(pwd)/zron"
SCAF="$(mktemp -d)/newbot"
( cd "$(dirname "$SCAF")" && "$ZRON" new newbot >/dev/null 2>&1 )
if [ -f "$SCAF/robot.manifest" ] && [ -f "$SCAF/bringup.rc" ]; then echo "  PASS  scaffold writes a manifest + services"; else echo "  FAIL  scaffold"; fail=1; fi
OUT="$("$ZRON" doctor "$SCAF/robot.manifest" 2>&1)"
ck "READY TO BOOT" "the scaffolded robot passes its own preflight"
rm -rf "$(dirname "$SCAF")"

echo "== zron mujoco (live HITL, skipped unless a MuJoCo bridge is set) =="
if [ -n "${ZRON_MJ:-}" ] && [ -n "${ZRON_MODEL:-}" ]; then
  OUT="$(./zron mujoco "$ZRON_MODEL" --bridge "$ZRON_MJ" --cycles 50 2>&1)"
  ck '"ok":true' "zron drives a real MuJoCo model through the bridge"
  ck '"stable":true' "physics stays stable under zron's control loop"
else
  echo "  SKIP  set ZRON_MJ=\"python3 bridge/zron_mujoco.py\" and ZRON_MODEL=<scene.xml> to run"
fi

echo "== scheduler resolution follows the fastest service =="
OUT="$(./zron demo --seconds 2 tests/fast.manifest 2>&1)"
RUNS=$(echo "$OUT" | awk '$1=="fast" && $2=="RUNNING" {print $6}')
if [ -n "$RUNS" ] && [ "$RUNS" -ge 19 ] && [ "$RUNS" -le 21 ]; then echo "  PASS  10 Hz service runs ~20 times in 2 s ($RUNS)"; else echo "  FAIL  10 Hz service ran $RUNS times in 2 s"; fail=1; fi

echo "== zron up (real time, headless) =="
T0=$(date +%s.%N)
OUT="$(timeout 10 ./zron up system/system.manifest --seconds 2 2>&1)"; RC=$?
T1=$(date +%s.%N)
ck "boot complete -- entering scheduler" "up boots"
ck "zronOS band. Alvida." "up halts after --seconds"
if awk -v a="$T0" -v b="$T1" 'BEGIN{d=b-a; exit !(d>=1.8 && d<4)}'; then echo "  PASS  up runs on the wall clock (~2 s)"; else echo "  FAIL  up wall time"; fail=1; fi
[ "$RC" -eq 0 ] && echo "  PASS  up exits 0" || { echo "  FAIL  up exit code $RC"; fail=1; }
LOG="$(mktemp)"
./zron up system/system.manifest > "$LOG" 2>&1 & PID=$!
sleep 1.5; kill -TERM "$PID"; wait "$PID"; RC=$?
OUT="$(cat "$LOG")"; rm -f "$LOG"
ck "signal received" "SIGTERM is caught"
ck "summary: HEALTHY\|summary: DEGRADED\|summary: UNHEALTHY" "SIGTERM prints a final health report"
ck "zronOS band. Alvida." "SIGTERM halts cleanly"
[ "$RC" -eq 0 ] && echo "  PASS  SIGTERM exit 0" || { echo "  FAIL  SIGTERM exit code $RC"; fail=1; }

echo "== shell =="
OUT="$(printf 'ps\nquit\n' | ./zron run system/system.manifest 2>&1)"
ck "zronOS band. Alvida." "quit halts the shell"
OUT="$(printf 'ps\n' | ./zron run system/system.manifest 2>&1)"
ck "zronOS band. Alvida." "end of input halts the shell"
OUT="$(./zron frobnicate 2>&1)"; RC=$?
ck "anjaan command" "unknown subcommand is reported"

echo "== version =="
OUT="$(./zron --version 2>&1)"
ck "zronOS 1.1.1" "version string"

[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $fail
