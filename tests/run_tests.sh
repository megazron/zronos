#!/usr/bin/env bash
# ChalaoOS test suite. Exits nonzero on any failure.
set -u
cd "$(dirname "$0")/.."
make >/dev/null 2>&1 || { echo "BUILD FAILED"; exit 1; }
fail=0
ck(){ if echo "$OUT" | grep -q -- "$1"; then echo "  PASS  $2"; else echo "  FAIL  $2 (missing: $1)"; fail=1; fi; }

echo "== boot / lifecycle / IPC / e-stop / halt =="
OUT="$(./chalaoos --demo --seconds 8 2>&1)"
ck "boot complete -- entering scheduler" "boots and enters scheduler"
ck "'bringup' is RUNNING"   "run-once service starts"
ck "'heartbeat' is RUNNING" "periodic service starts"
ck "'watchdog' is RUNNING"  "dependency-ordered start"
ck "\[hal\] wheels drove"   "HAL actuator driven by a service"
ck "battery        76"      "blackboard: battery topic published"
ck "detection      cone"    "blackboard: detection topic published"
ck "E-STOP asserted by service 'watchdog'" "service trips the e-stop"
ck "\[hal\] BLOCKED -- actuators are in e-stop" "HAL respects e-stop after trip"
ck "ChalaoOS band. Alvida." "clean halt"

echo "== supervisor restart policy =="
OUT="$(./chalaoos --demo --seconds 3 tests/restart.manifest 2>&1)"
ck "FAILED" "a crashing service is marked FAILED"
ck "restart policy 'always'" "restart policy kicks in"
ck "restarting 'crasher'" "supervisor restarts the service"

echo "== boot order (bringup before its dependents) =="
OUT="$(./chalaoos --demo --seconds 1 2>&1)"
BR=$(echo "$OUT" | grep -n "starting 'bringup'" | head -1 | cut -d: -f1)
HB=$(echo "$OUT" | grep -n "starting 'heartbeat'" | head -1 | cut -d: -f1)
if [ -n "$BR" ] && [ -n "$HB" ] && [ "$BR" -lt "$HB" ]; then echo "  PASS  bringup starts before heartbeat (dependency)"; else echo "  FAIL  boot order"; fail=1; fi

[ $fail -eq 0 ] && echo "ALL TESTS PASSED" || echo "SOME TESTS FAILED"
exit $fail
