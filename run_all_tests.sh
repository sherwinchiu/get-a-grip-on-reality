#!/usr/bin/env sh
# =============================================================================
#  run_all_tests.sh  --  one command to run every test in the project.
#
#    ./run_all_tests.sh
#
#  Runs:
#    1. Firmware logic tests   (host C++ via g++, no ESP32 needed)   -- 52 checks
#    2. Web-demo protocol tests (Node built-in test runner)          -- 22 checks
#    3. (optional) Firmware compile gate, if arduino-cli is on PATH
#  Exits non-zero if anything fails, so it doubles as a CI entry point.
# =============================================================================
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
fail=0

echo "──────────────────────────────────────────────"
echo " 1/3  Firmware logic tests (g++)"
echo "──────────────────────────────────────────────"
if command -v g++ >/dev/null 2>&1; then
  ( cd "$ROOT/glove_firmware_rtos/test" && sh run_tests.sh ) || fail=1
  rm -f "$ROOT/glove_firmware_rtos/test/test_logic" "$ROOT/glove_firmware_rtos/test/test_logic.exe" 2>/dev/null || true
else
  echo "  SKIP: g++ not found"
fi

echo
echo "──────────────────────────────────────────────"
echo " 2/3  Web-demo protocol tests (node --test)"
echo "──────────────────────────────────────────────"
if command -v node >/dev/null 2>&1; then
  ( cd "$ROOT/mobile-demo" && node --test ) || fail=1
else
  echo "  SKIP: node not found"
fi

echo
echo "──────────────────────────────────────────────"
echo " 3/3  Firmware compile gate (arduino-cli, optional)"
echo "──────────────────────────────────────────────"
if command -v arduino-cli >/dev/null 2>&1; then
  arduino-cli compile --fqbn esp32:esp32:esp32s3 "$ROOT/glove_firmware_rtos" || fail=1
else
  echo "  SKIP: arduino-cli not on PATH (the Arduino IDE bundles its own copy)."
  echo "        Build in the IDE, or point this at that binary to enable the gate."
fi

echo
if [ "$fail" -eq 0 ]; then echo "ALL TEST SUITES PASSED ✓"; else echo "SOME TESTS FAILED ✗"; fi
exit $fail
