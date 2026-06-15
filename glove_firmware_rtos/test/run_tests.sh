#!/usr/bin/env sh
# Build & run the firmware host-side logic tests. Requires g++ with C++17.
# These run on a PC (no ESP32) by testing protocol_logic.h in isolation.
set -e
cd "$(dirname "$0")"
g++ -std=c++17 -Wall -Wextra -I.. test_protocol_logic.cpp -o test_logic
./test_logic
