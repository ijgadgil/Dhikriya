#!/usr/bin/env bash
# Stop the data plane started by ndn_up.sh and remove the namespace and link.
pkill -f "cogndn_ndn.producer" 2>/dev/null || true
sudo pkill -f "cogndn_ndn.producer" 2>/dev/null || true
pkill -f "ndnd fw run" 2>/dev/null || true
sudo pkill -f "ndnd fw run" 2>/dev/null || true
sudo ip link del veh0 2>/dev/null || true
sudo ip netns del rsu 2>/dev/null || true
rm -f /run/cogndn/*.sock 2>/dev/null || true
echo "NDN plane down"
