#!/usr/bin/env bash
# Issue #1: Gazebo Harmonic from the OSRF apt repo, headless use only.
set -eo pipefail
LOG=~/deletdis/log/05_gz_install.log; mkdir -p "$(dirname "$LOG")"; exec > >(tee "$LOG") 2>&1
echo "=== $(date -Is) start"
sudo apt-get install -y -q curl lsb-release gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update -q
apt-cache policy gz-harmonic | head -3
sudo apt-get install -y -q gz-harmonic
echo "--- proof: gz sim headless 100 iterations"
gz sim --version
gz sim -s -r --iterations 100 empty.sdf; echo "exit=$?"
echo "=== $(date -Is) done"
