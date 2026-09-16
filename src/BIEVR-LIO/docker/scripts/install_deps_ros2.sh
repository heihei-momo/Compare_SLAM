#!/bin/bash
set -o pipefail

# Install system deps from apt
apt-get -qq update &&  apt-get install -y libgoogle-glog-dev git

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
