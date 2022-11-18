#! /bin/sh

sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y && 
rm -rf /var/lib/apt/lists/*
