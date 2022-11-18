#! /bin/bash

apt-get update && apt-get install --no-install-recommends -y python-catkin-tools python-rosdep python-wstool
rm -rf /var/lib/apt/lists/*
