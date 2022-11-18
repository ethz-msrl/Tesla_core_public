#! /bin/sh

apt-get update 
apt-get install --no-install-recommends -y swig libyaml-cpp-dev libeigen3-dev git autotools-dev autoconf automake libtool
rm -rf /var/lib/apt/lists/*
