#! /usr/bin/env bash

sudo apt-get install geographiclib-tools
sudo apt-get install libgeographic-dev
cd ~/mavros_ws/src/mavros
sudo ./mavros/scripts/install_geographiclib_datasets.sh
sudo apt install ros-humble-geographic-msgs -y
