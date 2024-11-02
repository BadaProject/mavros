#! /usr/bin/env bash

sudo apt-get install python3-rosdep
pip install rosdep 
sudo rosdep init
rosdep update

sudo apt-get install geographiclib-tools
sudo apt-get install libgeographic-dev

cd ~/mavros_ws/src/mavros
sudo ./mavros/scripts/install_geographiclib_datasets.sh

cd ~/bada2_ws
rosdep install --from-paths src -y --ignore-src


# sudo apt install ros-humble-geographic-msgs -y
# sudo apt install libboost-all-dev -y
# sudo apt-get install libasio-dev -y
# sudo apt install python3-pip -y
# sudo apt install libeigen3-dev -y
