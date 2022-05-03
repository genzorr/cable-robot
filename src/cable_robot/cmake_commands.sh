#CMAKE_PREFIX_PATH=/opt/ros/rolling /usr/bin/cmake /home/genzorr/dev/cable-robot/src/cable_robot -DCMAKE_INSTALL_PREFIX=/home/genzorr/dev/cable-robot/install/cable_robot
CMAKE_PREFIX_PATH=/opt/ros/rolling /usr/bin/cmake --build /home/genzorr/dev/cable-robot/build/cable_robot -- -j8 -l8
CMAKE_PREFIX_PATH=/opt/ros/rolling /usr/bin/cmake --install /home/genzorr/dev/cable-robot/build/cable_robot
