sudo apt install python3-scipy
sudo apt-get install ros-noetic-aruco-detect
sudo apt install ros-noetic-xacro
sudo apt install ros-noetic-rosserial

Export ROSDISTRO=noetic

sudo apt-get install -y \
ros-$ROSDISTRO-roslint \
ros-$ROSDISTRO-rosserial \
ros-$ROSDISTRO-rosserial-arduino \
ros-$ROSDISTRO-imu-filter-madgwick \
ros-$ROSDISTRO-gmapping \
ros-$ROSDISTRO-map-server \
ros-$ROSDISTRO-navigation \
ros-$ROSDISTRO-robot-localization \
ros-$ROSDISTRO-teleop-twist-keyboard \
ros-$ROSDISTRO-tf2 \
ros-$ROSDISTRO-tf2-ros 

sudo apt-get install ros-noetic-robot-state-publisher
sudo apt-get install ros-noetic-joint-state-publisher
sudo apt-get install ros-noetic-compressed-image-transport
sudo apt-get install ros-noetic-camera-info-manager
