
Perform the following actions to build the sdk_logger_user directory:
```
git submodule update --init --recursive
cd spdlog
mkdir build
cd build
cmake ..
make -j
sudo make install

catkin build sdk_logger_user
source ~/Dev/ready_ws/devel/setup.bash
rosrun sdk_logger_user sdk_logger_user_node
```


Perform the following actions to build the multi_process directory:
```
git submodule update --init --recursive
cd multi_process
mkdir build
cd build
cmake ..
make
```
