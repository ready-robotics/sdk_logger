
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
The multi_process directory will create two executables. The log_process executable can be 
executed with no command line options do start a server that will receive string to log
from the user processes. The user processes only send strings to the server when provided
the -s option. 
The user_process executable creates a user process that logs message to console, server or
file as requested by the command line options. See the command line help for information
on the options available. Any number of user_processes can exist, all processes create
two child threads and log messages at hard coded rates. 
