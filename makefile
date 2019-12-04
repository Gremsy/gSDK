all:  mavlink_control

mavlink_control: mavlink_control.cpp
	g++ -I mavlink/include/mavlink/v2.0 mavlink_control.cpp serial_port.cpp gimbal_interface.cpp -o gSDK -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o mavlink_control
