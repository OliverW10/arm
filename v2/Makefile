FLAGS := -g -std=c++17 -Wall -I/usr/local/include/eigen3

main: build/main.o build/kinematics.o build/vision.o build/arm.o
	g++ -o build/main $^ -Lpigpio/build -lpigpio -Llibrealsense/build -lrealsense2 -Lapriltag/build -lapriltag -pthread -lrt
	@echo "\033[0;32mBUILD SUCCESSFULL\\033[0m\n\n"

bringup: build/bringup.o build/kinematics.o build/arm.o
	g++ -o build/test $^ -Lpigpio/build -lpigpio -pthread -lrt
	@echo "\033[0;32mBUILD SUCCESSFULL\\033[0m\n\n"


build/bringup.o: src/bringup.cpp src/arm.hpp src/vision.hpp
	g++ -c ${FLAGS} -o $@ $< -Ipigpio

build/main.o: src/main.cpp src/arm.hpp src/vision.hpp
	g++ -c ${FLAGS} -o $@ $< -Ilibrealsense/include -Ipigpio -Iapriltag

build/kinematics.o: src/kinematics.cpp src/kinematics.hpp
	g++ -c ${FLAGS} -o $@ $<

build/vision.o: src/vision.cpp src/vision.hpp
	g++ -c ${FLAGS} -o $@ $< -Ilibrealsense/include -Iapriltag

build/arm.o: src/arm.cpp src/arm.hpp src/kinematics.hpp
	g++ -c ${FLAGS} -o $@ $< -Ipigpio
