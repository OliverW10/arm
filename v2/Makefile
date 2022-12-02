# TODO: fix this entire thing
FLAGS := -g -std=c++17 -I/usr/local/include/eigen3 #-Wall

all: main bringup test

main: build/main.o build/kinematics.o build/vision.o build/arm.o
	g++ -o build/main $^ -Lpigpio/build -lpigpio -Llibrealsense/build -lrealsense2 -Lapriltag/build -lapriltag -pthread -lrt
	@echo "\033[0;32mBUILT MAIN SUCCESSFULLY\\033[0m\n\n"

bringup: build/bringup.o build/kinematics.o build/arm.o
	g++ -o build/brinup $^ -Lpigpio/build -lpigpio -pthread -lrt
	@echo "\033[0;32mBUILT BRINGUP SUCCESSFULLY\\033[0m\n\n"

test: tests/*.cpp main
	mkdir -p build/tests
	@echo "building test: " $<
	g++ -c $< -o build/$<.o ${FLAGS} -Isrc -Llibrealsense/build -Ilibrealsense/include -Iapriltag
	g++ -o build/$<_test build/$<.o build/vision.o build/arm.o build/kinematics.o -Llibrealsense/build -lrealsense2 -Lapriltag/build -lapriltag -pthread -lrt
	@echo "\033[0;32mBUILT TEST SUCCESSFULLY\\033[0m\n\n"

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
