FLAGS := -g -std=c++17 -O -pg

build: main kinematics vision arm
	g++ -o build/main build/main.o build/kinematics.o build/vision.o build/arm.o -Lpigpio/build -lpigpio -lrealsense2 -Lapriltag/build -lapriltag -pg -pthread
	@echo "\033[0;32mBUILD SUCCESSFULL\\033[0m\n\n"

main: src/main.cpp
	g++ -c ${FLAGS} -o build/main.o src/main.cpp -I/usr/include/eigen3 -I/usr/local/include -Iapriltag

kinematics: src/kinematics.cpp src/kinematics.hpp
	g++ -c ${FLAGS} -o build/kinematics.o src/kinematics.cpp -I/usr/include/eigen3

vision: src/vision.cpp src/vision.hpp
	g++ -c ${FLAGS} -o build/vision.o src/vision.cpp -I/usr/include/eigen3 -I/usr/local/include -Iapriltag

arm: src/arm.cpp src/arm.hpp
	g++ -c ${FLAGS} -o build/arm.o src/arm.cpp -I/usr/include/eigen3 -Ipigpio/build 
