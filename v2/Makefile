FLAGS := -g -std=c++17 -O3 -pg -g -Wall

main: build/main.o build/kinematics.o build/vision.o build/arm.o
	g++ -o build/main build/main.o build/kinematics.o build/vision.o build/arm.o -Lpigpio/build -lpigpio -Llibrealsense/build -lrealsense2 -Lapriltag/build -lapriltag -pg -pthread
	@echo "\033[0;32mBUILD SUCCESSFULL\\033[0m\n\n"

bringup: build/bringup.o build/kinematics.o build/arm.o
	g++ -o build/test build/bringup.o build/kinematics.o build/arm.o -Lpigpio/build -lpigpio -pg -pthread
	@echo "\033[0;32mBUILD SUCCESSFULL\\033[0m\n\n"

build/bringup.o: src/bringup.cpp src/arm.hpp src/vision.hpp
	g++ -c ${FLAGS} -o $@ src/bringup.cpp -I/usr/include/eigen3 -I/usr/local/include

build/main.o: src/main.cpp src/arm.hpp src/vision.hpp
	g++ -c ${FLAGS} -o $@ src/main.cpp -I/usr/include/eigen3 -I/usr/local/include -Iapriltag

build/kinematics.o: src/kinematics.cpp src/kinematics.hpp
	g++ -c ${FLAGS} -o $@ src/kinematics.cpp -I/usr/include/eigen3

build/vision.o: src/vision.cpp src/vision.hpp
	g++ -c ${FLAGS} -o $@ src/vision.cpp -I/usr/include/eigen3 -Ilibrealsense/include -Iapriltag

build/arm.o: src/arm.cpp src/arm.hpp src/kinematics.hpp
	g++ -c ${FLAGS} -o $@ src/arm.cpp -I/usr/include/eigen3 -Ipigpio
