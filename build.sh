#!/bin/bash
if [ ! -d "build" ]; then
	mkdir build
	cd build
	cmake ..
	cd ..
fi
cd build
make -j16&&[[ -d "$PICO_DRIVE_PATH" ]]&&cp ultrasonic_robot.uf2 "$PICO_DRIVE_PATH/ultrasonic_robot.uf2"
cd ..
