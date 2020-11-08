screen -X quit
arduino-cli compile --fqbn Seeeduino:samd:zero --libraries="/home/odyssey/misc/arduino/libraries" fan_controller_zero
arduino-cli upload --fqbn Seeeduino:samd:zero -p /dev/ttyACM0 fan_controller_zero
