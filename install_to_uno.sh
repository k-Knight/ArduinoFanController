screen -X quit
arduino-cli compile --fqbn arduino:avr:uno --libraries="/home/odyssey/misc/arduino/libraries" fan_controller_uno
arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM1 fan_controller_uno
