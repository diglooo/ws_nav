Compile the sketch:

cd arduino-cli

./arduino-cli compile --fqbn Intel:arc32:arduino_101 /absolute/path/to/sketch/folder
./arduino-cli upload -p /dev/ttyACM0 --fqbn Intel:arc32:arduino_101 /absolute/path/to/sketch/folder