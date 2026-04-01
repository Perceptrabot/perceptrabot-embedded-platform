# Perceptrabot Embedded Platform

This repository containg the code and relevant information to perceptrabots embedded platform.

Current platform: Arduino UNO


Current script in usage: `control_motors_read_encoders.ino`
the script probides control capabilities as well as wheel encoder values


additional usefult scripts:

`test_wiring.ino`

The script allows for efficient wiring evaluation:
1) laod it to arduino 
2) inspect the printed state of the robot (ex: turtning left) and compare it with the robots movement

If the wiring and pin definitions are correct the described values should match the robots acrions.



# Arduino CLI

documentation [link](https://arduino.github.io/arduino-cli/1.4/commands/arduino-cli/)

installation:
```bash
mkdir -p ~/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/bin sh

echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

arduino-cli version
```
after the installation: 
```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli board list
```

check available connected boads:
```
arduino-cli borad list
```

compilation:
```
arduino-cli compile -b arduino:avr:uno /home/user/Arduino/MySketch
```
upload:
```
arduino-cli upload /home/user/Arduino/MySketch -p /dev/ttyACM0 -b arduino:avr:uno
```
