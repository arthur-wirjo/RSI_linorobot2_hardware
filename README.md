## linorobot2_hardware

The repository to configure and flash firmware for the yahboom microros expansion board which runs ESP32-S3
Its kinda all over the place due to the incompatibility between the projects hardware and the hardware supported by the original linorobot2_hardware
The incompatibility includes: 
*  IMU: ICM42670P
*  Motor: M310


## Installation

### 1. Download linorobot2_hardware

    cd $HOME
    git clone https://github.com/linorobot/linorobot2_hardware

### 2. Install PlatformIO
Download and install platformio. [Platformio](https://platformio.org/) allows you to develop, configure, and upload the firmware without the Arduino IDE. This means that you can upload the firmware remotely which is ideal on headless setup especially when all components have already been fixed. 
    
    cd $HOME
    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py
    
Add platformio to your $PATH:

    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
    source $HOME/.bashrc


## Building the robot

### 1. Robot orientation
Robot Orientation:

-------------FRONT-------------

WHEEL1 WHEEL2 (2WD)

WHEEL3 WHEEL4 (4WD)

--------------BACK--------------

