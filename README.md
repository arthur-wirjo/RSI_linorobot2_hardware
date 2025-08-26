# linorobot2_hardware

The repository to configure and flash firmware for the yahboom microros expansion board which runs ESP32-S3
Its kinda all over the place due to the incompatibility between the projects hardware and the hardware supported by the original linorobot2_hardware
The incompatibility includes: 
*  IMU: ICM42670P
*  Motor: M310


# Installation

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

### 3. Install Dependencies
Go to firmware and calibration directory and run the command "pio run" to download the dependencies required

    cd ~/RSI_linorobot2_hardware/calibration
    pio run

    cd ~/RSI_linorobot2_hardware/firmware
    pio run

### 4. Install Docker for MicroROS
Add Docker's official GPG key:

    sudo apt-get update
    sudo apt-get install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

Add the repository to Apt sources:

    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
      sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update

Install Docker

    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

Create Docker group and user to docker group

    sudo groupadd docker
    sudo usermod -aG docker $USER
    newgrp docker


# Building the robot

### 1. Robot orientation
Robot Orientation:

-------------FRONT-------------

WHEEL1 WHEEL2 (2WD)

WHEEL3 WHEEL4 (4WD)

--------------BACK--------------


# Setting up configurations

Use your prefered text editor to edit the config file at the path: ~/RSI_linorobot2_hardware/config/custom/esp32s3_config.h
Get the GPIO pins by going through https://www.yahboom.net/study/MicroROS-Board
Things to configure for now is: 

    #define MAX_RPM_RATIO ??
    #define LR_WHEELS_DISTANCE ??
    
    #define MOTOR1_ENCODER_A 11
    #define MOTOR1_ENCODER_B 12
    #define MOTOR2_ENCODER_A 1
    #define MOTOR2_ENCODER_B 2
    #define MOTOR3_ENCODER_A 6
    #define MOTOR3_ENCODER_B 7
    #define MOTOR4_ENCODER_A 47
    #define MOTOR4_ENCODER_B 48

    #ifdef USE_BTS7960_MOTOR_DRIVER
      #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
      #define MOTOR1_IN_A ??
      #define MOTOR1_IN_B ??

      #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
      #define MOTOR2_IN_A ??
      #define MOTOR2_IN_B ??

      #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
      #define MOTOR3_IN_A ??
      #define MOTOR3_IN_B ??

      #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
      #define MOTOR4_IN_A ??
      #define MOTOR4_IN_B ??

      #define PWM_MAX pow(2, PWM_BITS) - 1
      #define PWM_MIN -PWM_MAX
    #endif

    #define SERVO_PIN 21


# Calibrating Robot

### 1. Upload calibration firmware to the robot

    cd  ~/RSI_linorobot2_hardware/calibration
    ./upload_firmware.sh

### 2. Calibrate motor direction
*    After the upload was successful, you should see that you connected to the ESP32 USB serial port
*    Make sure to the robots wheels are not touching the ground
*    Type the word "spin" and press ENTER
*    The robot wheels should spin one by one. Observe each wheel and note down the motors that spin backwards, we will invert it in the configs
*    Hit Ctrl+C to stop the serial communication

### 3. Update the motor direction configurations
Edit  ~/RSI_linorobot2_hardware/config/custom/esp32s3_config.h again, this time editing the motor direction

    #define MOTOR1_INV ??
    #define MOTOR2_INV ??
    #define MOTOR3_INV ??
    #define MOTOR4_INV ??

### 4. Calibrate motor encoder
Go back to calibration directory and connect to the ESP32 USB serial port again

    cd  ~/RSI_linorobot2_hardware/calibration
    pio device monitor
    
*    Press the RESET button in the development board
*    Make sure the robots wheels are not touching the ground
*    Type the word "sample" and press ENTER
*    After the wheels stop moving, take note which wheels have counts as negative, we will invert them in the config files again to make sure all counts my the motor encoder are positive

### 5. Update the motor encoder configuration
Edit  ~/RSI_linorobot2_hardware/config/custom/esp32s3_config.h again, this time editing the motor encoder

    #define MOTOR1_ENCODER_INV ??
    #define MOTOR2_ENCODER_INV ??
    #define MOTOR3_ENCODER_INV ??
    #define MOTOR4_ENCODER_INV ??


# Uploading firmware
Run the upload_firmware.sh script in the firmware directory

    cd ~/RSI_linorobot2_hardware/firmware
    ./upload_firmware.sh


# Run MicroROS Agent
### 1. Open two tabs in your terminal while in the root RSI_linorobot2_hardware directory 

    cd ~/RSI_linorobot2_hardware

On one terminal run the MicroROS agent script
    
    ./run_microros_agent.sh
   
### 3. On another terminal run the script that activates it. 
## IMPORTANT: The result should be feedback that the /dev/ttyUSB0 is constantly connecting and disconnecting. Don't worry, this is intended. Simply hit Ctrl+C to cancel it

    ./activate_agent.sh

### 3. Go back to the first tab and you should see the ROS2 topics being created.
### 4. To test the robot out if it can move, run the keyboard controller script in another tab

    ./keyboard_controller.sh


    
    







