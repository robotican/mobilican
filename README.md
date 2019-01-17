<p>
  <img src="/mobilican/docs/mobilican.png" width="700">
</p>

## Supported Hardware
1. Lizi 2 - Hardware ID: 1750466818 - Tested
2. Komodo 2 - Hardware ID: 1750467074 - Not Yet Tested
3. Armadillo 1 - Hardware ID: 1750467329 - Not Yet Tested
4. Armadillo 2 - Work in progress
5. Armadillo 2.1 - Work in progress

## Prerequisite 

* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get)

* Internet connection during installation process

## Installation Instructions

1. If you haven't already, create a catkin workspace using ```catkin build```. Mobilican uses ```catkin_tools``` to compile, so workspace must be created using ```catkin build``` instead of ```catkin_make``` for example. See more details about ```catkin_tools``` [here](https://catkin-tools.readthedocs.io/en/latest/index.html)

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin build
```

2. Navigate into your catkin workspace src folder. i.e.:
```
$ cd ~/catkin_ws/src
```

3. Download mobilican package:
```
$ git clone https://github.com/robotican/mobilican.git
```

4. Navigate into the setup folder:
```
$ cd mobilican/mobilican/setup
```

5. Run setup.sh script:
```
./setup.sh
```

6. You'll be required to insert your user password. Please do so as the script require root permition

7. Please be patiant during installation process. It might take some time to download and compile all the packages. Installation should normally take between 10-30 minutes. The script will automatically exit in case of failure, and show a message with corresponding error. In case of installation failure, check out the installation log file under 
```
mobilican/mobilican/setup/log.txt
```

## Usage Instructions



