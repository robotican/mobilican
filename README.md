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

1. If you haven't already, create a catkin workspace using ```catkin build```. Mobilican uses ```catkin_tools``` to compile, so workspace **must be created** using ```catkin build``` instead of ```catkin_make``` for example. See more details about ```catkin_tools``` [here](https://catkin-tools.readthedocs.io/en/latest/index.html)

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin build
```

2. Download mobilican package from latest release [here](https://github.com/robotican/mobilican/releases) into your workspace src folder
```
$ cd ~/catkin_ws/src
```

3. Extract the files into your workspace source directory (e.g. ~/catkin_ws/src/)

4. Navigate into the setup folder:
```
$ cd ~/catkin_ws/src/mobilican/mobilican/setup
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

## Updating Instructions
Updating existing mobilican package can be done automatically with the updater script.
The script assumes you downloaded the package from latest release (as opposed to ```git clone```), because it uses the package name to compare between newest version to the existing package version.

### Running Updater Script
********************************************
**!!!WARNING!!! IF NEWER VERSION EXIST, THE SCRIPT WILL DELETE ALL FILES IN WORKSPACE SRC FOLDER, AND STARTS UPDATE PROCESS IMMEDIATELY. BACK UP YOUR SRC FOLDER BEFORE USING THIS SCRIPT**
********************************************

1. cd into mobilican folder insdie mobilican package:
```
$ cd mobilican/mobilican
```
2. execute the script:
```
./updater.sh
```
3. if newer version exist, the script will start its installation process immediately by downloading the new package, and then the Mobilican installation screen will come up, asking you to press OK to start the new version installation process.  

## Usage Instructions



