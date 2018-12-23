#!/bin/bash

# ______  __    ___    __     ___  _____      ___   ___         ______
#  _____ |__\  |   |  |  \   |   |   |    |  |     /   \  |\  | _____
#   ____ | \   |   |  |__|_  |   |   |    |  |     |___|  | \ | ____
#    ___ |  \  |___|  |____| |___|   |    |  |___  |   |  |  \| ___

# from this point on, exit and notify immediately 
# if a command exits with a non-zero status
set -eb

# welcome screen
whiptail --textbox robotican.txt_img 20 75

# robot selection menu
ROBOT_SELECTION=$(whiptail --title "Choose your robot. Press space to select" --radiolist "Choose" 20 50 15 \
"LIZI_2" "Lizi 2 robot" ON \
"KOMODO_2" "Komodo 2 robot" OFF \
3>&1 1>&2 2>&3)

# get user password for sudo use
psw=$(whiptail --title "Password Box" --passwordbox "Enter your password and choose Ok to continue." 10 60 3>&1 1>&2 2>&3)
# check user password 
sudo -S <<< $psw -l
if ! [ "$?" == "0" ]; then
    whiptail --title "Incorrect Password" --msgbox "The password you entered is incorrect. Please try again..." 8 78
    
    exit 1
fi


update_progress() {
    percentage=$1
    text=$2
    echo -e "XXX\n$percentage\n$text \nXXX"
}

{  # start of whiptail progressbar command

    # validate ros version
    sleep 0.5
    update_progress 0 "Validating ROS version..."
    version=`rosversion -d`
    if [ "$version" == "kinetic" ]; then
        printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
    else
        printf "${RED_TXT}Error: found not compatible ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
        exit 1
    fi
    sleep 0.5
    update_progress 2 "Validating ROS version... Done."
    sleep 0.5

    #navigating to workspace
    update_progress 2 "Navigating to workspace..."
    CATKIN_WS_SRC=$( cd "$(dirname "$0")" && cd ../.. && pwd )
    cd $CATKIN_WS_SRC
    update_progress 3 "Navigating to workspace... Done."
    sleep 0.5

    # update packages
    update_progress 3 "Updating packages..."
    sudo -S <<< $psw apt-get -y update /
            dist-upgrade /
            upgrade
    update_progress 19 "Updating packages... Done."
    sleep 0.5

    # installing ros packages
    update_progress 19 "Installing ROS packages..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-controller-manager \
            ros-kinetic-control-toolbox \
            ros-kinetic-transmission-interface \
            ros-kinetic-joint-limits-interface \
            ros-kinetic-ros-controllers \
            ros-kinetic-ros-control \
            ros-kinetic-move-base \
            ros-kinetic-navigation \
            ros-kinetic-hector-slam \
            ros-kinetic-gmapping \
            ros-kinetic-pid \
            ros-kinetic-ar-track-alvar \
            ros-kinetic-serial \
            ros-kinetic-robot-localization \
            ros-kinetic-trac-ik ros-kinetic-moveit-kinematics  \
            ros-kinetic-urg-node \
            ros-kinetic-usb-cam \
            ros-kinetic-rqt-robot-monitor \
            ros-kinetic-hector-gazebo-plugins \
            espeak espeak-data libespeak-dev
    update_progress 47 "Installing ROS packages... Done."
    sleep 0.5

    #installing 3rd party packages
    update_progress 47 "Installing 3rd party packages: diff_slip_controller"
    DIFF_SLIP_CONTROLLER_V="1.0.0"
    wget https://github.com/robotican/diff_drive_slip_controller/archive/V"$DIFF_SLIP_CONTROLLER_V".tar.gz
    tar -xvzf V"$DIFF_SLIP_CONTROLLER_V".tar.gz
    rm V"$DIFF_SLIP_CONTROLLER_V".tar.gz

    update_progress 50 "Installing 3rd party packages: ric_interface_ros"
    RIC_INTERFACE_ROS_V="1.0.6"
    wget https://github.com/robotican/ric_interface_ros/archive/V"$RIC_INTERFACE_ROS_V".tar.gz
    tar -xvzf V"$RIC_INTERFACE_ROS_V".tar.gz
    rm V"$RIC_INTERFACE_ROS_V".tar.gz

    update_progress 54 "Installing 3rd party packages: mobilican_macros"
    MOBILICAN_MACROS_V="1.0.0"
    wget https://github.com/robotican/mobilican_macros/archive/V"$MOBILICAN_MACROS_V".tar.gz
    tar -xvzf V"$MOBILICAN_MACROS_V".tar.gz
    rm V"$MOBILICAN_MACROS_V".tar.gz

    update_progress 59 "Installing 3rd party packages: lpf_ros"
    LPF_ROS_V="1.0.1"
    wget https://github.com/elhayra/lpf_ros/archive/V"$LPF_ROS_V".tar.gz
    tar -xvzf V"$LPF_ROS_V".tar.gz
    rm V"$LPF_ROS_V".tar.gz

    update_progress 63 "Installing 3rd party packages: espeak_ros"
    ESPEAK_ROS_V="1.0.2"
    wget https://github.com/robotican/espeak_ros/archive/V"$ESPEAK_ROS_V".tar.gz
    tar -xvzf V"$ESPEAK_ROS_V".tar.gz
    rm V"$ESPEAK_ROS_V".tar.gz

    update_progress 67 "Installing 3rd party packages: mobilican_rules"
    MOBILICAN_RULES_V="1.0.1"
    wget https://github.com/robotican/mobilican_rules/archive/V"$MOBILICAN_RULES_V".tar.gz
    tar -xvzf V"$MOBILICAN_RULES_V".tar.gz
    rm V"$MOBILICAN_RULES_V".tar.gz

    #install ric_interface deb
    update_progress 71 "Installing 3rd party packages: ric_interface"
    cd $CATKIN_WS_SRC/ric_interface_ros-$RIC_INTERFACE_ROS_V/ric_interface_deb/
    sudo -S <<< $psw dpkg -i ric-interface.deb

    # realsense depth camera 
    update_progress 73 "Installing 3rd party packages: realsense"
    cd $CATKIN_WS_SRC/
    wget https://github.com/intel-ros/realsense/archive/2.0.3.tar.gz
    tar -xvzf 2.0.3.tar.gz
    rm 2.0.3.tar.gz     
    wget https://github.com/IntelRealSense/librealsense/archive/v2.10.3.tar.gz
    tar -xvzf v2.10.3.tar.gz
    rm v2.10.3.tar.gz
    sudo apt-get -y install libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev                                                                                                                                                
    cd librealsense-2.10.3                                                                                                                                                               
    mkdir build && cd build               
    cmake ../  
    sudo -S <<< $psw make uninstall && make clean && make -j8 && sudo make install
    update_progress 82 "Installing 3rd party packages... Done."
    sleep 0.5

    # installing usb rules
    update_progress 82 "Installing USB rules..."
    sudo -S <<< $psw apt -y install setserial #for setting port latency
    sudo -S <<< $psw cp $CATKIN_WS_SRC/mobilican_rules-$MOBILICAN_RULES_V/rules/* /etc/udev/rules.d
    ssudo -S <<< $psw udevadm control --reload-rules && udevadm trigger
    update_progress 88 "Installing USB rules... Done."
    sleep 0.5
    
    # compiling everything
    update_progress 88 "Compiling..."
    cd $CATKIN_WS_SRC/..
    catkin build -DCMAKE_BUILD_TYPE="Release"
    update_progress 100 "Compiling... Done."
    sleep 0.5

    # finished message
    update_progress 100 "Setup finished!"
    sleep 1

} |whiptail --title "Installing $ROBOT_SELECTION" --gauge "Installing..." 6 60 0
# end of whiptail progressbar command

# finish box
whiptail --textbox "Please restart your PC to complete installation process" 20 75




