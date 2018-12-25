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
#lizi_2="Lizi_2"
#armadillo_1="armadillo_1"
#komodo_2="komodo_2"
#armadillo_2="armadillo_2"
#ROBOT_SELECTION=$(whiptail --title "Choose your robot. Press space to select" --radiolist "Choose" 20 50 15 \
#"$lizi_2" "Lizi 2 robot" ON \
#"$komodo_2" "Komodo 2 robot" OFF \
#"$armadillo_1" "Armadillo 1 robot" OFF \
#"$armadillo_2" "Armadillo 2 robot" OFF \
#3>&1 1>&2 2>&3)

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
    sleep 0.5
}

{  # start of whiptail progressbar command

    # validate ros version
    update_progress 0 "Validating ROS version..."
    version=`rosversion -d`
    if [ "$version" == "kinetic" ]; then
        printf "${GREEN_TXT}ROS version OK${NO_COLOR}\n"
    else
        printf "${RED_TXT}Error: found not compatible ROS version ${version}, please install ROS Kinetic and try again${NO_COLOR}\n"
        exit 1
    fi
    update_progress 2 "Validating ROS version... Done."

    #navigating to workspace
    update_progress 2 "Navigating to workspace..."
    CATKIN_WS_SRC=$( cd "$(dirname "$0")" && cd ../.. && pwd )
    cd $CATKIN_WS_SRC
    update_progress 3 "Navigating to workspace... Done."

    # update packages
    update_progress 3 "Updating... updating packages..."
    sudo -S <<< $psw apt-get -y update 
    update_progress 7 "Updating... updating packages... Done"
    update_progress 7 "Updating packages... upgrading dist..."
    sudo -S <<< $psw apt-get -y dist-upgrade 
    update_progress 13 "Updating packages... upgrading dist... Done"
    update_progress 13 "Updating packages... upgrading packages..."
    sudo -S <<< $psw apt-get -y upgrade
    update_progress 19 "Updating packages... upgrading packages... Done."
    update_progress 19 "Updating packages... Done."

    # installing ros packages
    update_progress 19 "Installing ROS packages..."
    update_progress 19 "Installing ROS packages... installing controller-manager..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-controller-manager 
    update_progress 21 "Installing ROS packages... installing controller-manager... Done"
    update_progress 21 "Installing ROS packages... installing control-toolbox..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-control-toolbox 
    update_progress 23 "Installing ROS packages... installing control-toolbox... Done"
    update_progress 23 "Installing ROS packages... installing control-toolbox..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-transmission-interface 
    update_progress 25 "Installing ROS packages... installing transmission-interface... Done"
    update_progress 25 "Installing ROS packages... installing joint-limits-interface..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-joint-limits-interface 
    update_progress 26 "Installing ROS packages... installing joint-limits-interface... Done"
    update_progress 26 "Installing ROS packages... installing ros-controllers..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-ros-controllers 
    update_progress 29 "Installing ROS packages... installing ros-controllers... Done"
    update_progress 29 "Installing ROS packages... installing ros-control..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-ros-control 
    update_progress 30 "Installing ROS packages... installing ros-control... Done"
    update_progress 30 "Installing ROS packages... installing move-base..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-move-base 
    update_progress 31 "Installing ROS packages... installing move-base... Done"
    update_progress 31 "Installing ROS packages... installing navigation..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-navigation 
    update_progress 33 "Installing ROS packages... installing navigation... Done"
    update_progress 33 "Installing ROS packages... installing hector-slam..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-hector-slam 
    update_progress 34 "Installing ROS packages... installing hector-slam... Done"
    update_progress 34 "Installing ROS packages... installing gmapping..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-gmapping 
    update_progress 35 "Installing ROS packages... installing gmapping... Done"
    update_progress 35 "Installing ROS packages... installing pid..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-pid 
    update_progress 36 "Installing ROS packages... installing pid... Done"
    update_progress 36 "Installing ROS packages... installing ar-track-alvar..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-ar-track-alvar 
    update_progress 37 "Installing ROS packages... installing ar-track-alvar... Done"
    update_progress 37 "Installing ROS packages... installing serial..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-serial 
    update_progress 38 "Installing ROS packages... installing serial... Done"
    update_progress 38 "Installing ROS packages... installing robot-localization..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-robot-localization 
    update_progress 39 "Installing ROS packages... installing robot-localization... Done"
    update_progress 39 "Installing ROS packages... installing trac-ik..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-trac-ik 
    update_progress 40 "Installing ROS packages... installing trac-ik... Done"
    update_progress 40 "Installing ROS packages... installing moveit-kinematics..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-moveit-kinematics  
    update_progress 41 "Installing ROS packages... installing moveit-kinematics... Done"
    update_progress 41 "Installing ROS packages... installing urg-node..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-urg-node 
    update_progress 42 "Installing ROS packages... installing urg-node... Done"
    update_progress 42 "Installing ROS packages... installing usb-cam..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-usb-cam 
    update_progress 43 "Installing ROS packages... installing usb-cam... Done"
    update_progress 43 "Installing ROS packages... installing rqt-robot-monitor..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-rqt-robot-monitor 
    update_progress 44 "Installing ROS packages... installing controller rqt-robot-monitor... Done"
    update_progress 44 "Installing ROS packages... installing control hector-gazebo-plugins..."
    sudo -S <<< $psw apt-get -y install ros-kinetic-hector-gazebo-plugins 
    update_progress 53 "Installing ROS packages... installing hector-gazebo-plugins... Done"
    update_progress 53 "Installing ROS packages... Done."

    #installing 3rd party packages
    update_progress 53 "Installing ROS packages... installing espeak..."
    sudo -S <<< $psw apt-get -y install espeak espeak-data libespeak-dev
    update_progress 54 "Installing ROS packages... installing espeak... Done"

    update_progress 54 "Installing 3rd party packages: diff_slip_controller..."
    DIFF_SLIP_CONTROLLER_V="1.0.0"
    wget https://github.com/robotican/diff_drive_slip_controller/archive/V"$DIFF_SLIP_CONTROLLER_V".tar.gz
    tar -xvzf V"$DIFF_SLIP_CONTROLLER_V".tar.gz
    rm V"$DIFF_SLIP_CONTROLLER_V".tar.gz
    update_progress 55 "Installing 3rd party packages: diff_slip_controller... Done"

    update_progress 55 "Installing 3rd party packages: ric_interface_ros..."
    RIC_INTERFACE_ROS_V="1.0.6"
    wget https://github.com/robotican/ric_interface_ros/archive/V"$RIC_INTERFACE_ROS_V".tar.gz
    tar -xvzf V"$RIC_INTERFACE_ROS_V".tar.gz
    rm V"$RIC_INTERFACE_ROS_V".tar.gz
    update_progress 57 "Installing 3rd party packages: ric_interface_ros... Done"

    update_progress 57 "Installing 3rd party packages: mobilican_macros..."
    MOBILICAN_MACROS_V="1.0.0"
    wget https://github.com/robotican/mobilican_macros/archive/V"$MOBILICAN_MACROS_V".tar.gz
    tar -xvzf V"$MOBILICAN_MACROS_V".tar.gz
    rm V"$MOBILICAN_MACROS_V".tar.gz
    update_progress 58 "Installing 3rd party packages: mobilican_macros... Done"


    update_progress 58 "Installing 3rd party packages: lpf_ros..."
    LPF_ROS_V="1.0.1"
    wget https://github.com/elhayra/lpf_ros/archive/V"$LPF_ROS_V".tar.gz
    tar -xvzf V"$LPF_ROS_V".tar.gz
    rm V"$LPF_ROS_V".tar.gz
    update_progress 60 "Installing 3rd party packages: lpf_ros... Done"


    update_progress 60 "Installing 3rd party packages: espeak_ros"
    ESPEAK_ROS_V="1.0.2"
    wget https://github.com/robotican/espeak_ros/archive/V"$ESPEAK_ROS_V".tar.gz
    tar -xvzf V"$ESPEAK_ROS_V".tar.gz
    rm V"$ESPEAK_ROS_V".tar.gz
    update_progress 63 "Installing 3rd party packages: espeak_ros... Done"


    #install ric_interface deb
    update_progress 71 "Installing 3rd party packages: ric_interface..."
    cd $CATKIN_WS_SRC/ric_interface_ros-$RIC_INTERFACE_ROS_V/ric_interface_deb/
    sudo -S <<< $psw dpkg -i ric-interface.deb
    update_progress 74 "Installing 3rd party packages: ric_interface... Done"


    # realsense depth camera 
    update_progress 74 "Installing 3rd party packages: realsense..."
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
    update_progress 83 "Installing 3rd party packages: realsense... Done"

    update_progress 83 "Installing 3rd party packages... Done."

    # installing usb rules
    update_progress 83 "Installing USB rules..."
    update_progress 83 "Installing USB rules... setserial to low latency..."
    sudo -S <<< $psw apt -y install setserial #for setting port latency
    update_progress 84 "Installing USB rules... setserial to low latency... Done"
    update_progress 84 "Installing USB rules... copying rules"
    sudo -S <<< $psw cp $CATKIN_WS_SRC/mobilican_rules-$MOBILICAN_RULES_V/rules/* /etc/udev/rules.d
    update_progress 86 "Installing USB rules... copying rules... Done"
    update_progress 86 "Installing USB rules... applying rules"
    ssudo -S <<< $psw udevadm control --reload-rules && udevadm trigger
    update_progress 88 "Installing USB rules... applying rules... Done"
    update_progress 88 "Installing USB rules... Done."

    # compiling everything
    update_progress 88 "Compiling..."
    cd $CATKIN_WS_SRC/..
    catkin build
    update_progress 100 "Compiling... Done."

    # finished message
    update_progress 100 "Setup finished!"
    sleep 1
    
} | whiptail --title "Installing Mobilican" --gauge "Preparing..." 6 60 0
# end of whiptail progressbar command

# finish box
whiptail --textbox "Please restart your PC to complete installation process" 20 75




