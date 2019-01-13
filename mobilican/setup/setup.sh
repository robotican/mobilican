#!/bin/bash

###################### Author: Elhay Rauper ##########################
# ______  __    ___    __     ___  _____      ___   ___         ______
#  _____ |__\  |   |  |  \   |   |   |    |  |     /   \  |\  | _____
#   ____ | \   |   |  |__|_  |   |   |    |  |     |___|  | \ | ____
#    ___ |  \  |___|  |____| |___|   |    |  |___  |   |  |  \| ___
#
######################################################################

# from this point on, exit and notify immediately 
# if a command exits with a non-zero status
# set -eb

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
CATKIN_WS_SRC=`cd $SCRIPT_DIR/../../.. && pwd`
LOG_FILE=$CATKIN_WS_SRC/mobilican/mobilican/setup/log.txt
# init log file
echo "****** installation started at `date` ******" > $LOG_FILE

update_progress() {
    percentage=$1
    text=$2
    echo -e "XXX\n$percentage\n$text \nXXX"
    sleep 0.25
}

add_command() {
    cmd=$1
    txt=$2
    cmd_arr+=("$cmd")
    txt_arr+=("$txt")
}

log() {
    title="[$1]"
    text=$2
    echo "$title: $text" >> $LOG_FILE
}

# welcome screen
whiptail --textbox $SCRIPT_DIR/robotican.txt_img 20 75

# force entering password on next sudo command
sudo -k
# get user password for sudo use
psw=$(whiptail --title "Password Box" --passwordbox \
"Enter your password and choose Ok to continue." 10 60 3>&1 1>&2 2>&3)
# validate user password 
if sudo -S <<< $psw true &>/dev/null; then
    log "INFO" "correct sudo password"
else
    log "INFO" "correct sudo password"
    whiptail --title "Wrong Password" --msgbox \
    "The password you entedred is incorrect. Please restart setup and try again" 8 78
    exit 1
fi

# validate ros version
rosversion=`rosversion -d`
add_command \
"if [ "$rosversion" == "kinetic" ]; then 
    log 'INFO' 'ROS version OK' 
else 
    log 'ERROR' 'expected ROS version kinetic, actual version: \
    ROS ${rosversion}. please install ROS Kinetic and try again' 
    exit 1 
fi" \
"Validating ROS version..." 

# navigating to workspace
add_command "cd $CATKIN_WS_SRC" \
"Navigating to workspace..."

# updating system
add_command "sudo -S <<< $psw apt-get -y update" \
"Updating... updating packages" 
add_command "sudo -S <<< $psw apt-get -y dist-upgrade" \
"Updating packages... upgrading dist" 
add_command "sudo -S <<< $psw apt-get -y upgrade" \
"Updating packages... upgrading packages" 

# installing packages
add_command "sudo -S <<< $psw apt-get install python-catkin-tools" \
"Installing packages... catkin tools"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-controller-manager" \
"Installing packages... controller-manager"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-control-toolbox" \
"Installing packages... control-toolbox"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-transmission-interface" \
"Installing packages... transmission-interface"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-joint-limits-interface" \
"Installing packages... joint-limits-interface"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-ros-controllers" \
"Installing packages... controllers"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-ros-control" \
"Installing packages... control"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-move-base" \
"Installing packages... move-base"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-navigation" \
"Installing packages... navigation"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-hector-slam" \
"Installing packages... hector-slam"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-gmapping" \
"Installing packages... gmapping"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-pid" \
"Installing packages... pid"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-ar-track-alvar" \
"Installing packages... ar-track-alvar"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-serial" \
"Installing packages... serial"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-robot-localization" \
"Installing packages... robot-localization"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-trac-ik" \
"Installing packages... trac-ik"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-moveit-kinematics" \
"Installing packages... moveit-kinematics"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-urg-node" \
"Installing packages... urg-node"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-usb-cam" \
"Installing packages... usb-cam"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-rqt-robot-monitor" \
"Installing packages... rqt-robot-monitor"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-gazebo-ros-control" \
"Installing packages... gazebo-ros-control"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-moveit" \
"Installing packages... kinetic-moveit"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-moveit-ros-planning" \
"Installing packages... moveit-ros-planning"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-moveit-ros-planning-interface" \
"Installing packages... moveit-ros-planning-interface"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-twist-mux" \
"Installing packages... twist-mux"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-joy" \
"Installing packages... joy"
add_command "sudo -S <<< $psw apt-get -y install joystick" \
"Installing packages... joystick"
add_command "sudo -S <<< $psw apt-get -y install jstest-gtk" \
"Installing packages... jstest-gtk"
add_command "sudo -S <<< $psw apt-get -y install ros-kinetic-hector-gazebo-plugins" \
"Installing packages... hector-gazebo-plugins"
add_command "sudo -S <<< $psw apt-get -y install espeak espeak-data libespeak-dev" \
"Installing packages... espeak"

RIC_INTERFACE_ROS_V="1.1.0"
add_command "wget -q https://github.com/robotican/ric_interface_ros/archive/V$RIC_INTERFACE_ROS_V.tar.gz" \
"Installing packages: ric_interface_ros...  downloading"
add_command "tar -xvzf V$RIC_INTERFACE_ROS_V.tar.gz" \
"Installing packages: ric_interface_ros... untar"
add_command "rm V$RIC_INTERFACE_ROS_V.tar.gz" \
"Installing packages: ric_interface_ros... removing tar file"
add_command "cd $CATKIN_WS_SRC/ric_interface_ros-$RIC_INTERFACE_ROS_V/ric_interface_deb/" \
"Installing packages: ric_interface... finding intallation file"
add_command "sudo -S <<< $psw dpkg -i ric-interface.deb" \
"Installing packages: ric_interface... installing"

LPF_ROS_V="1.0.2"
add_command "cd $CATKIN_WS_SRC" \
"Installing packages: lpf_ros...  cd to src folder"
add_command "wget -q https://github.com/elhayra/lpf_ros/archive/V$LPF_ROS_V.tar.gz" \
"Installing packages: lpf_ros...  downloading"
add_command "tar -xvzf V$LPF_ROS_V.tar.gz" \
"Installing packages: lpf_ros... untar"
add_command "rm V$LPF_ROS_V.tar.gz" \
"Installing packages: lpf_ros... removing tar file"

ESPEAK_ROS_V="1.0.2"
add_command "wget -q https://github.com/robotican/espeak_ros/archive/V$ESPEAK_ROS_V.tar.gz" \
"Installing packages: espeak_ros...  downloading"
add_command "tar -xvzf V$ESPEAK_ROS_V.tar.gz" \
"Installing packages: espeak_ros... untar"
add_command "rm V$ESPEAK_ROS_V.tar.gz" \
"Installing packages: espeak_ros... removing tar file"

ROBOTEQ_CONTROL_V="0.1.0"
add_command "wget -q https://github.com/robotican/roboteq_control/archive/V$ROBOTEQ_CONTROL_V.tar.gz" \
"Installing packages: roboteq_control... downloading"
add_command "tar -xvzf V$ROBOTEQ_CONTROL_V.tar.gz" \
"Installing packages: roboteq_control... untar"
add_command "rm V$ROBOTEQ_CONTROL_V.tar.gz" \
"Installing packages: roboteq_control... removing tar file"

BMS_INTERFACE_V="0.1.0"
add_command "wget -q https://github.com/robotican/bms_interface/archive/$BMS_INTERFACE_V.tar.gz" \
"Installing packages: bms_interface... downloading"
add_command "tar -xvzf $BMS_INTERFACE_V.tar.gz" \
"Installing packages: bms_interface... untar"
add_command "rm $BMS_INTERFACE_V.tar.gz" \
"Installing packages: bms_interface... removing tar file"

DXL_INTERFACE_V="1.0.0"
add_command "wget -q https://github.com/robotican/dxl_interface/archive/V$DXL_INTERFACE_V.tar.gz" \
"Installing packages: dxl_interface... downloading"
add_command "tar -xvzf V$DXL_INTERFACE_V.tar.gz" \
"Installing packages: dxl_interface... untar"
add_command "rm V$DXL_INTERFACE_V.tar.gz" \
"Installing packages: dxl_interface... removing tar file"

DYNAMIXEL_SDK_V="1.0.0"
add_command "wget -q https://github.com/robotican/dynamixel_sdk/archive/V$DYNAMIXEL_SDK_V.tar.gz" \
"Installing packages: dynamixel_interface... downloading"
add_command "tar -xvzf V$DYNAMIXEL_SDK_V.tar.gz" \
"Installing packages: dynamixel_interface... untar"
add_command "rm V$DYNAMIXEL_SDK_V.tar.gz" \
"Installing packages: dynamixel_interface... removing tar file"

add_command "cd $CATKIN_WS_SRC/" \
"Installing packages: realsense"
add_command "wget -q https://github.com/intel-ros/realsense/archive/2.0.3.tar.gz" \
"Installing packages: realsense... downloading ros package"
add_command "tar -xvzf 2.0.3.tar.gz" \
"Installing packages: realsense... untar"
add_command "rm 2.0.3.tar.gz" \
"Installing packages: realsense... remove tar file"
add_command "wget -q https://github.com/IntelRealSense/librealsense/archive/v2.10.3.tar.gz" \
"Installing packages: realsense... downloading librealsense"
add_command "tar -xvzf v2.10.3.tar.gz" \
"Installing packages: realsense... untar"
add_command "rm v2.10.3.tar.gz" \
"Installing packages: realsense... remove tar file"
add_command "sudo apt-get -y install libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev" \
"Installing packages: realsense... depends"
add_command "cd librealsense-2.10.3 && mkdir build && cd build" \
"Installing packages: realsense... building"
add_command "cmake ../ && sudo -S <<< $psw make uninstall && make clean" \
"Installing packages: realsense... cleaning"
add_command "make -j8 > build_output.txt 2>&1 && sudo -S <<< $psw make install" \
"Installing packages: realsense... installing"

add_command "sudo -S <<< $psw apt -y install setserial" \
"Installing USB rules... setserial to low latency" 
add_command "sudo -S <<< $psw cp $CATKIN_WS_SRC/mobilican/mobilican/setup/rules/* /etc/udev/rules.d" \
"Installing USB rules... copying rules" 
add_command "sudo -S <<< $psw udevadm control --reload-rules && udevadm trigger" \
"Installing USB rules... applying rules" 

add_command "cd $CATKIN_WS_SRC/.. && catkin build" \
"Compiling Workspace..." 


{
    for (( i = 0; i < ${#cmd_arr[@]} ; i++ )); do
        percentage=$(( $i*100/${#cmd_arr[@]} ))
        update_progress $percentage "${txt_arr[$i]}"

        # Run each command in the commands array, and log errors/info output
        eval "${cmd_arr[$i]} >> $LOG_FILE 2>&1" >> $LOG_FILE 2>&1

        # Handle command result
        if [ $? == 0 ]; then
            log "INFO" "command: ${cmd_arr[$i]} finished successfully"
        else
            log "ERROR" "failed to execute: ${cmd_arr[$i]}, error code: $?"
            exit 1
        fi

        # print done after each command
        update_progress $percentage "${txt_arr[$i]} Done."

    done

    # finished message
    update_progress 100 "Setup finished!"
    sleep 1

} | whiptail --title "Installing Mobilican" --gauge "Preparing..." 6 60 0
# end of whiptail progressbar command

LAST_LOG_LINE="$(grep "." $LOG_FILE | tail -1)"

# if log's last line contains error, installation failed
if [[ $LAST_LOG_LINE == *"ERROR"* ]]; then
    whiptail --title "Installation Failed" --msgbox "$LAST_LOG_LINE" 8 78
    exit 1
else
    # finish and reboot
    whiptail --title "Installation Completed" --msgbox \
    "After hitting OK, PC will restart to complete installation." 8 78
    sudo -S <<< $psw reboot
fi





