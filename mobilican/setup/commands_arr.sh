#!/bin/bash



#cmd_arr+="sudo -S <<< $psw apt update"
#txt_arr+="Updating... updating packages..."

#set -eb



add_command() {
    cmd=$1
    txt=$2
    cmd_arr+=("$cmd")
    txt_arr+=("$txt")
}

add_command "sleep 1" "printing a..." 
add_command "echo b" "printing b..." 
add_command "echo c" "printing c..." 
add_command "echo d" "printing d..."



update_progress() {
    percentage=$1
    text=$2
    echo -e "XXX\n$percentage\n$text \nXXX"
    sleep 0.5
}

{
    # prepare log file
    echo "installation started at `date`" > log.txt

    for (( i = 0; i < ${#cmd_arr[@]} ; i++ )); do
        percentage=$(( $i*100/${#cmd_arr[@]} ))
        update_progress $percentage "${txt_arr[$i]}"

        # Run each command in array 
        eval "${cmd_arr[$i]}"

        if [ $? == 0 ]; then
            echo "success ${cmd_arr[$i]}"
        else
           echo "failed to execute: ${cmd_arr[$i]}, error code: $?" >> log.txt
           exit 1
        fi

        update_progress $percentage "${txt_arr[$i]} Done."

    done

} | whiptail --title "Installing Mobilican" --gauge "Preparing..." 6 60 0











#eval "${cmd_arr[0]}"

##todo: echo with done after each command