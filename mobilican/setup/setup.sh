#!/bin/bash

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
NO_COLOR='\033[0m'

# progress bar view function
# params: title, text, percentage
progress_view() {
    percentage=$1
    title=$2
    text=$3

    # installation script
    {
        # i="0"
        # while (true)
        # do
        #     proc=$(ps aux | grep -v grep | grep -e "mysqldump")
        #     if [[ "$proc" == "" ]]; then break; fi
        #     # Sleep for a longer period if the database is really big 
        #     # as dumping will take longer.
        #     sleep 1
        #     echo $i
        #     i=$(expr $i + 1)
        # done
        # If it is done then display 100%
        echo "$percentage"
        # Give it some time to display the progress to the user.
        sleep 2
    } | whiptail --title "$title" --gauge "$text" 8 78 0 
}

# welcome screen
whiptail --textbox mobilican/setup/robotican.txt_img 20 75
# robot selection menu
ROBOT_SELECTION=$(whiptail --title "Choose your robot. Press space to select" --radiolist "Choose" 20 50 15 \
"LIZI_2" "Lizi 2 robot" ON \
"KOMODO_2" "Komodo 2 robot" OFF \
3>&1 1>&2 2>&3)

progress_view 6 "Installing $ROBOT_SELECTION..." "doing something" 5
