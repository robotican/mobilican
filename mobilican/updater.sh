#!/bin/bash

GREEN_TXT='\e[0;32m'
WHITE_TXT='\e[1;37m'
RED_TXT='\e[31m'
YELLOW_TXT='\e[1;33m'
NO_COLOR='\033[0m'

print_white() {
  printf "${WHITE_TXT}$1\n${NO_COLOR}"
}

print_green() {
  printf "${GREEN_TXT}$1\n${NO_COLOR}"
}

print_red() {
  printf "${RED_TXT}$1\n${NO_COLOR}"
}

print_yellow() {
  printf "${YELLOW_TXT}$1\n${NO_COLOR}"
}

# get latest version tag from github
get_latest_release() {
  curl --silent "https://api.github.com/repos/$1/releases/latest" | # Get latest release from GitHub api
  grep '"tag_name":' |                                            # Get tag line
  sed -E 's/.*"([^"]+)".*/\1/'                                    # Pluck JSON value
}

# get installed package version (from its name)
get_package_version() {
  echo "$(basename $1)" | cut -d"$2" -f2-
}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
MOBILICAN_DIR=`cd $SCRIPT_DIR/../ && pwd`
PACKAGE_VERSION="V$(get_package_version $MOBILICAN_DIR '-')"

# if package was not downloaded from a release (e.g the user downloaded 
# it using git clone), the name of the mobilican package will not include 
# version number. In this case the script will fail
print_white "*** Mobilican Updater ***\n"
print_white "Checking package version..."
if [ "$PACKAGE_VERSION" = "Vmobilican" ]; then
  print_red "Update failed: Your package name doesn't include release version."
  exit 1
else
  print_white "Found package version: $PACKAGE_VERSION"
fi

print_white "Checking latest version..."
LATEST_TAG=$(get_latest_release "robotican/mobilican")
print_white "Latest available version: $LATEST_TAG"

if [ "$LATEST_TAG" = "$PACKAGE_VERSION" ]; then
  print_green "Your package is up to date."
else
  print_yellow "Your package is out of date, installing latest version..."
  sleep 2
  cd $MOBILICAN_DIR/.. #cd into src folder
  print_white "Cleaning src folder..."
  rm -rf ./*
  print_white "Downloading updated package..."
  wget https://github.com/robotican/mobilican/archive/$LATEST_TAG.tar.gz
  tar -xvzf $LATEST_TAG.tar.gz
  rm $LATEST_TAG.tar.gz
  LATEST_TAG_NUM=$(echo $LATEST_TAG | cut -c2-)
  cd mobilican-$LATEST_TAG_NUM/mobilican/setup/
  ./setup.sh
fi