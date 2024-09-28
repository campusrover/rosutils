#!/bin/bash
set -e

infof=`tput setaf 6`
errorf=`tput setaf 1`
donef=`tput setaf 2`
reset=`tput sgr0`

case "$( lsb_release -d | grep -Eoi 'Ubuntu 16.04|Ubuntu 18.04|Ubuntu 20.04|Debian' )" in
  "Debian")
    gpg=https://pkgs.tailscale.com/stable/raspbian/buster.gpg
    list=https://pkgs.tailscale.com/stable/raspbian/buster.list
    ;;
  "Ubuntu 18.04")
    gpg=https://pkgs.tailscale.com/stable/ubuntu/bionic.gpg
    list=https://pkgs.tailscale.com/stable/ubuntu/bionic.list
    ;;
  "Ubuntu 20.04")
    gpg=https://pkgs.tailscale.com/stable/ubuntu/focal.gpg
    list=https://pkgs.tailscale.com/stable/ubuntu/focal.list
    ;;
  "Ubuntu 16.04")
    gpg=https://pkgs.tailscale.com/stable/ubuntu/xenial.gpg
    list=https://pkgs.tailscale.com/stable/ubuntu/xenial.list
    ;;
  *)
    echo "${errorf}  Unsupported OS${reset}"
    exit 1
    ;;
esac

if ! command -v tailscale &> /dev/null; then
  echo "${errorf}  Tailscale not found${reset}"
  echo "${infof}  Installing Tailscale client${reset}"
  echo "${infof}  Adding Tailscale repo${reset}"

  sudo apt-get install -y apt-transport-https curl
  curl ${gpg} | sudo apt-key add -
  curl ${list} | sudo tee /etc/apt/sources.list.d/tailscale.list

  echo "${infof}  Installing Tailscale${reset}"
  sudo apt-get update -y && apt-get install -y tailscale
fi

if [ -n "$1" ]; then
  echo -ne "${infof}  Connecting... Might takes up to 5 minutes \r${reset}"
  sudo tailscale up --authkey=$1 --accept-routes 2>&1

  addr=`ip addr show dev tailscale0 | grep -Eo '([0-9]{1,3}[\.]){3}[0-9]{1,3}'`
  if [ -n "${addr}" ]; then
    echo "${donef}  Connected. IP address: ${addr}${reset}"
  else
    echo "${errorf}  ERROR: Connection timeout${reset}"
    echo "${errorf}  Please run again with: sudo ./pi_connect.sh tskey-123abc...${reset}"
  fi
else
  echo "${errorf}  ERROR: Missing Authkey${reset}"
  echo "${errorf}  Please run again with: sudo ./pi_connect.sh tskey-123abc...${reset}"
fi
