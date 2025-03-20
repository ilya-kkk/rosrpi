#!/bin/bash

# Install docker
for pkg in docker.io docker-doc docker-compose podman-docker containerd runc; do sudo apt-get remove $pkg; done

sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin


# sudo apt update && sudo apt install -y apt-transport-https \
#                                   ca-certificates \
#                                   curl \
#                                   gnupg-agent \
#                                   software-properties-common

# curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
# sudo apt-key fingerprint 0EBFCD88
# sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
# sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io
# sudo groupadd docker
# sudo usermod -aG docker $USER


# # NVIDIA Container Toolkit
# if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
#   then
#       distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
#       curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
#       curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

#       sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
#       sudo systemctl restart docker
# fi


newgrp docker
