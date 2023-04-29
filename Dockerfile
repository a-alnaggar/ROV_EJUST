FROM ros:noetic-ros-base

RUN sudo apt update -y &&\
    sudo apt install python3-pip -y &&\
    sudo apt install nano -y