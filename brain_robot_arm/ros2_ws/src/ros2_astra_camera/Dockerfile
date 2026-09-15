FROM arm64v8/ros:humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# create a non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config \
    && apt-get update \
    # give sudo 
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
# RUN apt-get install -y python3-pip 
RUN apt install ros-humble-magic-enum -y 
RUN apt install ros-humble-camera-info-manager -y 
RUN apt install ros-humble-vision-opencv -y
RUN apt install ros-humble-image-pipeline -y
RUN apt install libuvc-dev -y
RUN apt-get install nlohmann-json3-dev -y
RUN apt-get install libgoogle-glog-dev -y


RUN apt-get install python3-opencv -y 
# RUN apt-get install -y python3-pip python3-venv
ENV SHELL /bin/bash

# install python packages
WORKDIR /astra
COPY requirements.txt .
# RUN python3 -m venv .venv
# RUN source .venv/bin/activate
# RUN python3 -m pip install -r requirements.txt

WORKDIR /astra/src
# RUN colcon build
# RUN source install/local_setup.bash