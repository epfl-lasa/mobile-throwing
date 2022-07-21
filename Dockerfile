ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION}
#FROM osrf/ros2:devel

RUN sudo apt-get update && sudo apt-get install -y python3-tk && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /home/${USER}/darko_pybullet_sim
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY --chown=${USER} . .
