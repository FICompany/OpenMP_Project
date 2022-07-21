FROM ros:melodic-ros-base

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

RUN sudo apt-get update && sudo apt-get install -y vim iputils-ping net-tools python-pip ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher dos2unix

RUN sudo apt-get install -y ros-melodic-joint-state-publisher

RUN sudo -H pip install rospkg jsonpickle

ENV ROS_WORKSPACE=/catkin_ws

ENV DISPLAY=host.docker.internal:0.0

# Copy each directory explicitly to avoid workspace cruft
COPY ./ROS/src/moveit_msgs $ROS_WORKSPACE/src/moveit_msgs
COPY ./ROS/src/openmp_moveit $ROS_WORKSPACE/src/openmp_moveit
COPY ./ROS/src/open_mp_urdf $ROS_WORKSPACE/src/open_mp_urdf
COPY ./ROS/src/ros_tcp_endpoint $ROS_WORKSPACE/src/ros_tcp_endpoint

COPY ./docker/set-up-workspace /setup.sh
COPY docker/tutorial /

RUN /bin/bash -c "find $ROS_WORKSPACE -type f -print0 | xargs -0 dos2unix"

RUN dos2unix /tutorial && dos2unix /setup.sh && chmod +x /setup.sh && /setup.sh && rm /setup.sh

WORKDIR $ROS_WORKSPACE

# making sure the file modes are executable
RUN chmod +x src/openmp_moveit/scripts/*.py

ENTRYPOINT ["/tutorial"]

