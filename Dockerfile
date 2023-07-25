FROM osrf/ros:melodic-desktop-full

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Setup
RUN apt-get update
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get install -y --no-install-recommends apt-utils \
    dialog apt-utils locales lsb-release psmisc wget
RUN dpkg-reconfigure locales

RUN apt-get update --fix-missing
RUN apt-get install -y build-essential vim tmux python3-pip 
RUN pip3 install --upgrade pip
RUN pip3 install defusedxml rospkg netifaces numpy empy torch torch-vision \
    pandas scipy scikit-build tensorboard setuptools==59.5.0 \ 
    matplotlib opencv-python==4.6.0.66

RUN source /opt/ros/melodic/setup.bash \
    && mkdir -p ~/jackal_ws/src \
    && cd ~/jackal_ws/src \
    && git clone https://github.com/jackal/jackal.git --branch melodic-devel \
    && git clone https://github.com/jackal/jackal_simulator.git --branch melodic-devel \
    && git clone https://github.com/jackal/jackal_desktop.git --branch melodic-devel

COPY lflh/utils/.ignition /root/.ignition
COPY lflh/utils/nodeprocess.py /opt/ros/melodic/lib/python2.7/dist-packages/roslaunch/
COPY lflh/utils/lflh_navigation.rviz /root/jackal_ws/src/jackal_desktop/jackal_viz/rviz
COPY lflh/docker/barn_entrypoint.sh /
COPY . /root/jackal_ws/src/

RUN apt-get update --fix-missing \
    && apt-get install -y ros-melodic-pointgrey-camera-description \
    ros-melodic-velodyne-description \
    ros-melodic-map-server \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-twist-mux \
    ros-melodic-hector-gazebo-plugins \
    ros-melodic-sick-tim \
    ros-melodic-interactive-marker-twist-server \
    ros-melodic-teleop-twist-joy \
    ros-melodic-rosdoc-lite \
    ros-melodic-gmapping \
    ros-melodic-robot-localization \
    ros-melodic-amcl \
    ros-melodic-lms1xx \
    ros-melodic-move-base \
    ros-melodic-gazebo-plugins \
    ros-melodic-joy

RUN source /opt/ros/melodic/setup.bash \
    && cd ~/jackal_ws \
    && rosdep fix-permissions \
    && rosdep update \
    && apt-get update \
    && rosdep install -y --from-paths . --ignore-src --rosdistro=melodic

RUN source /opt/ros/melodic/setup.bash \
    && cd ~/jackal_ws \
    && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -j$(nproc)

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source ~/jackal_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root/jackal_ws/src/nav-competition-icra2023
ENTRYPOINT ["/barn_entrypoint.sh"]
CMD ["bash"]
