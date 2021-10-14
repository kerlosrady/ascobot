FROM registry.gitlab.com/competitions4/airlab/stocking-challenge:1.4

LABEL maintainer="Daniel López Puig <daniel.lopez@pal-robotics.com>"

ARG REPO_WS=/ws
RUN mkdir -p /home/user/ws/src
WORKDIR /home/user/$REPO_WS

# TODO: Put inside ./ws your ROS packges
COPY ./ws /home/user/ws

RUN rosinstall src /opt/ros/melodic tiago_public.rosinstall

# TODO: add here the debians you need to install
#RUN apt install -y ros-melodic-<pkg_name> pal-ferrum-<pkg_name> <apt-pkg>

# Build and source your ros packages 
RUN bash -c "source /home/user/sim_ws/devel/setup.bash \
    && catkin build \
    && echo 'source /home/user/sim_ws/devel/setup.bash' >> ~/.bashrc"
    # Add below line to automatically source your packages


USER user

ENTRYPOINT ["bash"]