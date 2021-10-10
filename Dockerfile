FROM registry.gitlab.com/ascobot

LABEL maintainer="Hend Aafia <hend.aafia@gmail.com>"

ARG REPO_WS=/ws
RUN mkdir -p ws/src
WORKDIR /home/user/$REPO_WS

# TODO: Put inside ./ws your ROS packges
COPY ./ws /home/user/ws

# TODO: add here the debians you need to install
#RUN apt install -y ros-melodic-<pkg_name> pal-ferrum-<pkg_name> <apt-pkg>

# Build and source your ros packages 
RUN bash -c "source /opt/pal/ferrum/setup.bash \
    && catkin build \
    && echo 'source /opt/pal/ferrum/setup.bash' >> ~/.bashrc"
    # \&& echo 'source devel/setup.bash'

ENTRYPOINT ["bash"]

