FROM registry.gitlab.com/competitions4/airlab/stocking-challenge:1.4

LABEL maintainer="Daniel López Puig <daniel.lopez@pal-robotics.com>"

ARG REPO_WS=/ws
RUN mkdir -p /home/user/ws/src
WORKDIR /home/user/$REPO_WS

# TODO: Put inside ./ws your ROS packges
COPY ./ws /home/user/ws

# TODO: add here the debians you need to install
#RUN apt install -y ros-melodic-<pkg_name> pal-ferrum-<pkg_name> <apt-pkg>
# #Install git
# RUN apt-get update         
# RUN apt-get install -y git
# RUN git clone https://github.com/pal-robotics/tiago_tutorials.git

# Build and source your ros packages 
RUN bash -c "source /home/user/sim_ws/devel/setup.bash \
    && catkin build \
    && echo 'source /home/user/sim_ws/devel/setup.bash' >> ~/.bashrc"
    # Add below line to automatically source your packages
    # && echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc

RUN cd src && git clone https://github.com/pal-robotics/tiago_tutorials.git 
RUN echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc

USER user

ENTRYPOINT ["bash"]
