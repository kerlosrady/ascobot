FROM registry.gitlab.com/competitions4/airlab/stocking-challenge:1.4

LABEL maintainer="Daniel López Puig <daniel.lopez@pal-robotics.com>"

ARG REPO_WS=/ws
RUN mkdir -p /home/user/ws/src
WORKDIR /home/user/$REPO_WS

# TODO: Put inside ./ws your ROS packges
COPY ./ws /home/user/ws


RUN apt-get update && apt-get install -y --no-install-recommends \
    python-rosinstall\
    python-pip 

RUN git clone --recursive https://github.com/opencv/opencv-python.git

RUN cd /home/user/ws/src
RUN sudo rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 joint_impedance_trajectory_controller"

# TODO: add here the debians you need to install
#RUN apt install -y ros-melodic-<pkg_name> pal-ferrum-<pkg_name> <apt-pkg>
WORKDIR /home/user/$REPO_WS

# Build and source your ros packages 
RUN bash -c "source /home/user/sim_ws/devel/setup.bash \
    && catkin build \
    && echo 'source /home/user/sim_ws/devel/setup.bash' >> ~/.bashrc"
    # Add below line to automatically source your packages


USER user

ENTRYPOINT ["bash"]