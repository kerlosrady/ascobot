FROM registry.gitlab.com/competitions4/airlab/stocking-challenge:1.4

LABEL maintainer="Daniel López Puig <daniel.lopez@pal-robotics.com>"

ARG REPO_WS=/ws
RUN mkdir -p /home/user/ws/src
WORKDIR /home/user/$REPO_WS

# TODO: Put inside ./ws your ROS packges
COPY ./ws /home/user/ws

# TODO: add here the debians you need to install
#RUN apt install -y ros-melodic-<tiago_public.rosinstall> pal-ferrum-<tiago_public.rosinstall> <apt-pkg>
RUN mkdir tiago_dep
WORKDIR /home/user/ws/tiago_dep

RUN git clone 'https://github.com/pal-robotics/aruco_ros.git'
RUN git clone 'https://github.com/pal-robotics-forks/gazebo_ros_pkgs.git'
RUN git clone 'https://github.com/pal-robotics/head_action.git'
RUN git clone 'https://github.com/pal-robotics/hey5_description.git'
RUN git clone 'https://github.com/pal-robotics/pal_gazebo_plugins.git'
RUN git clone 'https://github.com/pal-robotics/pal_gazebo_worlds.git'
RUN git clone 'https://github.com/pal-robotics/pal_gripper.git'
RUN git clone 'https://github.com/pal-robotics/pal_hardware_gazebo.git'
RUN git clone 'https://github.com/pal-robotics/pal_hardware_interfaces.git'
RUN git clone 'https://github.com/pal-robotics/pal_msgs.git'
RUN git clone 'https://github.com/pal-robotics/pal_navigation_sm.git'
RUN git clone 'https://github.com/pal-robotics/pal_python.git'
RUN git clone 'https://github.com/pal-robotics/play_motion.git'
RUN git clone 'https://github.com/pal-robotics/pmb2_robot.git'
RUN git clone 'https://github.com/pal-robotics/pmb2_simulation.git'
RUN git clone 'https://github.com/pal-robotics/pmb2_navigation'
RUN git clone 'https://github.com/pal-robotics-forks/ros_control.git'
RUN git clone 'https://github.com/pal-robotics-forks/ros_controllers.git'
RUN git clone 'https://github.com/pal-robotics/rviz_plugin_covariance.git'
RUN git clone 'https://github.com/pal-robotics/simple_grasping_action.git'
RUN git clone 'https://github.com/pal-robotics/simple_models_robot.git'
RUN git clone 'https://github.com/pal-robotics/tf_lookup.git'
RUN git clone 'https://github.com/pal-robotics/tiago_description_calibration.git'
RUN git clone 'https://github.com/pal-robotics/tiago_moveit_config.git'
RUN git clone 'https://github.com/pal-robotics/tiago_navigation.git'
RUN git clone 'https://github.com/pal-robotics/tiago_robot.git'
RUN git clone 'https://github.com/pal-robotics/tiago_simulation.git'
RUN git clone 'https://github.com/pal-robotics/tiago_tutorials.git'
RUN git clone 'https://github.com/pal-robotics/pal_navigation_cfg_public.git'
RUN git clone 'https://github.com/pal-robotics/ddynamic_reconfigure_python'
RUN git clone 'https://github.com/pal-robotics/dynamic_introspection'
RUN git clone 'https://github.com/pal-robotics/backward_ros'
RUN git clone 'https://github.com/pal-robotics/pal_statistics'
RUN git clone 'https://github.com/pal-robotics/robot_pose'
RUN git clone 'https://github.com/pal-robotics/pal_wsg_gripper'
RUN git clone 'https://github.com/pal-robotics-forks/roboticsgroup_gazebo_plugins.git'
RUN git clone 'https://github.com/pal-robotics-forks/teleop_tools.git'
RUN git clone 'https://github.com/ahornung/humanoid_msgs.git'
RUN git clone 'https://github.com/ros-perception/slam_gmapping.git'
RUN git clone 'https://github.com/ros-perception/openslam_gmapping.git'
RUN git clone 'https://github.com/pal-robotics-forks/eband_local_planner.git'
RUN git clone 'https://github.com/DLu/navigation_layers.git'
RUN git clone 'https://github.com/pal-robotics/custom_end_effector.git'
RUN git clone 'https://github.com/pal-robotics/pal_robotiq_gripper.git'

WORKDIR /home/user/$REPO_WS

# Build and source your ros packages 
RUN bash -c "source /home/user/sim_ws/devel/setup.bash \
    && catkin build \
    && echo 'source /home/user/sim_ws/devel/setup.bash' >> ~/.bashrc"
    # Add below line to automatically source your packages
    # && echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc

RUN echo 'source $REPO_WS/devel/setup.bash' >> ~/.bashrc

USER user

ENTRYPOINT ["bash"]
