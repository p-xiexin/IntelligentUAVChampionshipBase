FROM m.daocloud.io/docker.io/osrf/ros:noetic-desktop-full

ADD src /basic_dev/src/
ADD setup.bash /

RUN chmod +x /setup.bash

USER root

ENV HTTPS_PROXY=192.168.0.176:7890

RUN apt update && apt install -y python3-catkin-tools ros-noetic-geographic-msgs \
 ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-image-transport \
 net-tools libeigen3-dev ros-noetic-pcl-conversions git libgoogle-glog-dev

RUN git clone https://github.com/Livox-SDK/Livox-SDK.git && \
 cd Livox-SDK && \
 cd build && cmake .. && \
 make && \
 make install && \
 cd ../.. && \
 rm -rf ./Livox-SDK

ENV ROS_DISTRO=noetic

WORKDIR /basic_dev/
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin build && . devel/setup.sh
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    catkin_make --only-pkg-with-deps airsim_ros && . devel/setup.sh && \
    catkin_make --only-pkg-with-deps livox_ros_driver && . devel/setup.sh && \
    catkin_make --only-pkg-with-deps mav_comm && . devel/setup.sh && \
    catkin_make --only-pkg-with-deps basic_dev && . devel/setup.sh && \
    catkin_make --only-pkg-with-deps imu_gps_odometry && . devel/setup.sh && \
    catkin_make --only-pkg-with-deps point_lio && . devel/setup.sh && \
    catkin_make --only-pkg-with-deps rotors_control && . devel/setup.sh


ENTRYPOINT [ "/setup.bash" ]