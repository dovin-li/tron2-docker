FROM ros:noetic-ros-core-focal

RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y     gazebo11 libgazebo11-dev     ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control     ros-noetic-ros-control ros-noetic-ros-controllers     ros-noetic-hardware-interface ros-noetic-realtime-tools     ros-noetic-robot-state-publisher ros-noetic-xacro     ros-noetic-joy     ros-noetic-controller-manager ros-noetic-controller-interface     ros-noetic-joint-state-controller     libeigen3-dev liburdfdom-dev     build-essential python3-catkin-tools python3-pygame python3-flask ros-noetic-rviz ros-noetic-gazebo-plugins ros-noetic-move-base ros-noetic-amcl ros-noetic-gmapping ros-noetic-map-server ros-noetic-dwa-local-planner ros-noetic-rosbridge-server     && rm -rf /var/lib/apt/lists/*

# Stage 0: libraries / headers (no downstream deps)
COPY limxsdk-sim/ /ws/src/limxsdk_sim/
COPY onnxruntime_sdk/ /ws/src/onnxruntime_sdk/
COPY robot_common/ /ws/src/robot_common/
COPY limxsdk-lowlevel/ /ws/src/limxsdk_lowlevel/
COPY robot-description/ /ws/src/robot_description/

# Stage 1: packages with downstream deps
COPY tron2_gazebo/ /ws/src/tron2_gazebo/
COPY tron2_controllers/ /ws/src/tron2_controllers/
COPY tron2_hw/ /ws/src/tron2_hw/

RUN . /opt/ros/noetic/setup.sh && cd /ws &&     catkin config --install &&     catkin build limxsdk_sim onnxruntime_sdk robot_common limxsdk_lowlevel robot_description &&     catkin build tron2_gazebo tron2_controllers tron2_hw

COPY entry.sh /entry.sh
RUN chmod +x /entry.sh
ENTRYPOINT ["/entry.sh"]
