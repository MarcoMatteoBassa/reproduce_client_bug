# pull ros
FROM osrf/ros:humble-desktop

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /home/dev/ros2_ws/

# Add Entrypoint 
CMD [ "/bin/bash" ]
