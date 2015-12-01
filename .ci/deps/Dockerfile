# VERSION 0.0.1
FROM ros:jade
MAINTAINER Oliver Kalmend <kalmend.oliver@gmail.com>
LABEL Description="Speech recognition stack container dependencies"
RUN apt-get update
# Install non-ROS dependencies
RUN apt-get install -y g++ python-pip
# Install ROS dependencies
RUN apt-get install -y ros-jade-tf ros-jade-tf2
# Install pip based stuff
RUN pip install gcovr
