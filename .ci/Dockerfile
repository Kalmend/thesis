# VERSION 0.0.1
FROM ros-sprec-deps
MAINTAINER Oliver Kalmend <kalmend.oliver@gmail.com>
LABEL Description="Speech recognition thesis"
# Copy reposity contents
RUN mkdir /test
RUN mkdir /test/src
COPY my_ros_project /test/src/my_ros_project # replace with real value
ADD docker.sh .
RUN mkdir /results
CMD bash docker.sh 2> /results/stderr 1> /results/stdout
VOLUME /results
