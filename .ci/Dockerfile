# VERSION 0.0.1
FROM ros:jade
#installing a bunch of dependencies
MAINTAINER Oliver Kalmend <kalmend.oliver@gmail.com>
LABEL Description="Speech recognition thesis"
RUN apt-get update
RUN apt-get install -y g++ python-pip
RUN apt-get install -y ros-jade-tf ros-jade-tf2 ros-jade-roslint
RUN pip install gcovr

# Copy reposity contents
RUN mkdir /test
RUN mkdir /test/workspace
# replace with real project name
COPY * /test/workspace/
ADD docker.sh .
RUN mkdir /results
CMD bash docker.sh 2> /results/stderr 1> /results/stdout
VOLUME /results
