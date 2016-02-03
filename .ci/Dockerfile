# VERSION 0.0.1
FROM ros:jade
#installing a bunch of dependencies
MAINTAINER Oliver Kalmend <kalmend.oliver@gmail.com>
LABEL Description="Speech recognition thesis"
RUN apt-get update
RUN apt-get install -y g++ python-pip
RUN apt-get install -y ros-jade-tf ros-jade-tf2 ros-jade-roslint
RUN pip install gcovr
RUN apt-get install -y ros-jade-audio-common
RUN apt-get install -y gstreamer1.0-plugins-bad  gstreamer1.0-plugins-base gstreamer1.0-plugins-good  gstreamer1.0-pulseaudio  gstreamer1.0-plugins-ugly  gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 
RUN apt-get install -y gstreamer0.10-plugins-bad  gstreamer0.10-plugins-base gstreamer0.10-plugins-good  gstreamer0.10-pulseaudio  gstreamer0.10-plugins-ugly  gstreamer0.10-tools libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev

#RUN pip install gcovr

# Copy reposity contents
RUN mkdir /test
RUN mkdir /test/workspace
# replace with real project name
COPY ./ /test/workspace/
ADD docker.sh .
RUN mkdir /results
CMD bash docker.sh 2> /results/stderr 1> /results/stdout
VOLUME /results
