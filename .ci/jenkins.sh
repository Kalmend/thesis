#!/bin/bash
rm -rf results
rm -f docker.sh
rm -f Dockerfile
rm -f jenkins.sh
cp .ci/* .
mkdir results 
if [ ${REBUILD_DEPS} -ne 0 ]; then nocache="--no-cache"; fi
docker build $nocache -t ros-sprec .
docker run -v "${WORKSPACE}/results:/results" ros-sprec
