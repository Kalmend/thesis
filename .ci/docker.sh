#!/bin/bash
#PROJECT_NAME="my_ros_project"
# init workspace
source /opt/ros/jade/setup.bash
## Run roscore
#roscore &
cd /test/workspace
# Build with coverage support
./make_all_debug.sh
#catkin_make --cmake-args -DCMAKE_CXX_FLAGS="-fprofile-arcs -ftest-coverage"
#sleep 60
## Run tests
#catkin_make run_tests
#sleep 30
## Move test reports to results
#mkdir /results/gtest_reports/
#mv /test/build/test_results/* /results/gtest_reports/
## Generate coverage report
#cd build/${PROJECT_NAME}/CMakeFiles/${PROJECT_NAME}-test.dir/src/
#gcovr --xml -o coverage_report.xml -r /test/src/${PROJECT_NAME}/src/ .
## Move coverage report
#mv coverage_report.xml /results/
## Move .cpp files
#mv /test/src/${PROJECT_NAME}/src/* /results/
## Replace source directory in report
#sed -i 's/\/test\/src\/'${PROJECT_NAME}'\/src\//results\//g' /results/coverage_report.xml
chmod -R 777 /results/*
