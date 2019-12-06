# sdbackground

Package using Sigma Delta Background Substraction algorithm : [Paper](https://www.researchgate.net/publication/220843310_Sigma-Delta_Background_Subtraction_and_the_Zipf_Law)

Autor : Antoine Manzanera [mail](antoine.manzanera@ensta.fr)

### Status :

Kinetic : [![Build Status](http://jenkins-u2is.ensta.fr:8080/buildStatus/icon?job=docker+sdbg+kinetic)](http://jenkins-u2is.ensta.fr:8080/view/sdbg/job/docker%20sdbg%20kinetic/)
Melodic : [![Build Status](http://jenkins-u2is.ensta.fr:8080/buildStatus/icon?job=docker+sdbg+melodic)](http://jenkins-u2is.ensta.fr:8080/view/sdbg/job/docker%20sdbg%20melodic/)

Two jobs building the 2 docker images based on corresponding dockerfiles

### How to compile :

Simple compilation of the node (not installed in /opt/ros)

`catkin_make` 

Compile and run unit tests (gtest dependency)

`catkin_make run_tests`

Call the linter for coding style compliance, can fail as a test to block job

`catkin_make roslint_sdbackground`

Get the coverage based on unit tests and generate a html report (dependency with package code_coverage and lcov (system) )

`catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug sdbackground_coverage_report`

Consult package.xml for ros dependencies

### Node
 `sdbackground_node.cpp`
 The `sdbackground_node` provides a sdbackground node.
 
### Subscribed topic
 `in_image`
 Inputs the image topic to treate

### Published topic
 `out_image`
 Outputs the image topic result

### Parameters
 `amplification_factor`
 
*  type = int
*  default = 1`
*  Amplication factor for the algorithm
 
 `rate`

*  type = int
*  default = 10 
*  Frequency treatment rate (Hz)

