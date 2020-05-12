# ABB - Externally Guided Motion
## Introduction
This project contains the necessary code to implement Externally Guided Motion (EGM) functionality in ABB robots and external axes. The project also has EGM implementations external axes.

## Description
EGM is a new funtionality in ABB robots that allows the robot motion to be controlled from an external master. EGM used Google Protobuf protocols to enable a high speed low latency UDP communication. THis enables high response control over the manipulators. Since this feature has limited industry adoptation, there are only limited resources available on configuring EGM. This repository contains wrappers that are built over the original base code for an easy and elegant implementation of EGM. It also contains an example for controlling external axis through EGM which is not an officially supported functionality by ABB (as of now). In the example program, two robots and two tracks are controlled simultaneously on different threads. This project is targeted for windows platform.

## Software Requirements
    - Visual Studio 2018 or later

## License
This project is licensed under the BSD 2-Clause License - see the LICENSE for more details