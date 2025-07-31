This repository is created to provide an external link for one of the supplementary materials of a manuscript submitted to *Cyborg and Bionic Systems* journal.

This repository contains MATLAB code corresponding to two experimental protocols (Session 2 and Session 3). The code can used to control parameters, such as speed, direction, inclination, acceleration and so on, of an instrumented treadmill (Bertec, USA), and is allowed to be modified for related applications. 

Detailed information about the associated article will be provided once it is accepted and published.



## Codes w.r.t. Experiment Session 2 (`MainProgram_session2.m`)
### Purpose
This program synchronously starts the Vicon/Nexus, Bertec/Treadmill and Exoskeleton, and controls the treadmill to run at specific incline and speed for level walking, level running, ramp ascent and ramp descent activities. Four speed perturbations will be imposed in each locomotion bout, with each two for speed forward (increase) and speed backward (decrease). 


### Procedures
(1) Configure and enable the Nexus UDP and the Bertec TCP function;  
(2) Run this program and wait for a start signal from robot via a TCP connection to trigger a synchronized start of all devices.  
(3) The program will terminate automatically when a termination condition is satisfied or once receiving a STOP signal from Robot.


### Check Items
(1) Check port number of TCP and UDP connections;  
(2) Check the content of Nexus `Trail Description`;  
(3) Check the variable values of `PSpeed` and `PSlope`;


The program has been tested on Matlab R2022b, Vicon Nexus 2.10.3, and Bertec Treadmill Control Panel 1.8.8.1.


## Codes w.r.t. Experiment Session 3 (`MainProgram_session3.m`)
### Purpose
This program synchronously starts the Vicon/Nexus (connected to Noraxon/Ultium), Bertec/Treadmill and Exoskeleton, and controls the treadmill to run at specific incline and speed for level walking, level running, ramp ascent and ramp descent activities.


### Procedures
(1) Configure and enable the Nexus UDP and the Bertec TCP function;  
(2) Connect the sync cables from Noraxon/sEMG to Lock Lab, and then set  the Noraxon MR software into Waiting Mode;  
(3) Run this program and wait for a start signal from robot via a TCP connection to trigger a synchronized start of all devices.  
(4) The program will terminate automatically when a termination condition is satisfied or once receiving a STOP signal from Robot.


### Check Items
(1) Check port number of TCP and UDP connections;  
(2) Check the content of Nexus `Trail Description`;  
(3) Check the variable values of `PSpeed` and `PSlope`;


The program has been tested on Matlab R2022b, Vicon Nexus 2.10.3, and Bertec Treadmill Control Panel 1.8.8.1.
