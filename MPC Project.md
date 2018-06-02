# MPC Project
## GitHub Repo
[CarND-MPC-Project](https://github.com/QuantumCoherence/CarND-MPC-Project)


This project was completed on Ubuntu 16.04.

For both github repo links, the ecbuild subfolder contains all ECLIPSE project files, including compiled binary.

The build subfolder contains all the binaries and the makefile for compiling the project at the prompt with no formal IDE. 

**Compiling on your system: CMakelists.txt**

Use the CMakeLists.txt in the repo root folder to create the make file and compile on your system.


### Model Predictive Control model	

The model used to estimate the vehicle control inputs is based on the model dicussed during the MPC class.

#### State Vector


![model](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/model.jpg?raw=true)

where 

- X and Y are the position in global coordinates
- ![equation](http://www.sciweavers.org/tex2img.php?eq=a&fc=Black&im=jpg&fs=12&ff=arev&edit=)  is the orientation of the vehicle
- ![equation](http://www.sciweavers.org/tex2img.php?eq=v&fc=Black&im=jpg&fs=12&ff=arev&edit=) is the forward speed
- *cte* is the crosstrack error
- e ![equation](http://www.sciweavers.org/tex2img.php?eq=\psi&fc=Black&im=jpg&fs=12&ff=arev&edit=) is the orientation error


The index indicates the timestamp

#### Full Kinematic Model


![model](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/discrete%20kinmatic%20model.jpg?raw=true)


where 

- *dt* is the duration of the time interval
- ![equation](http://www.sciweavers.org/tex2img.php?eq=\delta&fc=Black&im=jpg&fs=12&ff=arev&edit=)  is the steering control input
- ![equation](http://www.sciweavers.org/tex2img.php?eq=a&fc=Black&im=jpg&fs=12&ff=arev&edit=)  is the forward acceleration/breaking control input
- e ![equation](http://www.sciweavers.org/tex2img.php?eq=\psi des&fc=Black&im=jpg&fs=12&ff=arev&edit=)  is the desired orientation of the vehicle

Lf describes the distance between the rear wheel axel and the forward steering wheels, which is specific of the vehicle modeled and affects how the steering control input changes the vehicle orientation during motion.


 
#### Video and Images

**Video**
Download the videoscreen capture of an entire loop around the track in ziped form, gunzip and play.

[PID MPC Video Download from here](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/vokoscreen-2018-06-02_09-26-10.mkv.gz)

**Images**

``


![sample 1]()


![sample 2]()

