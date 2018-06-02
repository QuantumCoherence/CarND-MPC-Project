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
- ![accel](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/psi.png?raw=true) is the orientation of the vehicle
- ![accel](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/v.png?raw=true) is the forward speed
- *cte* is the crosstrack error
- e ![equation3](http://www.sciweavers.org/tex2img.php?eq=\psi&fc=Black&im=jpg&fs=12&ff=arev&edit=) is the orientation error


The index indicates the timestamp

#### Full Kinematic Model


![model](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/discrete%20kinmatic%20model.jpg?raw=true)


where 

- *dt* is the duration of the discrete time interval
- ![accel](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/delta.png?raw=true)  is the steering control input
- ![accel](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/a.png?raw=true)  is the forward acceleration/breaking control input
- ![accel](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/psi.png?raw=true)des   is the desired orientation of the vehicle

Lf describes the distance between the rear wheel axel and the forward steering wheels, which is specific of the vehicle modeled and affects how the steering control input changes the vehicle orientation during motion.

### Cost Model and Control Input Constraints

The cost model is the set of conditions on the state variables that can be used to find the values of the control inputs, such that the cost model is minimized against the motion model, or in other words, such that the vehicle will drive as close as possible to the desired trajectory. The outcome of the optimal sovler is a set of control input that will minimize the error betwen planned path and actual trajectory.

#### The Update Loop and Latency

1. The current value of the state vector is simply obtained form the vehicle's sensors (current position, orientation, speed and current control inoput values - steering input and accelerator/brake input).
2. The desireed trajectory's equation is estimated using polynomial fitting from the waypoints, which are the outcome of the Path Planning.
3. A Latency update before optimal solver is applied using the Motion Model, to predict the future state vector at a time latency dt from the current state. The implication here is that it will take some time for the vehicle actuators to execute control inputs and it's therefore necessary to predict where the vehicle will have moved to, between the time of the current state vector and the expected time when the control inputs will be applied.This update state vector is then passed to the optimal solver.
4. The optimal solver finds the optimzed control inputs to achieve the desired trajectory as estimated using plynomiual fititng of the planned path and updated state vector.
5. After excution the new state vector is read from the vheicle sensor and the process is repeated.

### MPC Hyperparameter Estimation

The update loop only uses the first set of control inputs for actual actuation. It then updates the state vector with sensor reading and recalculates the next set of trajectory prediction and control inputs.
The reason for this is simply that the actuation errors are cumulative in nature and since we are modeling a non linear system, the error accumulation is exponential.

It serves little purpose to estimate the control inputs for a trajectory that is timewise far from the current state, becasue it will be inaccurate anyway. On the other hand, if the estimated trajectory is very short, the polynmial fitting will also be very inaccurate and consequently the control inputs will as well be indequate.

Estimating the future trajectory with a high resolution might sound like a good idea, since it will in theory give a set of control inputs able to guide the vehicle with intuitively as high resolution. Unfortuantley the latency of the system makes it purposless to calculate control inputs at a frequency that the system has no time to execute.
Therefore, in any case, the resolution of the optimization solver time step cannot be shorter than the latency of the system. The remaining question is, how much longer do we want the time step to be than the latency.
The answwer is fairly simple: the shortest possible.

Consequently the time step is dt = 100 ms, which is the simulated latency.

The final question is then about how many steps to simulate.
This is a trade off between computing demand and accuracy of the trajectory.
Empirically anything longer than 1 second up to 1.5 seconds works well. BY the law of "don't waste time computing thinga you don't need ... ", N was set at 10 steps, which is one second.
Anything longer than 15 steps started being unstable and did send the vehicle off the track. This is likely due to the solver not being able to complete the solution in time for the simulated delayed actuators to actully execute the estimated control inputs.





 
### Video

**Video**
Download the videoscreen capture of an entire loop around the track in ziped form, gunzip and play.

[PID MPC Video Download from here](https://github.com/QuantumCoherence/CarND-MPC-Project/blob/master/vokoscreen-2018-06-02_09-26-10.mkv.gz)


