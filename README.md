# MPC-Project-Udacity

[//]: # (Image References)
[modelstate]: ./img/modelstate.JPG
[cte]: ./img/cte.JPG
[cte1]: ./img/cte1.JPG
[actuators]: ./img/actuators.JPG
[orientationerror]: ./img/orientationerror.JPG
[finalmodel]: ./img/finalmodel.JPG
[simulator]: ./img/Simulation.png


![Simulation][simulator]

## Introduction  

The purpose of this project is to implement an MPC Controller to provide an autonomous driven vehicle simulator with actuators' commands in real time. The simulator sends, by telemetry and through IP/TPC protocol, a part of the vehicle state component:

    1.position (x,y)
    2.orientation (psi)
    3. velocity (v)
and the waypoints (the desired trajectory). On the other side the MPC Controller processes this data with the following steps :

    1.Receive and Transform the acquired state vector component position from the global system to local (vehicle) system
    2.Define the vehicle model
    3.Fit the polynomial to the waypoints
    4.Calculate initial cross track error and orientation error
    5.Define N sets of state variables (x,y,psi,v,cte,ctepsi)
    6.Define the components of the cost function (state, actuators, etc)
    7.Define the model constraints using state update equations
    8.Use the optimizer to solve for the N sets of state variables that minimizes the cost while satisfying the constraints
    9.Apply the control variables (in this case, the acceleration and the delta rotation) that are in the very first set of the N and throw away the rest
    10.Go to 1 (Repeat the process)
    
    And sends back the simulator best optimized values of the steering angle and throttle.
    
    For a full simulation, find the video in the Video folder
    
## Considerations 

There are some considrations to take into account by the MPC Controller to ensure the best results 

    1. The considered vehicule model for this project is Kinematics bicycle model. It doesn't take into account the longitudinal and transversal forces at the tire level, neither the air resistance ...
    2. A time delay of 100 ms between the actuators command and the real time execution of the simulator to approach a real situation
    3. Make the prediction horizon (T= N*dt) small to inform the solver matching the reference trajectory fastly (continuously)

## Vehicle Model

### 1.State Model

![Model State][modelstate]

### 2.Cross Track Error (Cte)

The cross-track error is the difference between the reference trajectory and the current vehicle's position. The point in the path closest to the current position at time t is f(x), the error at time t is:

![CTE][cte]

where f(x) is the output of the polynomial that best fit the waypoints at point x (x-value of the current position, which in vehicle coordinates is 0). cte is the initial error

The cross track error at time t+1 is :

![CTE1][cte1]

### 3.Orientation Error

![Orientation Error][orientationerror]

### 4.Final Model

![Final Model][finalmodel]

### 5.Actuators

![Actuators][actuators]

### 6.Cost Function

Cost  = Sum_i cte(i)^2 
              + epsi(i)^2 
              + (v(i)-v_ref)^2 + delta(i)^2 
              + 10 a(i)^2 
              + 600 [delta(i+1)-delta(i)] 
              + [a(i+1)-a(i)]
              
## Discussions

### 1.Timestep Length &  Duration

I use for length and duration a classical approach of testing. So, the test set contains input values (N, dt) and output represented by textual observation. Several values were tested for N and dt to determine the best driven behavior. The following table reports the test set components :

| Length (N) | Duration (dt) | Observation |
|------------|---------------|-------------|
| 20 | .03  |  The vehicle crashes few seconds after the start and some oscillations|
| 20 | .06   | The vehicle drives with weak oscillations until the first turn after the bridge  and crashes |
| 20 |  .08|  The vehicle drives straight following the central line and crashes at the first turn before the bridge |
| 15 | 0.07 | The vehicle drives straight following the central line and crashes at the first turn before the bridge |
|15|.04| The vehicle drives fine until the bridge where it crashes|
|15|.07|The vehicle drives straight following the central line and crashes at the first turn before the bridge|
|13|.04| The vehicle crashes few seconds after the start and some strong oscillations|
|13|.05| The vehicle accomplishes two tracks with the maximum oscillations around the central yellow line|
|13|.06|The vehicle drives straight following the central line, runs with weak oscillations when crossing the bridge until the first turn and crashes there|
|12|.04|The vehicle crashes few seconds after the start and some strong oscillations|
|12|.06|The vehicle drives straight following the central line and crashes at the second turn after the bridge|
|10|.1|The vehicle accomplishes several tracks with the minimum oscillations around the central yellow line|

#### Conclusion
After playing around with the length and duration hyperparameters, i noticed that best choice is N=10 and dt=.1 for the optimizer to calculate the correct trajectory

### 2.Polynomial fitting

The waypoint are first converted from global coordinates system to vehicle's one by translating the the global origin (X0g,Y0g) to vehicle origin (X0v,Y0v) and rotating the vehicle origin by psi. The MapToCarTransform method in main.cpp implements the transform matrix (translation and rotation). The transform process is used to provide the polynomial expression an initial value (X0v=0) and therefore eases the fitting process. The result of the polynomial fitting are coefficients which will serve to calculate the cross-track error CTE.

### 3.Vehicle Latency

The algorithm used to solve the vehicle latency problem treats the estimation of the vehicle's state 100ms into the future before calling the MPC.solve() function, using the vehicle kinematics equations. This can be done using the update equations and model error equations described in the lessons. In vehicle coordinates, the vehicle is at the origin and so px, py and psi are considered as zero and substituting these values into kinematic equations with dt=0.1 will give the state and errors 100ms into the future.




