# MPC-Project-Udacity

[//]: # (Image References)
[modelstate]: ./img/modelstate.JPG
[cte]: ./img/cte.JPG
[cte1]: ./img/cte1.JPG
[actuators]: ./img/actuators.JPG
[orientationerror]: ./img/orientationerror.JPG
[finalmodel]: ./img/finalmodel.JPG


## Introduction  

The purpose of this project is to implement an MPC Controller for to provide an autonomous driven vehicle simulator with actuators' commands in real time. The simulator sends in real time, by telemetry through IP/TPC protocol, a part of the vehicle state component:

    1.position (x,y)
    2.orientation (psi)
    3. velocity (v)
and the waypoints (the desired trajectory). On the other side the MPC Controller processes the following steps :

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
              





