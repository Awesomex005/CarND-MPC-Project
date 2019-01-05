# Mode Predictive Control

## The mode
The mode used here is Kinematic Model. 

### state and actuator
[x,y,ψ,v] is the state of the vehicle, [delta, a] are the actuators of the vehicle. 

state|description
-|-
x, y| position of the vehicle 
ψ| the angle between vehicle facing and x axis 
v| magnitude of the vechicle velocity 
delta| steering angle 
a| acceleration on vechicle facing direction 


### state update equations
```
x1 = (x0 + v0 * cos(ψ) * dt); 
y1 = (y0 + v0 * sin(ψ) * dt); 
psi1 = (psi0 + v0 * delta0 / Lf * dt); 
v1 = (v0 + a0 * dt); 
```
where **Lf** is the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower the turn rate.

## Timestep length and Elapsed Duration (N & dt)
**N** = 15, **dt** = 0.1 were chosen. 

So prediction horizon here 1.5 seconds was chosen, because in high speed situation with more larger horizon, 
the environment will change enough to have the further prediction no more make sense. 

Then the smaller the dt is the more accuracy our prediction will be, but with smaller dt we will have larger N that increase computational cost.
So with the limited compuation capability we chosen dt = 0.1 here.

I used to choose the conbination of N = 30, dt = 0.05, and it performed as what we expected, it took to much time to optimize the prediction, the car drive off the road.

## Polynomial Fitting and MPC Preprocessing
The waypoints from simulator are in Map coordinate. 
At some points of the map, the trajectory/waypoints could perpendicular to the x axis, multipe y map to the same x.
At such point, it is diffcult to fit a polunomial for that trajectory.

So before Polynomial Fitting, we change the waypoints to **vehicle coordinate**. So that the waypoints are always parallel to x axis(in vehicle coordinate).
```
void g2v_coordinate_transform(vector<double> &trans_ptsx, vector<double> &trans_ptsy,
                              vector<double> &ptsx, vector<double> &ptsy, double px, double py, double psi){
  double trans_ptsx_;
  double trans_ptsy_;
  for(int i=0; i<ptsx.size(); i++){
      trans_ptsx_ =      cos(psi)*(ptsx[i] - px) + sin(psi)*(ptsy[i] - py);
      trans_ptsy_ = (-1)*sin(psi)*(ptsx[i] - px) + cos(psi)*(ptsy[i] - py);
      trans_ptsx.push_back(trans_ptsx_);
      trans_ptsy.push_back(trans_ptsy_);
  }
}
```

## Model Predictive Control with Latency
To deal with the latency, we make a additional prediction before MPC procedure. The first MPC state is the output of that prediction.

Furthe more we also take the time cost of MPC procedure into account, which makes the control more stable and accuracy.
