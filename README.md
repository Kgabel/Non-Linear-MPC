# Non-Linear-MPC Drag Racing

In this, we will design a nonlinear MPC controller to let the car to travel as far as possible in positive $x$ direction in a fixed time interval. 
We are supposed to keep close to $y = 0$, while avoiding the obstacle located at $x = 500, y = 0$. 
The key to this problem is to use as much tire forces as possible, while ensuring that the car does not lose control. 

# Car Dynamics

<div>
<img src="CarModel.png" width="1200">
</div>



We consider the car dynamics with lumped rear and front tire forces. 



For simplification, we consider the positions and orientation in the world coordinates.  

<p align="center"> <img src="https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Baligned%7D%5Cdot%7BU%7D_x%26%3D%5Cfrac%7BF_%7Bx_f%7D%5Ccos%5Cdelta-F_%7By_f%7D%5Csin%5Cdelta%2BF_%7Bx_r%7D-F_d%7D%7Bm%7D%2BrU_y%5C%5C%5Cdot%7BU%7D_y%26%3D%5Cfrac%7BF_%7By_f%7D%5Ccos%5Cdelta%2BF_%7Bx_f%7D%5Csin%5Cdelta%2BF_%7By_r%7D%2BF_b%7D%7Bm%7D-rU_x%5C%5C%5Cdot%7Br%7D%26%3D%5Cfrac%7Ba%5Cleft(F_%7By_f%7D%5Ccos%5Cdelta%2BF_%7Bx_f%7D%5Csin%5Cdelta%5Cright)-bF_%7By_r%7D%7D%7BI_%7Bzz%7D%7D%5C%5C%5Cdot%7Bx%7D%26%3D%5Ccos%28%5Cphi%29U_x-%5Csin%28%5Cphi%29U_y%5C%5C%5Cdot%7By%7D%26%3D%5Csin%28%5Cphi%29U_x%2B%5Csin%28%5Cphi%29U_y%5C%5C%5Cdot%7B%5Cphi%7D%26%3Dr.%5C%5C%5Cend%7Baligned%7D"> </p>




We consider the Euler first order integration to discretize the model. 

The actual control input of the vehicle is the traction force $F_w$ and steering angle $\delta$, which implicitly appears in the vehicle dynamics via the tire force $F_{x_f}$, $F_{y_f}$, $F_{x_r}$ and $F_{y_r}$. 

