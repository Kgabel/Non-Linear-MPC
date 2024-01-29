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

\begin{aligned}
\dot{U}_x & =\frac{F_{x_f} \cos \delta-F_{y_f} \sin \delta+F_{x_r}-F_d}{m}+r U_y \\
\dot{U}_y & =\frac{F_{y_f} \cos \delta+F_{x_f} \sin \delta+F_{y_r}+F_b}{m}-r U_x \\
\dot{r} & =\frac{a\left(F_{y_f} \cos \delta+F_{x_f} \sin \delta\right)-b F_{y_r}}{I_{z z}} \\
\dot{x} & = \cos(\phi)U_x - \sin(\phi)U_y  \\
\dot{y} & = \sin(\phi)U_x + \sin(\phi)U_y   \\ 
\dot{\phi} & =r  . \\
\end{aligned}




We consider the Euler first order integration to discretize the model. 

The actual control input of the vehicle is the traction force $F_w$ and steering angle $\delta$, which implicitly appears in the vehicle dynamics via the tire force $F_{x_f}$, $F_{y_f}$, $F_{x_r}$ and $F_{y_r}$. 

